#include "world.h"
#include "contact_listener.h"
#include "contact_solve_listener.h"

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace longeron {

namespace {

// Jolt's Factory + RegisterTypes are process-global and only need to
// run once. We use std::once_flag so multiple LongeronWorld instances
// don't fight over them.
std::once_flag g_jolt_init_flag;

static void LongeronTrace(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    std::fprintf(stderr, "[lng/JPH] ");
    std::vfprintf(stderr, fmt, args);
    std::fprintf(stderr, "\n");
    std::fflush(stderr);
    va_end(args);
}

#ifdef JPH_ENABLE_ASSERTS
static bool LongeronAssertFailed(const char* expr, const char* msg, const char* file, JPH::uint line) {
    std::fprintf(stderr, "[lng/JPH_ASSERT] %s:%u: %s%s%s\n", file, line, expr,
                 msg ? " — " : "", msg ? msg : "");
    std::fflush(stderr);
    return true;  // signal break/abort
}
#endif

void EnsureJoltGlobals() {
    std::call_once(g_jolt_init_flag, []() {
        JPH::RegisterDefaultAllocator();
        JPH::Trace = LongeronTrace;
#ifdef JPH_ENABLE_ASSERTS
        JPH::AssertFailed = LongeronAssertFailed;
#endif
        if (JPH::Factory::sInstance == nullptr) {
            JPH::Factory::sInstance = new JPH::Factory();
        }
        JPH::RegisterTypes();
    });
}

} // anonymous namespace

// Defined out-of-line so the call_once'd init runs before any other
// Jolt object's constructor in the member initializer list.
LongeronWorld::JoltGlobalsSentinel::JoltGlobalsSentinel() {
    EnsureJoltGlobals();
}

namespace {

// Read a value from a record stream, advancing the cursor. Stops at
// end-of-buffer; caller must handle truncation by checking the cursor.
template <typename T>
inline T Read(const uint8_t*& cur, const uint8_t* end) {
    T v{};
    if (cur + sizeof(T) <= end) {
        std::memcpy(&v, cur, sizeof(T));
        cur += sizeof(T);
    } else {
        cur = end;  // mark EOF
    }
    return v;
}

inline JPH::Vec3 ReadFloat3(const uint8_t*& cur, const uint8_t* end) {
    const float x = Read<float>(cur, end);
    const float y = Read<float>(cur, end);
    const float z = Read<float>(cur, end);
    return JPH::Vec3(x, y, z);
}

inline JPH::RVec3 ReadDouble3(const uint8_t*& cur, const uint8_t* end) {
    const double x = Read<double>(cur, end);
    const double y = Read<double>(cur, end);
    const double z = Read<double>(cur, end);
#ifdef JPH_DOUBLE_PRECISION
    return JPH::RVec3(x, y, z);
#else
    return JPH::RVec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
#endif
}

inline JPH::Quat ReadFloat4Quat(const uint8_t*& cur, const uint8_t* end) {
    const float x = Read<float>(cur, end);
    const float y = Read<float>(cur, end);
    const float z = Read<float>(cur, end);
    const float w = Read<float>(cur, end);
    return JPH::Quat(x, y, z, w);
}

template <typename T>
inline void Write(uint8_t*& cur, const T& v) {
    std::memcpy(cur, &v, sizeof(T));
    cur += sizeof(T);
}

} // anonymous namespace

LongeronWorld::LongeronWorld(const ::LongeronConfig& cfg)
    : mJoltGlobalsSentinel{}
    // 32 MB temp allocator — Jolt's HelloWorld uses 10 MB for trivial
    // scenes; under double-precision builds with even a couple of
    // bodies, 10 MB is exceeded during the broadphase/narrowphase
    // pass. Phase 2+ may revisit this once the working-set is
    // measured.
    , mTempAllocator(32 * 1024 * 1024)
    , mJobSystem(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
                 std::max(1u, std::thread::hardware_concurrency() - 1))
{
    // Default max_bodies bumped to 16384 to accommodate Phase 3b PQS
    // terrain streaming. A typical near-surface scene tops ~1000–4000
    // collider quads; combined with vessel parts and any future
    // KSC-static mirror, 16384 is comfortable headroom.
    const uint32_t max_bodies         = cfg.max_bodies      ? cfg.max_bodies      : 16384;
    const uint32_t max_constraints    = cfg.max_constraints ? cfg.max_constraints : 4096;
    const uint32_t num_body_mutexes   = 0;       // Jolt picks a sensible default
    const uint32_t max_body_pairs     = max_bodies * 8;
    const uint32_t max_contact_constraints = max_bodies * 4;

    mPhysicsSystem.Init(
        max_bodies,
        num_body_mutexes,
        max_body_pairs,
        max_contact_constraints,
        mBPLayerInterface,
        mObjectVsBPFilter,
        mObjectVsObjectFilter);

    // Default gravity: world supplies it (per-body via input records).
    // Disable Jolt's global gravity so it doesn't double-apply.
    mPhysicsSystem.SetGravity(JPH::Vec3(
        static_cast<float>(cfg.gravity_x),
        static_cast<float>(cfg.gravity_y),
        static_cast<float>(cfg.gravity_z)));

    // Solver iterations stay at Jolt defaults. Single-body-per-vessel
    // (Phase 2.x) means there are no intra-vessel constraints to
    // converge over — vessel motion is integrated as one rigid body,
    // and inter-vessel contacts are the only constraints PGS needs to
    // resolve. Defaults handle that fine.
    //
    // History (kept for reference): we briefly ran the multi-body
    // FixedConstraint chain at 24/8 → 64/32 → 128/64 → 256/128 to
    // brute-force PGS convergence on KSP-scale mass ratios. None of
    // those gave clean results; the right answer was to drop the
    // multi-body model entirely. Phase 4 (Featherstone ABA) computes
    // joint forces post-hoc for break detection without re-introducing
    // any iterative coupling.
    //
    // Defaults from Jolt's PhysicsSettings.h:
    //   mNumVelocitySteps = 10
    //   mNumPositionSteps = 2
    //   mBaumgarte = 0.2

    mContactListener = std::make_unique<ContactListenerImpl>(this);
    mPhysicsSystem.SetContactListener(mContactListener.get());

    // Phase 4.2: post-solve callback that routes per-contact resolved
    // impulses (lambdas from Jolt's velocity solver) into the per-part
    // external-wrench accumulator. Without it, RNEA joint forces are
    // upper-bound (suspended-vessel weight) when the vessel is in
    // contact with anything.
    mContactSolveListener = std::make_unique<ContactSolveListenerImpl>(this);
    mPhysicsSystem.SetContactSolveListener(mContactSolveListener.get());

    // 1 sub-group per vessel — every part of a vessel shares
    // sub_group_id = 0, so GroupFilterTable's "same group + same
    // subgroup" rule rejects intra-vessel collision pairs.
    mGroupFilter = new JPH::GroupFilterTable(1);

    (void)max_constraints;  // Phase 2+ when constraints land
}

LongeronWorld::~LongeronWorld() {
    // Remove constraints first (they reference bodies).
    for (auto& kv : mConstraintRegistry) {
        if (kv.second != nullptr) {
            mPhysicsSystem.RemoveConstraint(kv.second);
        }
    }
    mConstraintRegistry.clear();

    // Remove + destroy all bodies still registered.
    if (!mBodyRegistry.empty()) {
        JPH::BodyInterface& bi = mPhysicsSystem.GetBodyInterface();
        for (auto& kv : mBodyRegistry) {
            bi.RemoveBody(kv.second);
            bi.DestroyBody(kv.second);
        }
        mBodyRegistry.clear();
    }
}

uint32_t LongeronWorld::GetUserIdForBody(const JPH::Body& body) const {
    return static_cast<uint32_t>(body.GetUserData());
}

void LongeronWorld::AppendContactReport(
    uint32_t user_id_a, uint32_t user_id_b,
    JPH::RVec3 point, JPH::Vec3 normal,
    float depth, float impulse)
{
    // tag(1) + a(4) + b(4) + point(24 doubles) + normal(12) + depth(4) + impulse(4) = 53
    constexpr size_t kSize = 53;
    if (!ReserveOutput(kSize)) return;

    uint8_t* p = mOutputBuffer + mOutputLen;
    *p = static_cast<uint8_t>(RecordType::ContactReport);
    p += 1;
    Write(p, user_id_a);
    Write(p, user_id_b);
#ifdef JPH_DOUBLE_PRECISION
    const double px = point.GetX(), py = point.GetY(), pz = point.GetZ();
#else
    const double px = static_cast<double>(point.GetX());
    const double py = static_cast<double>(point.GetY());
    const double pz = static_cast<double>(point.GetZ());
#endif
    Write(p, px); Write(p, py); Write(p, pz);
    Write(p, normal.GetX()); Write(p, normal.GetY()); Write(p, normal.GetZ());
    Write(p, depth);
    Write(p, impulse);
    mOutputLen += kSize;
}

bool LongeronWorld::ReserveOutput(size_t bytes) {
    if (mOutputLen + bytes > mOutputCap) {
        mOutputOverflow = true;
        return false;
    }
    return true;
}

// --------------------------------------------------------------------
// Input handlers

// Build a single Jolt shape from a sub-shape record on the input
// stream. Advances `cur`. Returns null Ref on failure (logged via
// JPH::Trace which routes to the LongeronTrace handler).
static JPH::RefConst<JPH::Shape> BuildSubShape(const uint8_t*& cur, const uint8_t* end) {
    const uint8_t kind = Read<uint8_t>(cur, end);
    switch (static_cast<ShapeKind>(kind)) {
    case ShapeKind::Box: {
        const JPH::Vec3 half_extents = ReadFloat3(cur, end);
        return JPH::RefConst<JPH::Shape>(new JPH::BoxShape(half_extents));
    }
    case ShapeKind::Sphere: {
        const float radius = Read<float>(cur, end);
        return JPH::RefConst<JPH::Shape>(new JPH::SphereShape(radius));
    }
    case ShapeKind::ConvexHull: {
        const uint32_t raw_count = Read<uint32_t>(cur, end);
        const uint32_t count = raw_count > kMaxConvexHullVertices
                                  ? kMaxConvexHullVertices
                                  : raw_count;
        if (raw_count > kMaxConvexHullVertices) {
            JPH::Trace("ConvexHull truncated: %u verts capped at %u",
                       raw_count, kMaxConvexHullVertices);
        }
        JPH::Array<JPH::Vec3> verts;
        verts.reserve(count);
        for (uint32_t i = 0; i < count; ++i) {
            verts.push_back(ReadFloat3(cur, end));
        }
        // Skip any remaining truncated verts so the cursor lines up.
        for (uint32_t i = count; i < raw_count; ++i) {
            (void)ReadFloat3(cur, end);
        }
        JPH::ConvexHullShapeSettings settings(verts);
        JPH::Shape::ShapeResult result = settings.Create();
        if (result.HasError()) {
            JPH::Trace("ConvexHull create failed: %s", result.GetError().c_str());
            return JPH::RefConst<JPH::Shape>();
        }
        return result.Get();
    }
    case ShapeKind::TriangleMesh: {
        // Wire format:
        //   u32 vertex_count, vertex_count × float3 vertices,
        //   u32 triangle_count, triangle_count × u32×3 indices.
        // Used for streaming PQS terrain quads. Static-only — Jolt's
        // MeshShape isn't supported on Dynamic / Kinematic bodies
        // (Body::SetShape would assert), so the caller guarantees
        // BodyType::Static.
        const uint32_t vertex_count = Read<uint32_t>(cur, end);
        JPH::VertexList verts;
        verts.reserve(vertex_count);
        for (uint32_t i = 0; i < vertex_count; ++i) {
            const JPH::Vec3 v = ReadFloat3(cur, end);
            verts.push_back(JPH::Float3(v.GetX(), v.GetY(), v.GetZ()));
        }

        const uint32_t raw_tri_count = Read<uint32_t>(cur, end);
        const uint32_t tri_count = raw_tri_count > kMaxMeshTriangles
                                       ? kMaxMeshTriangles
                                       : raw_tri_count;
        if (raw_tri_count > kMaxMeshTriangles) {
            JPH::Trace("TriangleMesh truncated: %u tris capped at %u",
                       raw_tri_count, kMaxMeshTriangles);
        }
        JPH::IndexedTriangleList indices;
        indices.reserve(tri_count);
        for (uint32_t i = 0; i < tri_count; ++i) {
            const uint32_t i0 = Read<uint32_t>(cur, end);
            const uint32_t i1 = Read<uint32_t>(cur, end);
            const uint32_t i2 = Read<uint32_t>(cur, end);
            indices.push_back(JPH::IndexedTriangle(i0, i1, i2));
        }
        // Skip any remaining truncated triangles so the cursor lines up.
        for (uint32_t i = tri_count; i < raw_tri_count; ++i) {
            (void)Read<uint32_t>(cur, end);
            (void)Read<uint32_t>(cur, end);
            (void)Read<uint32_t>(cur, end);
        }

        if (verts.empty() || indices.empty()) {
            JPH::Trace("TriangleMesh: empty verts (%u) or tris (%u), skipping",
                       (unsigned)verts.size(), (unsigned)indices.size());
            return JPH::RefConst<JPH::Shape>();
        }

        JPH::MeshShapeSettings settings(verts, indices);
        JPH::Shape::ShapeResult result = settings.Create();
        if (result.HasError()) {
            JPH::Trace("TriangleMesh create failed: %s", result.GetError().c_str());
            return JPH::RefConst<JPH::Shape>();
        }
        return result.Get();
    }
    default:
        JPH::Trace("Unknown ShapeKind %u in BodyCreate", (unsigned)kind);
        return JPH::RefConst<JPH::Shape>();
    }
}

void LongeronWorld::HandleBodyCreate(const uint8_t*& cur, const uint8_t* end) {
    const uint32_t   user_id   = Read<uint32_t>(cur, end);
    const uint8_t    body_type = Read<uint8_t>(cur, end);
    const uint8_t    layer_id  = Read<uint8_t>(cur, end);
    const uint32_t   group_id  = Read<uint32_t>(cur, end);
    const JPH::RVec3 pos       = ReadDouble3(cur, end);
    const JPH::Quat  rot       = ReadFloat4Quat(cur, end);
    const float      mass      = Read<float>(cur, end);
    // Initial linear + angular velocity in CB-frame. Critical for
    // dynamic vessel bodies created on goOffRails: stock-side rb.velocity
    // is set during unpack (orbit propagation → Vessel.GoOffRails →
    // Part.SetWorldVelocity), and we must seed Jolt with that velocity
    // or the vessel pops back into physics at v=0 and re-accelerates
    // from zero. Static / kinematic ignore (kinematic motion is driven
    // by SetKinematicPose; static is immobile).
    const JPH::Vec3  lin_v     = ReadFloat3(cur, end);
    const JPH::Vec3  ang_v     = ReadFloat3(cur, end);
    const uint8_t    shape_count = Read<uint8_t>(cur, end);

    if (mBodyRegistry.find(user_id) != mBodyRegistry.end()) {
        JPH::Trace("BodyCreate: duplicate user_id %u, skipping", user_id);
        return;
    }
    if (shape_count == 0) {
        JPH::Trace("BodyCreate: shape_count=0 for user_id %u, skipping", user_id);
        return;
    }

    JPH::EMotionType motion_type;
    switch (static_cast<BodyType>(body_type)) {
    case BodyType::Static:    motion_type = JPH::EMotionType::Static; break;
    case BodyType::Kinematic: motion_type = JPH::EMotionType::Kinematic; break;
    case BodyType::Dynamic:   motion_type = JPH::EMotionType::Dynamic; break;
    default:
        JPH::Trace("BodyCreate: bad body_type %u", (unsigned)body_type);
        return;
    }

    JPH::ObjectLayer obj_layer;
    switch (static_cast<Layer>(layer_id)) {
    case Layer::Static:    obj_layer = ObjLayers::STATIC; break;
    case Layer::Kinematic: obj_layer = ObjLayers::KINEMATIC; break;
    default: obj_layer = (motion_type == JPH::EMotionType::Static)
                              ? ObjLayers::STATIC : ObjLayers::KINEMATIC; break;
    }

    // Read sub-shape records (transform + kind + params per shape).
    // If shape_count == 1 and the local transform is identity, use the
    // single shape directly. Otherwise wrap in a StaticCompoundShape.
    struct SubShape {
        JPH::Vec3 pos;
        JPH::Quat rot;
        JPH::RefConst<JPH::Shape> shape;
    };
    std::vector<SubShape> subs;
    subs.reserve(shape_count);
    for (uint8_t i = 0; i < shape_count; ++i) {
        SubShape s;
        s.pos   = ReadFloat3(cur, end);
        s.rot   = ReadFloat4Quat(cur, end);
        s.shape = BuildSubShape(cur, end);
        if (s.shape == nullptr) {
            JPH::Trace("BodyCreate: failed to build sub-shape %u for user_id %u",
                       (unsigned)i, user_id);
            return;
        }
        subs.push_back(std::move(s));
    }

    // Dynamic bodies are vessels — they need MutableCompoundShape so
    // ABA can ModifyShapes per tick to track per-part flex. Static and
    // kinematic bodies keep StaticCompoundShape (terrain, KSC props,
    // etc.) since their sub-shape geometry never changes after create.
    //
    // Single-shape bodies bypass the compound entirely (no flex is
    // possible with only one part anyway — single-part vessels stay
    // rigid by construction), but only when the local transform is
    // identity; otherwise we still need a compound to carry the
    // offset.
    JPH::RefConst<JPH::Shape> body_shape;
    const bool needs_mutable = (motion_type == JPH::EMotionType::Dynamic);
    if (subs.size() == 1
        && subs[0].pos == JPH::Vec3::sZero()
        && subs[0].rot == JPH::Quat::sIdentity()
        && !needs_mutable)
    {
        body_shape = subs[0].shape;
    } else if (needs_mutable) {
        JPH::MutableCompoundShapeSettings compound;
        for (const auto& s : subs) {
            compound.AddShape(s.pos, s.rot, s.shape);
        }
        JPH::Shape::ShapeResult result = compound.Create();
        if (result.HasError()) {
            JPH::Trace("BodyCreate: MutableCompoundShape failed: %s for user_id %u",
                       result.GetError().c_str(), user_id);
            return;
        }
        body_shape = result.Get();
    } else {
        JPH::StaticCompoundShapeSettings compound;
        for (const auto& s : subs) {
            compound.AddShape(s.pos, s.rot, s.shape);
        }
        JPH::Shape::ShapeResult result = compound.Create();
        if (result.HasError()) {
            JPH::Trace("BodyCreate: StaticCompoundShape failed: %s for user_id %u",
                       result.GetError().c_str(), user_id);
            return;
        }
        body_shape = result.Get();
    }

    JPH::BodyCreationSettings settings(body_shape, pos, rot, motion_type, obj_layer);
    settings.mUserData = static_cast<uint64_t>(user_id);
    settings.mLinearVelocity  = lin_v;
    settings.mAngularVelocity = ang_v;

    // Kinematic-vs-static and kinematic-vs-kinematic contact callbacks
    // are gated behind this flag in Jolt — Body.inl::sFindCollidingPairsCanCollide.
    // Without it, our entire architecture silently produces no
    // contacts. Set it on every body we register.
    settings.mCollideKinematicVsNonDynamic = true;

    // Jolt clamps body velocities to mMaxLinearVelocity (default
    // 500 m/s) / mMaxAngularVelocity (default 0.25π·60 rad/s ≈
    // 47 rad/s). 500 m/s is fine for game physics but caps KSP
    // vessels at ~Mach 1.5 — a vessel hitting 500 m/s in atmospheric
    // flight or accelerating toward orbital (~7800 m/s on Kerbin)
    // would silently stop accelerating. Bump well above any realistic
    // KSP scenario; interplanetary transfer dVs are tens of km/s.
    settings.mMaxLinearVelocity = 100000.0f;   // 100 km/s
    settings.mMaxAngularVelocity = 200.0f;     // ≈ 32 rev/s

    // Jolt's defaults bake in implicit linear/angular damping
    // (mLinearDamping = mAngularDamping = 0.05) — `dv/dt = -c*v`.
    // At v=600 m/s that's 30 m/s² of "drag" that doesn't come from
    // anywhere physical, on top of stock KSP's atmospheric drag (which
    // we already redirect through Harmony hooks).
    //
    // Stock KSP leaves rb.drag = 0 (PhysX equivalent of mLinearDamping).
    // It does set rb.angularDrag dynamically per-part for atmospheric
    // angular drag, but that's also not what Jolt's mAngularDamping
    // models. Zero both — let Jolt integrate strictly the forces we
    // give it, no implicit decay.
    settings.mLinearDamping  = 0.0f;
    settings.mAngularDamping = 0.0f;

    // Per-vessel collision group: bodies sharing a non-zero group_id
    // skip collision with each other (sub_group_id = 0 for every
    // part within a vessel, GroupFilterTable rejects same-subgroup
    // pairs). group_id = cInvalidGroup makes the body collide with
    // everything — used for terrain / synthetic ground / global
    // static geometry.
    if (group_id == 0) {
        settings.mCollisionGroup = JPH::CollisionGroup(
            mGroupFilter, JPH::CollisionGroup::cInvalidGroup, 0);
    } else {
        settings.mCollisionGroup = JPH::CollisionGroup(mGroupFilter, group_id, 0);
    }

    if (motion_type == JPH::EMotionType::Dynamic) {
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = mass;
    }

    JPH::BodyInterface& bi = mPhysicsSystem.GetBodyInterface();
    JPH::BodyID id = bi.CreateAndAddBody(
        settings,
        motion_type == JPH::EMotionType::Static
            ? JPH::EActivation::DontActivate
            : JPH::EActivation::Activate);
    if (id.IsInvalid()) {
        JPH::Trace("BodyCreate: CreateAndAddBody returned invalid id for user_id %u", user_id);
        return;
    }

    mBodyRegistry.emplace(user_id, id);
    mNeedsBroadphaseOptimize = true;
}

void LongeronWorld::HandleBodyDestroy(const uint8_t*& cur, const uint8_t* end) {
    const uint32_t user_id = Read<uint32_t>(cur, end);
    auto it = mBodyRegistry.find(user_id);
    if (it == mBodyRegistry.end()) return;

    JPH::BodyInterface& bi = mPhysicsSystem.GetBodyInterface();
    bi.RemoveBody(it->second);
    bi.DestroyBody(it->second);
    mBodyRegistry.erase(it);
    mTreeRegistry.Erase(user_id);
}

void LongeronWorld::HandleSetGravity(const uint8_t*& cur, const uint8_t* end) {
    const double gx = Read<double>(cur, end);
    const double gy = Read<double>(cur, end);
    const double gz = Read<double>(cur, end);
    mPhysicsSystem.SetGravity(JPH::Vec3(
        static_cast<float>(gx),
        static_cast<float>(gy),
        static_cast<float>(gz)));
}

void LongeronWorld::HandleSetKinematicPose(const uint8_t*& cur, const uint8_t* end) {
    const uint32_t user_id = Read<uint32_t>(cur, end);
    const JPH::RVec3 pos   = ReadDouble3(cur, end);
    const JPH::Quat  rot   = ReadFloat4Quat(cur, end);
    const JPH::Vec3  lin_v = ReadFloat3(cur, end);
    const JPH::Vec3  ang_v = ReadFloat3(cur, end);

    auto it = mBodyRegistry.find(user_id);
    if (it == mBodyRegistry.end()) return;

    JPH::BodyInterface& bi = mPhysicsSystem.GetBodyInterface();
    bi.SetPositionAndRotation(it->second, pos, rot, JPH::EActivation::Activate);
    bi.SetLinearAndAngularVelocity(it->second, lin_v, ang_v);
}

void LongeronWorld::HandleConstraintCreate(const uint8_t*& cur, const uint8_t* end) {
    const uint32_t constraint_id = Read<uint32_t>(cur, end);
    const uint8_t  kind          = Read<uint8_t>(cur, end);
    const uint32_t body_a_id     = Read<uint32_t>(cur, end);
    const uint32_t body_b_id     = Read<uint32_t>(cur, end);

    if (mConstraintRegistry.find(constraint_id) != mConstraintRegistry.end()) {
        JPH::Trace("ConstraintCreate: duplicate id %u, skipping", constraint_id);
        return;
    }

    auto it_a = mBodyRegistry.find(body_a_id);
    auto it_b = mBodyRegistry.find(body_b_id);
    if (it_a == mBodyRegistry.end() || it_b == mBodyRegistry.end()) {
        JPH::Trace("ConstraintCreate %u: body lookup failed (a=%u found=%d, b=%u found=%d)",
                   constraint_id, body_a_id, it_a != mBodyRegistry.end(),
                   body_b_id, it_b != mBodyRegistry.end());
        return;
    }

    // Need direct Body* pointers (not BodyIDs) for constraint
    // creation. We call this from the input-parse phase, before
    // PhysicsSystem::Update — no concurrent access — so the
    // no-lock interface is safe.
    const JPH::BodyLockInterfaceNoLock& lock = mPhysicsSystem.GetBodyLockInterfaceNoLock();
    JPH::Body* body_a = lock.TryGetBody(it_a->second);
    JPH::Body* body_b = lock.TryGetBody(it_b->second);
    if (body_a == nullptr || body_b == nullptr) {
        JPH::Trace("ConstraintCreate %u: body fetch returned null", constraint_id);
        return;
    }

    JPH::Ref<JPH::Constraint> constraint;
    switch (static_cast<ConstraintKind>(kind)) {
    case ConstraintKind::Fixed: {
        JPH::FixedConstraintSettings settings;
        // AutoDetectPoint computes a sensible anchor from the two
        // bodies' current poses — no need for the C# side to compute
        // exact attachment node positions.
        settings.mAutoDetectPoint = true;
        constraint = settings.Create(*body_a, *body_b);
        break;
    }
    default:
        JPH::Trace("ConstraintCreate %u: unknown kind %u", constraint_id, (unsigned)kind);
        return;
    }

    if (constraint == nullptr) {
        JPH::Trace("ConstraintCreate %u: settings.Create returned null", constraint_id);
        return;
    }

    mPhysicsSystem.AddConstraint(constraint);
    mConstraintRegistry.emplace(constraint_id, constraint);
}

void LongeronWorld::HandleConstraintCreateFixedAt(const uint8_t*& cur, const uint8_t* end) {
    const uint32_t constraint_id = Read<uint32_t>(cur, end);
    const uint8_t  kind          = Read<uint8_t>(cur, end);
    const uint32_t body_a_id     = Read<uint32_t>(cur, end);
    const uint32_t body_b_id     = Read<uint32_t>(cur, end);
    const JPH::RVec3 anchor      = ReadDouble3(cur, end);

    if (mConstraintRegistry.find(constraint_id) != mConstraintRegistry.end()) {
        JPH::Trace("ConstraintCreateFixedAt: duplicate id %u, skipping", constraint_id);
        return;
    }

    auto it_a = mBodyRegistry.find(body_a_id);
    auto it_b = mBodyRegistry.find(body_b_id);
    if (it_a == mBodyRegistry.end() || it_b == mBodyRegistry.end()) {
        JPH::Trace("ConstraintCreateFixedAt %u: body lookup failed (a=%u found=%d, b=%u found=%d)",
                   constraint_id, body_a_id, it_a != mBodyRegistry.end(),
                   body_b_id, it_b != mBodyRegistry.end());
        return;
    }

    const JPH::BodyLockInterfaceNoLock& lock = mPhysicsSystem.GetBodyLockInterfaceNoLock();
    JPH::Body* body_a = lock.TryGetBody(it_a->second);
    JPH::Body* body_b = lock.TryGetBody(it_b->second);
    if (body_a == nullptr || body_b == nullptr) {
        JPH::Trace("ConstraintCreateFixedAt %u: body fetch returned null", constraint_id);
        return;
    }

    JPH::Ref<JPH::Constraint> constraint;
    switch (static_cast<ConstraintKind>(kind)) {
    case ConstraintKind::Fixed: {
        // Same FixedConstraint primitive as the autoDetect path, but
        // with an explicit world-space anchor. Both bodies resolve
        // their local-CoM offsets from this world point at construction;
        // the constraint then enforces "lock relative pose" exactly as
        // the autoDetect variant. The win is purely solver conditioning:
        // anchor at the actual KSP attach node (e.g., between a tank
        // and a radial decoupler) rather than at the midpoint between
        // the bodies' CoMs (which lands ~0.5 m inside the tank).
        JPH::FixedConstraintSettings settings;
        settings.mSpace = JPH::EConstraintSpace::WorldSpace;
        settings.mAutoDetectPoint = false;
        settings.mPoint1 = anchor;
        settings.mPoint2 = anchor;
        constraint = settings.Create(*body_a, *body_b);
        break;
    }
    default:
        JPH::Trace("ConstraintCreateFixedAt %u: unknown kind %u", constraint_id, (unsigned)kind);
        return;
    }

    if (constraint == nullptr) {
        JPH::Trace("ConstraintCreateFixedAt %u: settings.Create returned null", constraint_id);
        return;
    }

    mPhysicsSystem.AddConstraint(constraint);
    mConstraintRegistry.emplace(constraint_id, constraint);
}

void LongeronWorld::HandleShiftWorld(const uint8_t*& cur, const uint8_t* end) {
    const double dx = Read<double>(cur, end);
    const double dy = Read<double>(cur, end);
    const double dz = Read<double>(cur, end);
#ifdef JPH_DOUBLE_PRECISION
    const JPH::RVec3 delta(dx, dy, dz);
#else
    const JPH::RVec3 delta(static_cast<float>(dx), static_cast<float>(dy), static_cast<float>(dz));
#endif

    // KSP shifted Unity's world origin (Krakensbane / FloatingOrigin).
    // Translate every body in absolute Jolt coords by the same delta
    // so Jolt's pose readback continues to match Unity's expected
    // view. Velocities are unaffected (a translation doesn't change
    // them).
    JPH::BodyInterface& bi = mPhysicsSystem.GetBodyInterface();
    for (const auto& kv : mBodyRegistry) {
        JPH::RVec3 p = bi.GetPosition(kv.second);
        bi.SetPosition(kv.second, p + delta, JPH::EActivation::DontActivate);
    }
}

void LongeronWorld::HandleConstraintDestroy(const uint8_t*& cur, const uint8_t* end) {
    const uint32_t constraint_id = Read<uint32_t>(cur, end);
    auto it = mConstraintRegistry.find(constraint_id);
    if (it == mConstraintRegistry.end()) return;
    if (it->second != nullptr) {
        mPhysicsSystem.RemoveConstraint(it->second);
    }
    mConstraintRegistry.erase(it);
}

void LongeronWorld::HandleMassUpdate(const uint8_t*& cur, const uint8_t* end) {
    const uint32_t user_id = Read<uint32_t>(cur, end);
    const float    mass    = Read<float>(cur, end);

    if (mass <= 0.0f) return;

    auto it = mBodyRegistry.find(user_id);
    if (it == mBodyRegistry.end()) return;

    const JPH::BodyLockInterfaceNoLock& lock = mPhysicsSystem.GetBodyLockInterfaceNoLock();
    JPH::Body* body = lock.TryGetBody(it->second);
    if (body == nullptr) return;

    JPH::MotionProperties* mp = body->GetMotionProperties();
    if (mp == nullptr) return;  // static body

    // Recompute mass properties from the shape with the new mass —
    // this scales the inertia tensor proportionally rather than just
    // updating inverse mass. Without rescaling inertia, angular
    // acceleration would stay anchored to the launch-mass tensor and
    // burns wouldn't visibly change rotational responsiveness.
    const JPH::Shape* shape = body->GetShape();
    if (shape == nullptr) return;

    JPH::MassProperties props = shape->GetMassProperties();
    props.ScaleToMass(mass);
    mp->SetMassProperties(JPH::EAllowedDOFs::All, props);
}

void LongeronWorld::HandleVesselTreeUpdate(const uint8_t*& cur, const uint8_t* end) {
    const uint32_t body_id    = Read<uint32_t>(cur, end);
    const uint16_t part_count = Read<uint16_t>(cur, end);

    if (part_count == 0) {
        mTreeRegistry.Erase(body_id);
        return;
    }
    if (part_count > kMaxPartsPerVessel) {
        JPH::Trace("VesselTreeUpdate: part_count=%u exceeds cap %u for body %u",
                   (unsigned)part_count, (unsigned)kMaxPartsPerVessel, body_id);
        // Skip the entire payload to keep the parser aligned.
        const size_t payload_size = static_cast<size_t>(part_count)
            * (2 + 4 + 12 + 12 + 12 + 16);  // u16 + float + 3 × float3 + 4 × float
        if (cur + payload_size <= end) cur += payload_size;
        else cur = end;
        return;
    }

    std::vector<PartNode> nodes;
    std::vector<EdgeCompliance> compliance;
    nodes.reserve(part_count);
    compliance.reserve(part_count);
    for (uint16_t i = 0; i < part_count; ++i) {
        PartNode n;
        n.parent_idx   = Read<uint16_t>(cur, end);
        n.mass         = Read<float>(cur, end);
        n.com_local    = ReadFloat3(cur, end);
        n.inertia_diag = ReadFloat3(cur, end);
        n.attach_local = ReadFloat3(cur, end);
        EdgeCompliance c;
        c.k_lin = Read<float>(cur, end);
        c.c_lin = Read<float>(cur, end);
        c.k_ang = Read<float>(cur, end);
        c.c_ang = Read<float>(cur, end);
        // Validate parent: must be < i (topology order) or kInvalidPartIdx.
        if (n.parent_idx != kInvalidPartIdx && n.parent_idx >= i) {
            JPH::Trace("VesselTreeUpdate: bad parent_idx %u at node %u (must be < node)",
                       (unsigned)n.parent_idx, (unsigned)i);
            return;  // Drop the whole tree on malformed topology.
        }
        nodes.push_back(n);
        compliance.push_back(c);
    }

    mTreeRegistry.Upsert(body_id, std::move(nodes), std::move(compliance));
}

void LongeronWorld::HandleSetBodyGroup(const uint8_t*& cur, const uint8_t* end) {
    const uint32_t user_id = Read<uint32_t>(cur, end);
    const uint32_t group_id = Read<uint32_t>(cur, end);

    auto it = mBodyRegistry.find(user_id);
    if (it == mBodyRegistry.end()) {
        JPH::Trace("SetBodyGroup: unknown body %u", user_id);
        return;
    }

    // Direct Body* update — runs from the input-parse phase before
    // PhysicsSystem::Update, so the no-lock interface is safe.
    const JPH::BodyLockInterfaceNoLock& lock = mPhysicsSystem.GetBodyLockInterfaceNoLock();
    JPH::Body* body = lock.TryGetBody(it->second);
    if (body == nullptr) {
        JPH::Trace("SetBodyGroup: body fetch returned null for user_id %u", user_id);
        return;
    }

    if (group_id == 0) {
        body->SetCollisionGroup(JPH::CollisionGroup(
            mGroupFilter, JPH::CollisionGroup::cInvalidGroup, 0));
    } else {
        body->SetCollisionGroup(JPH::CollisionGroup(mGroupFilter, group_id, 0));
    }
}

void LongeronWorld::HandleForceDelta(const uint8_t*& cur, const uint8_t* end) {
    const uint32_t user_id = Read<uint32_t>(cur, end);
    const double fx = Read<double>(cur, end);
    const double fy = Read<double>(cur, end);
    const double fz = Read<double>(cur, end);
    const double tx = Read<double>(cur, end);
    const double ty = Read<double>(cur, end);
    const double tz = Read<double>(cur, end);

    auto it = mBodyRegistry.find(user_id);
    if (it == mBodyRegistry.end()) return;

    JPH::BodyInterface& bi = mPhysicsSystem.GetBodyInterface();
    bi.AddForce(it->second, JPH::Vec3(
        static_cast<float>(fx), static_cast<float>(fy), static_cast<float>(fz)));
    bi.AddTorque(it->second, JPH::Vec3(
        static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz)));
}

void LongeronWorld::HandleForceAtPosition(const uint8_t*& cur, const uint8_t* end) {
    const uint32_t user_id = Read<uint32_t>(cur, end);
    const JPH::Vec3 force  = JPH::Vec3(
        static_cast<float>(Read<double>(cur, end)),
        static_cast<float>(Read<double>(cur, end)),
        static_cast<float>(Read<double>(cur, end)));
    const JPH::RVec3 point = ReadDouble3(cur, end);
    const uint16_t  part_idx = Read<uint16_t>(cur, end);

    auto it = mBodyRegistry.find(user_id);
    if (it == mBodyRegistry.end()) return;

    // BodyInterface::AddForce(BodyID, Vec3 force, RVec3 point) computes
    // the implicit torque (point - CoM) × force internally.
    JPH::BodyInterface& bi = mPhysicsSystem.GetBodyInterface();
    bi.AddForce(it->second, force, point);

    // Phase 4: route the force to the per-part external-wrench
    // accumulator so RNEA can subtract it from the inertial wrench
    // when computing per-edge transmitted force. Skip if part_idx
    // is unattributed (0xFFFF) or the body has no tree (static
    // bodies, single-part vessels we haven't sent a tree for, etc.).
    if (part_idx != kInvalidPartIdx) {
        const JPH::BodyLockInterfaceNoLock& lock = mPhysicsSystem.GetBodyLockInterfaceNoLock();
        const JPH::Body* body = lock.TryGetBody(it->second);
        if (body != nullptr) {
            mTreeRegistry.AccumulateExternalForce(
                user_id, part_idx, force, point,
                body->GetCenterOfMassPosition(),
                body->GetRotation());
        }
    }
}

// --------------------------------------------------------------------
// Output emission

void LongeronWorld::EmitBodyPoses() {
    JPH::BodyInterface& bi = mPhysicsSystem.GetBodyInterface();
    // tag(1) + user_id(4) + pos(24) + rot(16) + lin_v(12) + ang_v(12) = 69
    constexpr size_t kSize = 69;

    for (const auto& kv : mBodyRegistry) {
        if (!ReserveOutput(kSize)) return;

        const uint32_t user_id = kv.first;
        const JPH::BodyID id   = kv.second;

        JPH::RVec3 pos = bi.GetPosition(id);
        JPH::Quat  rot = bi.GetRotation(id);
        JPH::Vec3  lin = bi.GetLinearVelocity(id);
        JPH::Vec3  ang = bi.GetAngularVelocity(id);

        uint8_t* p = mOutputBuffer + mOutputLen;
        *p = static_cast<uint8_t>(RecordType::BodyPose);
        p += 1;
        Write(p, user_id);
#ifdef JPH_DOUBLE_PRECISION
        const double px = pos.GetX(), py = pos.GetY(), pz = pos.GetZ();
#else
        const double px = static_cast<double>(pos.GetX());
        const double py = static_cast<double>(pos.GetY());
        const double pz = static_cast<double>(pos.GetZ());
#endif
        Write(p, px); Write(p, py); Write(p, pz);
        Write(p, rot.GetX()); Write(p, rot.GetY()); Write(p, rot.GetZ()); Write(p, rot.GetW());
        Write(p, lin.GetX()); Write(p, lin.GetY()); Write(p, lin.GetZ());
        Write(p, ang.GetX()); Write(p, ang.GetY()); Write(p, ang.GetZ());
        mOutputLen += kSize;
    }
}

// --------------------------------------------------------------------
// Step

int32_t LongeronWorld::Step(
    const uint8_t* input, size_t input_len,
    uint8_t* output, size_t output_cap, size_t* output_len,
    float dt)
{
    static const bool kDiag = []() {
        const char* e = std::getenv("LONGERON_NATIVE_DIAG");
        return e && e[0] == '1';
    }();
    if (kDiag) std::fprintf(stderr, "[lng] Step: input_len=%zu, dt=%f, bodies=%zu\n",
                            input_len, dt, mBodyRegistry.size());

    mOutputBuffer   = output;
    mOutputCap      = output_cap;
    mOutputLen      = 0;
    mOutputOverflow = false;
    mCurrentDt      = dt;

    // 1. Apply input records.
    const uint8_t* cur = input;
    const uint8_t* end = input + input_len;
    while (cur < end) {
        const uint8_t tag = *cur++;
        switch (static_cast<RecordType>(tag)) {
        case RecordType::VesselTreeUpdate:  HandleVesselTreeUpdate(cur, end); break;
        case RecordType::BodyCreate:        HandleBodyCreate(cur, end); break;
        case RecordType::BodyDestroy:       HandleBodyDestroy(cur, end); break;
        case RecordType::SetGravity:        HandleSetGravity(cur, end); break;
        case RecordType::SetKinematicPose:  HandleSetKinematicPose(cur, end); break;
        case RecordType::ForceDelta:        HandleForceDelta(cur, end); break;
        case RecordType::ForceAtPosition:   HandleForceAtPosition(cur, end); break;
        case RecordType::ConstraintCreate:  HandleConstraintCreate(cur, end); break;
        case RecordType::ConstraintCreateFixedAt: HandleConstraintCreateFixedAt(cur, end); break;
        case RecordType::ConstraintDestroy: HandleConstraintDestroy(cur, end); break;
        case RecordType::ShiftWorld:        HandleShiftWorld(cur, end); break;
        case RecordType::SetBodyGroup:      HandleSetBodyGroup(cur, end); break;
        case RecordType::MassUpdate:        HandleMassUpdate(cur, end); break;
        default:
            // Unknown / unsupported in Phase 1 — abort to avoid mis-
            // parsing the remainder of the stream.
            return -3;
        }
    }

    if (kDiag) std::fprintf(stderr, "[lng] before Update, bodies=%zu\n", mBodyRegistry.size());

    // OptimizeBroadPhase is recommended after batch body inserts —
    // it rebuilds the broad-phase tree for added bodies. Cheap with
    // small N; should not be called every tick once we have many
    // bodies, but for Phase 1 with a handful of bodies it's fine.
    if (mNeedsBroadphaseOptimize) {
        mPhysicsSystem.OptimizeBroadPhase();
        mNeedsBroadphaseOptimize = false;
    }

    // 2. Advance Jolt. Single collision sub-step per FixedUpdate; KSP
    //    runs at 50 Hz so dt ≈ 0.02s, well within Jolt's recommended
    //    ≤ 1/60 s for stability.
    constexpr int kCollisionSubsteps = 1;
    mPhysicsSystem.Update(dt, kCollisionSubsteps, &mTempAllocator, &mJobSystem);

    if (kDiag) std::fprintf(stderr, "[lng] after Update, output_len=%zu\n", mOutputLen);

    // 3a. Phase 5 ABA forward pass — integrate per-part flex state
    //     under spring-damper joint forces + flex residual of external
    //     wrenches. Reads vessel CoM kinematics from Jolt (post-step),
    //     writes new per-part (delta_pos, delta_rot, vels) to each
    //     tree's flex[] vector.
    mTreeRegistry.RunAbaPass(mPhysicsSystem, mBodyRegistry, dt);

    // 3b. Apply the new flex state to the vessel's MutableCompoundShape
    //     so collision geometry tracks visible flex on the next tick's
    //     broadphase. Single body's outer-AABB refit; cheap unless
    //     vessels have many active contacts.
    mTreeRegistry.ApplyFlexToBodies(mPhysicsSystem, mBodyRegistry);

    // 3c. Emit PartPose records — one per non-root part. Emitted BEFORE
    //     BodyPose so the C# drain populates JoltPart.FlexLocal* values
    //     before ApplyVesselPose runs, letting the per-part rb update
    //     compose the up-to-date flex this tick.
    //     35 bytes per part: tag(1) + body_id(4) + part_idx(2)
    //                        + delta_pos(12) + delta_rot(16).
    {
        constexpr size_t kSize = 35;
        for (const auto& [body_id, tree] : mTreeRegistry.GetTrees()) {
            if (tree.nodes.size() < 2) continue;
            for (uint16_t i = 1; i < tree.nodes.size(); ++i) {
                const auto& f = tree.flex[i];
                if (!ReserveOutput(kSize)) break;
                uint8_t* p = mOutputBuffer + mOutputLen;
                *p = static_cast<uint8_t>(RecordType::PartPose); p += 1;
                Write(p, body_id);
                Write(p, i);
                Write(p, f.delta_pos.GetX());
                Write(p, f.delta_pos.GetY());
                Write(p, f.delta_pos.GetZ());
                Write(p, f.delta_rot.GetX());
                Write(p, f.delta_rot.GetY());
                Write(p, f.delta_rot.GetZ());
                Write(p, f.delta_rot.GetW());
                mOutputLen += kSize;
            }
        }
    }

    // 3d. Emit body poses now that flex is applied + part poses are
    //     queued. The C# drain order is: PartPose updates JoltPart's
    //     FlexLocal*; BodyPose runs ApplyVesselPose composing the
    //     fresh flex.
    EmitBodyPoses();

    // 4. Advisory RNEA pass over per-vessel spanning trees. Uses the
    //     deflected geometry (com_local + delta_pos) so transmitted
    //     wrench reflects current loaded shape. Reuses cached
    //     a_body / alpha computed by RunAbaPass — no double finite-diff.
    const bool summary_ready = mTreeRegistry.RunAdvisoryPass(
        mPhysicsSystem, mBodyRegistry, mStepCount, dt);

    // Per-edge wrench records in joint frame — emitted every tick so
    // PartModules see fresh joint stress on the next FixedUpdate.
    // 43 bytes per edge:
    //   tag(1) + body_id(4) + part_idx(2)
    //   + force(float3=12, X=axial / YZ=shear)
    //   + torque(float3=12, X=torsion / YZ=bending)
    //   + ext_force(float3=12, body axes — diag).
    {
        constexpr size_t kSize = 43;
        for (const auto& w : mTreeRegistry.GetLastEdgeWrenches()) {
            if (!ReserveOutput(kSize)) break;
            uint8_t* p = mOutputBuffer + mOutputLen;
            *p = static_cast<uint8_t>(RecordType::JointWrench); p += 1;
            Write(p, w.body_id);
            Write(p, w.part_idx);
            Write(p, w.force.GetX());
            Write(p, w.force.GetY());
            Write(p, w.force.GetZ());
            Write(p, w.torque.GetX());
            Write(p, w.torque.GetY());
            Write(p, w.torque.GetZ());
            Write(p, w.ext_force.GetX());
            Write(p, w.ext_force.GetY());
            Write(p, w.ext_force.GetZ());
            mOutputLen += kSize;
        }
    }

    if (summary_ready) {
        // 45 bytes per summary: tag(1) + body_id(4) + part_count(2)
        //   + 5 × {value(4) + idx(2)} for compression / tension /
        //   shear / torsion / bending = 30
        //   + accel_mag(4) + alpha_mag(4).
        constexpr size_t kSize = 45;
        for (const auto& s : mTreeRegistry.GetLastSummaries()) {
            if (!ReserveOutput(kSize)) break;
            uint8_t* p = mOutputBuffer + mOutputLen;
            *p = static_cast<uint8_t>(RecordType::RneaSummary); p += 1;
            Write(p, s.body_id);
            Write(p, s.part_count);
            Write(p, s.max_compression); Write(p, s.max_compression_idx);
            Write(p, s.max_tension);     Write(p, s.max_tension_idx);
            Write(p, s.max_shear);       Write(p, s.max_shear_idx);
            Write(p, s.max_torsion);     Write(p, s.max_torsion_idx);
            Write(p, s.max_bending);     Write(p, s.max_bending_idx);
            Write(p, s.accel_mag);
            Write(p, s.alpha_mag);
            mOutputLen += kSize;
        }
    }

    *output_len = mOutputLen;
    ++mStepCount;
    if (mOutputOverflow) return -2;
    return 0;
}

} // namespace longeron
