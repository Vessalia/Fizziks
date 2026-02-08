#pragma once
#include <Fizziks/RigidBodyImpl.h>
#include <Fizziks/BroadPhase.h>
#include <Fizziks/Handle.h>
#include <Fizziks/ContactKey.h>
#include <Fizziks/ThreadPool.h>
#include <Fizziks/Allocator.h>

namespace Fizziks::internal
{
class FizzWorldImpl 
{
    friend class FizzWorld;
    friend class RigidBody;
    friend class RigidBodyImpl;

private:
    FizzWorldImpl(size_t unitsX, size_t unitsY, int collisionIterations, val_t timestep, FizzWorld::AccelStruct accel);
    ~FizzWorldImpl();

    size_t currstep = 0;
    static constexpr size_t THREAD_THRESHOLD = 1000;

    struct BodyData
    {
        uint32_t layer;

        Vec2 centroid;

        Vec2 position, velocity, accumForce;
        val_t rotation, angularVelocity, accumTorque;

        val_t mass, invMass;
        val_t MoI, invMoI;

        val_t gravityScale;

        val_t restitution;
        val_t staticFriction;
        val_t dynamicFriction;
        val_t linearDamping;
        val_t angularDamping;

        AABB bounds;

        std::vector<Collider> colliders;

        BodyType bodyType;
    };

    struct ColliderContact
    {
        uint32_t collAId, collBId;
        Contact contact;
    };

    struct CollisionManifold
    {
        size_t bodyAId, bodyBId;

        Allocator::Block allocContact;
        size_t numBlocks = 0;
        size_t allocIndex;
    };

    struct CollisionResolution
    {
        uint32_t bodyAId, bodyBId;
        uint32_t collIdA, collIdB;
        Contact contact;

        val_t normalImpulse;
        val_t tangentImpulse;

        val_t invEffMass;
        val_t invEffTangentMass;
        val_t bias;
    };

    static const BodyData null_body;

    ThreadPool threads;

    val_t timestep;
    val_t accumulator;

    size_t unitsX;
    size_t unitsY;

    int collisionIterations;

    std::vector<Handle> activeHandles;
    std::vector<uint32_t> freeList;

    std::vector<uint32_t> activeList;
    std::vector<BodyData> activeBodies;

    std::vector<Allocator*> threadAllocators;

    std::vector<CollisionManifold> collisionManifolds;
    std::vector<CollisionResolution> collisionResolutions;
    std::unordered_map<ContactKey, CollisionResolution> warmStartCache;

    std::queue<RigidBodyImpl> destructionQueue;

    Broadphase* broadphase;

    Vec2 get_worldPos(const BodyData& body, const Vec2& colliderPos) const;
    val_t get_worldRotation(const BodyData& body, const Collider& collider) const;

    CollisionManifold get_manifold(size_t idA, size_t idB, size_t allocIndex) const;
    void detect_collisions();
    ContactKey makeContactKey(const CollisionResolution& resolution) const;
    CollisionResolution collision_preStep(uint32_t idA, uint32_t idB, uint32_t collIdA, uint32_t collIdB, const Contact& constact, val_t dt);
    void solve_normalConstraint(CollisionResolution& resolution);
    void solve_frictionConstraint(CollisionResolution& resolution);
    void solve_contactConstraints(CollisionResolution& resolution);
    void resolve_collisions(val_t dt);

    void simulate_bodies(val_t dt, const Vec2& gravity);
    void handle_collisions(val_t dt);
    void destroy_bodies();

    void tick_end();

    void set_body(const RigidBodyImpl& rb, const BodyDef& def);
    void set_body(BodyData* body, const BodyDef& def);

    const BodyData* get_body(const RigidBodyImpl& handle) const;
    BodyData* get_body(const RigidBodyImpl& handle);

    void add_collider(const RigidBodyImpl& rb, const Collider& collider);
    void add_collider(BodyData* body, const Collider& collider);
    std::vector<Collider> body_colliders(const RigidBodyImpl& rb) const;

    const AABB get_bounds(const BodyData* body, bool compute) const;
    const AABB compute_bounds(const BodyData* body) const;

    void apply_force(const RigidBodyImpl& rb, const Vec2& force, const Vec2& at);

    Vec2 body_position(const RigidBodyImpl& rb) const;
    void body_position(const RigidBodyImpl& rb, const Vec2& pos);

    val_t body_rotation(const RigidBodyImpl& rb) const;
    void body_rotation(const RigidBodyImpl& rb, val_t rot);

    Vec2 body_centroidPosition(const RigidBodyImpl& rb) const;

    Vec2 body_velocity(const RigidBodyImpl& rb) const;
    void body_velocity(const RigidBodyImpl& rb, const Vec2& vel);

    val_t body_angularVelocity(const RigidBodyImpl& rb) const;
    void body_angularVelocity(const RigidBodyImpl& rb, val_t angVel);

    val_t body_mass(const RigidBodyImpl& rb) const;
    void body_mass(const RigidBodyImpl& rb, val_t m);

    val_t body_gravityScale(const RigidBodyImpl& rb) const;
    void body_gravityScale(const RigidBodyImpl& rb, val_t gs);

    BodyType body_bodyType(const RigidBodyImpl& rb) const;
    void body_bodyType(const RigidBodyImpl& rb, const BodyType& type);

    uint32_t body_layer(const RigidBodyImpl& rb) const;
    void body_layer(const RigidBodyImpl& rb, uint32_t layer);

    RigidBodyImpl createBody(const BodyDef& def, FizzWorld* parent);
    void tick(val_t dt, const Vec2& gravity);
};
}
