#pragma once
#include <RigidBodyImpl.h>
#include <BroadPhase.h>
#include <Handle.h>
#include <ContactKey.h>

namespace Fizziks::internal
{
class FizzWorldImpl 
{
    friend class FizzWorld;
    friend class RigidBody;
    friend class RigidBodyImpl;

private:
    FizzWorldImpl(size_t unitsX, size_t unitsY, size_t worldScale, int collisionIterations, val_t timestep);
    ~FizzWorldImpl();

    size_t currstep = 0;

    struct BodyData
    {
        uint32_t layermask;

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

        std::vector<std::pair<Collider, Vec2>> colliders;

        BodyType bodyType;
    };

    struct CollisionManifold
    {
        size_t bodyAId, bodyBId;
        std::vector<std::tuple<uint32_t, uint32_t, Contact>> contacts;
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

    val_t timestep;
    val_t accumulator;

    size_t unitsX;
    size_t unitsY;

    int collisionIterations;

    std::vector<Handle> activeHandles;
    std::vector<uint32_t> freeList;

    std::vector<uint32_t> activeList;
    std::vector<BodyData> activeBodies;

    std::vector<CollisionManifold> collisionManifolds;
    std::vector<CollisionResolution> collisionResolutions;
    std::unordered_map<ContactKey, CollisionResolution> warmStartCache;

    std::queue<RigidBodyImpl> destructionQueue;

    Broadphase* broadphase;

    Vec2 get_worldPos(const BodyData& body, const Vec2& colliderPos) const;
    val_t get_worldRotation(const BodyData& body, const Collider& collider) const;

    CollisionManifold get_manifold(const size_t idA, const size_t idB) const;
    void detect_collisions();
    ContactKey makeContactKey(const CollisionResolution& resolution) const;
    CollisionResolution collision_preStep(const uint32_t idA, const uint32_t idB, const uint32_t collIdA, const uint32_t collIdB, const Contact& constact, const val_t dt);
    void solve_normalConstraint(CollisionResolution& resolution);
    void solve_frictionConstraint(CollisionResolution& resolution);
    void solve_contactConstraints(CollisionResolution& resolution);
    void resolve_collisions(const val_t dt);

    void simulate_bodies(const val_t dt, const Vec2& gravity);
    void handle_collisions(const val_t dt);
    void destroy_bodies();

    void set_body(const RigidBodyImpl& rb, const BodyDef& def);
    void set_body(BodyData* body, const BodyDef& def);

    const BodyData* get_body(const RigidBodyImpl& handle) const;
    BodyData* get_body(const RigidBodyImpl& handle);

    void add_collider(const RigidBodyImpl& rb, const Collider& collider, const Vec2& at);
    void add_collider(BodyData* body, const Collider& collider, const Vec2& at);
    std::vector<std::pair<Collider, Vec2>> body_colliders(const RigidBodyImpl& rb) const;

    const AABB get_bounds(const BodyData* body, bool compute) const;
    const AABB compute_bounds(const BodyData* body) const;

    void apply_force(const RigidBodyImpl& rb, const Vec2& force, const Vec2& at);

    Vec2 body_position(const RigidBodyImpl& rb) const;
    void body_position(const RigidBodyImpl& rb, const Vec2& pos);

    val_t body_rotation(const RigidBodyImpl& rb) const;
    void body_rotation(const RigidBodyImpl& rb, const val_t rot);

    Vec2 body_centroidPosition(const RigidBodyImpl& rb) const;

    Vec2 body_velocity(const RigidBodyImpl& rb) const;
    void body_velocity(const RigidBodyImpl& rb, const Vec2& vel);

    val_t body_angularVelocity(const RigidBodyImpl& rb) const;
    void body_angularVelocity(const RigidBodyImpl& rb, const val_t& angVel);

    val_t body_mass(const RigidBodyImpl& rb) const;
    void body_mass(const RigidBodyImpl& rb, val_t m);

    val_t body_gravityScale(const RigidBodyImpl& rb) const;
    void body_gravityScale(const RigidBodyImpl& rb, val_t gs);

    BodyType body_bodyType(const RigidBodyImpl& rb) const;
    void body_bodyType(const RigidBodyImpl& rb, const BodyType& type);

    uint32_t body_layerMask(const RigidBodyImpl& rb) const;
    void body_layerMask(const RigidBodyImpl& rb, const uint32_t mask);

    RigidBodyImpl createBody(const BodyDef& def, FizzWorld* parent);
    void tick(const val_t dt, const Vec2& gravity);
};
}
