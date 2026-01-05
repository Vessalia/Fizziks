#pragma once
#include <RigidBody.h>
#include <FizzWorld.h>
#include <RigidBodyImpl.h>
#include <Types.h>
#include <ContactKey.h>
#include <ShapeP.h>

namespace Fizziks
{

using AABB = internal::AABB;
using Contact = internal::Contact;
using Vector2p = internal::Vector2p;

class FizzWorld::Impl 
{
    friend class FizzWorld;
    friend class RigidBody;
    friend class RigidBody::Impl;

private:
    Impl(size_t unitsX, size_t unitsY, size_t worldScale, int collisionIterations, val_t timestep);
    ~Impl();

    size_t currstep = 0;

    struct BodyData
    {
        uint32_t layermask;

        Vector2p centroid;

        Vector2p position, velocity, accumForce;
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

        std::vector<std::pair<Collider, Vector2p>> colliders;

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

    std::queue<RigidBody::Impl> destructionQueue;

    Broadphase* broadphase;

    Vector2p get_worldPos(const BodyData& body, const Vector2p& colliderPos) const;
    val_t get_worldRotation(const BodyData& body, const Collider& collider) const;
    val_t clamp_angle(const val_t rot) const;

    CollisionManifold get_manifold(const size_t idA, const size_t idB) const;
    void detect_collisions();
    ContactKey makeContactKey(const CollisionResolution& resolution) const;
    CollisionResolution collision_preStep(const uint32_t idA, const uint32_t idB, const uint32_t collIdA, const uint32_t collIdB, const Contact& constact, const val_t dt);
    void solve_normalConstraint(CollisionResolution& resolution);
    void solve_frictionConstraint(CollisionResolution& resolution);
    void solve_contactConstraints(CollisionResolution& resolution);
    void resolve_collisions(const val_t dt);

    void simulate_bodies(const val_t dt, const Vector2p& gravity);
    void handle_collisions(const val_t dt);
    void destroy_bodies();

    void set_body(const RigidBody::Impl& rb, const BodyDef& def);
    void set_body(BodyData* body, const BodyDef& def);

    const BodyData* get_body(const RigidBody::Impl& handle) const;
    BodyData* get_body(const RigidBody::Impl& handle);

    void add_collider(const RigidBody::Impl& rb, const Collider& collider, const Vector2p& at);
    void add_collider(BodyData* body, const Collider& collider, const Vector2p& at);
    std::vector<std::pair<Collider, Vector2p>> body_colliders(const RigidBody::Impl& rb) const;

    const AABB get_bounds(const BodyData* body, bool compute) const;
    const AABB compute_bounds(const BodyData* body) const;

    void apply_force(const RigidBody::Impl& rb, const Vector2p& force, const Vector2p& at);

    Vector2p body_position(const RigidBody::Impl& rb) const;
    void body_position(const RigidBody::Impl& rb, const Vector2p& pos);

    val_t body_rotation(const RigidBody::Impl& rb) const;
    void body_rotation(const RigidBody::Impl& rb, const val_t rot);

    Vector2p body_centroidPosition(const RigidBody::Impl& rb) const;

    Vector2p body_velocity(const RigidBody::Impl& rb) const;
    void body_velocity(const RigidBody::Impl& rb, const Vector2p& vel);

    val_t body_angularVelocity(const RigidBody::Impl& rb) const;
    void body_angularVelocity(const RigidBody::Impl& rb, const val_t& angVel);

    val_t body_mass(const RigidBody::Impl& rb) const;
    void body_mass(const RigidBody::Impl& rb, val_t m);

    val_t body_gravityScale(const RigidBody::Impl& rb) const;
    void body_gravityScale(const RigidBody::Impl& rb, val_t gs);

    BodyType body_bodyType(const RigidBody::Impl& rb) const;
    void body_bodyType(const RigidBody::Impl& rb, const BodyType& type);

    uint32_t body_layerMask(const RigidBody::Impl& rb) const;
    void body_layerMask(const RigidBody::Impl& rb, const uint32_t mask);

    RigidBody::Impl createBody(const BodyDef& def, FizzWorld* parent);
    void tick(const val_t dt, const Vector2p& gravity);
};
}
