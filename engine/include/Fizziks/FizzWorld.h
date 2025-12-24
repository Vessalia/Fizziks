#pragma once
#include <Dense.h>
#include <Pool.h>
#include <RigidBody.h>
#include <RigidDef.h>
#include <Broadphase.h>

#include <vector>
#include <unordered_map>

namespace Fizziks
{
class FizzWorld
{
public:
    Vector2p Gravity = {0, -9.81};

    FizzWorld(size_t unitsX, size_t unitsY, size_t worldScale, int collisionIterations, val_t timeStep);
    FizzWorld() : FizzWorld(20, 20, 2, 5, 1 / 200.f) { }
    ~FizzWorld();

    RigidBody createBody(const BodyDef& def);
    void destroyBody(RigidBody& body);

    Vector2p worldScale() const;

    void tick(val_t dt);

private:
    friend class RigidBody;

    struct BodyData
    {
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

    std::queue<RigidBody> destructionQueue;

    Broadphase* broadphase;

    Vector2p get_worldPos(const BodyData& body, const Vector2p& colliderPos) const;
    val_t get_worldRotation(const BodyData& body, const Collider& collider) const;
    void rotate_body(BodyData& body, const val_t rot);

    CollisionManifold get_manifold(const size_t idA, const size_t idB) const;
    void detect_collisions();
    ContactKey makeContactKey(const CollisionResolution& resolution) const;
    CollisionResolution collision_preStep(const uint32_t idA, const uint32_t idB, const uint32_t collIdA, const uint32_t collIdB, const Contact& constact, const val_t dt);
    void solve_normalConstraint(CollisionResolution& resolution);
    void solve_frictionConstraint(CollisionResolution& resolution);
    void solve_contactConstraints(CollisionResolution& resolution);
    void resolve_collisions(val_t dt);

    void simulate_bodies(val_t dt);
    void handle_collisions(val_t dt);
    void destroy_bodies();

    void set_body(const RigidBody& rb, const BodyDef& def);
    void set_body(BodyData* body, const BodyDef& def);

    const BodyData* get_body(const RigidBody& handle) const;
    BodyData* get_body(const RigidBody& handle);

    void add_collider(const RigidBody& rb, const Collider& collider, const Vector2p& at);
    void add_collider(BodyData* body, const Collider& collider, const Vector2p& at);
    std::vector<std::pair<Collider, Vector2p>> body_colliders(const RigidBody& rb) const;

    const AABB get_bounds(BodyData* body, bool compute) const;
    const AABB compute_bounds(BodyData* body) const;

    void apply_force(const RigidBody& rb, const Vector2p& force, const Vector2p& at);

    Vector2p body_position(const RigidBody& rb) const;
    void body_position(const RigidBody& rb, const Vector2p& pos);

    val_t body_rotation(const RigidBody& rb) const;
    void body_rotation(const RigidBody& rb, const val_t rot);

    Vector2p body_centroidPosition(const RigidBody& rb) const;

    Vector2p body_velocity(const RigidBody& rb) const;
    void body_velocity(const RigidBody& rb, const Vector2p& vel);

    val_t body_angularVelocity(const RigidBody& rb) const;
    void body_angularVelocity(const RigidBody& rb, const val_t& angVel);

    val_t body_mass(const RigidBody& rb) const;
    void body_mass(const RigidBody& rb, val_t m);

    val_t body_gravityScale(const RigidBody& rb) const;
    void body_gravityScale(const RigidBody& rb, val_t gs);

    BodyType body_bodyType(const RigidBody& rb) const;
    void body_bodyType(const RigidBody& rb, const BodyType& type);
};
};
