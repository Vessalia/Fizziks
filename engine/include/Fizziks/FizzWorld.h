#pragma once
#include <Dense.h>
#include <Pool.h>
#include <RigidBody.h>
#include <RigidDef.h>
#include <Broadphase.h>

namespace Fizziks
{
class FizzWorld
{
public:
    Vector2p Gravity = {0, -9.81};

    FizzWorld(size_t unitsX, size_t unitsY, size_t worldScale, int collisionResolution, val_t timeStep);
    FizzWorld() : FizzWorld(20, 20, 2, 3, 1 / 200.f) { }
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

        std::vector<std::pair<Collider, Vector2p>> colliders;

        AABB bounds;

        bool isStatic;

        val_t restitution = 0.2;
        val_t staticFriction = 0.2;
        val_t dynamicFriction = 0.1;
        val_t linearDamping = 0.05;
        val_t angularDamping = 0.05;
    };

    struct CollisionInfo
    {
        size_t bodyAId, bodyBId;
        Contact contact;
    };

    struct CollisionResolution
    {
        BodyData* body;
        Vector2p correction;
        Vector2p impulse;
    };

    static const BodyData null_body;

    val_t timestep;
    val_t accumulator;

    size_t unitsX;
    size_t unitsY;

    int collisionResolution;

    std::vector<Handle> activeHandles;
    std::vector<uint32_t> freeList;

    std::vector<uint32_t> activeList;
    std::vector<BodyData> activeBodies;

    std::queue<CollisionInfo> collisionQueue;
    std::queue<CollisionResolution> collisionResolveQueue;
    std::queue<RigidBody> destructionQueue;

    Broadphase* broadphase;

    CollisionInfo check_collision(uint32_t idA, uint32_t idB) const;
    void detect_collisions();
    void resolve_collisions();
    void simulate_bodies(val_t dt);
    void handle_collisions();
    void destroy_bodies();

    void set_body(const RigidBody& rb, const BodyDef& def);
    void set_body(BodyData* body, const BodyDef& def);

    const BodyData* get_body(const RigidBody& handle) const;
    BodyData* get_body(const RigidBody& handle);

    void add_collider(const RigidBody& rb, const Collider& collider, const Vector2p& at);
    void add_collider(BodyData* body, const Collider& collider, const Vector2p& at);
    const AABB getBounds(BodyData* body, bool compute) const;
    const AABB computeBounds(BodyData* body) const;

    void apply_force(const RigidBody& rb, const Vector2p& force, const Vector2p& at);

    Vector2p body_position(const RigidBody& rb) const;
    void body_position(const RigidBody& rb, const Vector2p& pos);

    Vector2p body_velocity(const RigidBody& rb) const;
    void body_velocity(const RigidBody& rb, const Vector2p& vel);

    val_t body_angularVelocity(const RigidBody& rb) const;
    void body_angularVelocity(const RigidBody& rb, const val_t& angVel);

    val_t body_mass(const RigidBody& rb) const;
    void body_mass(const RigidBody& rb, val_t m);

    val_t body_gravityScale(const RigidBody& rb) const;
    void body_gravityScale(const RigidBody& rb, val_t gs);

    bool body_isStatic(const RigidBody& rb) const;
    void body_isStatic(const RigidBody& rb, bool is);
};
};
