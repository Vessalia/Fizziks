#pragma once
#include <Dense.h>
#include <Pool.h>
#include <RigidBody.h>
#include <BodyDef.h>
#include <UniformGrid2D.h>

namespace Fizziks
{
class FizzWorld
{
public:
    static const Vector2p Gravity;

    FizzWorld(size_t unitsX, size_t unitsY);
    FizzWorld() : FizzWorld(20, 20) { }

    RigidBody createBody(const BodyDef& def);
    void destroyBody(RigidBody& body);

    void tick(val_t dt);

private:
    friend class RigidBody;

    struct BodyData
    {
        Vector2p position, velocity, angularVelocity, accumForce;
        val_t rotation, accumTorque;

        val_t mass;

        Shape shape;

        bool isStatic;
    };

    static const BodyData null_body;

    std::vector<Handle> activeHandles;
    std::vector<uint32_t> freeList;

    std::vector<uint32_t> activeList;
    std::vector<BodyData> activeBodies;

    std::queue<RigidBody> destructionQueue;

    UniformGrid2D grid;

    void simulate_bodies(val_t dt);
    void destroy_bodies();

    void set_body(const RigidBody& rb, const BodyDef& def);
    void set_body(BodyData* body, const BodyDef& def);

    const BodyData* get_body(const RigidBody& handle) const;
    BodyData* get_body(const RigidBody& handle);

    void apply_force(const RigidBody& rb, const Vector2p& force);

    Vector2p body_position(const RigidBody& rb) const;
    void body_position(const RigidBody& rb, const Vector2p& pos);

    Vector2p body_velocity(const RigidBody& rb) const;
    void body_velocity(const RigidBody& rb, const Vector2p& vel);

    Vector2p body_angularVelocity(const RigidBody& rb) const;
    void body_angularVelocity(const RigidBody& rb, const Vector2p& angVel);

    val_t body_mass(const RigidBody& rb) const;
    void body_mass(const RigidBody& rb, val_t m);

    Shape body_shape(const RigidBody& rb) const;
    void body_shape(const RigidBody& rb, Shape s);

    bool body_isStatic(const RigidBody& rb) const;
    void body_isStatic(const RigidBody& rb, bool is);
};
};
