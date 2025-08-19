#pragma once
#include "Dense.h"
#include "Pool.h"
#include "RigidBody.h"
#include "BodyDef.h"

namespace Fizziks
{
class FizzWorld
{
public:
    static const Vector2p Gravity;

    FizzWorld(int units_x, int units_y) { }
    FizzWorld() : FizzWorld(10, 10) { }

    RigidBody createBody(const BodyDef& def);
    void destroyBody(RigidBody& body);

    void tick(val_t dt);

private:
    friend class RigidBody;

    struct BodyData;

    static const BodyData null_body;

    Pool<BodyData> rigidBodyPool;

    std::vector<Handle> activeBodyHandles;
    std::vector<BodyData*> activeBodies;

    void set_body(const RigidBody& rb, const BodyDef& def);

    BodyData* get_body(const RigidBody& handle) const;

    void apply_force(const Handle& handle, const Vector2p& force);

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
