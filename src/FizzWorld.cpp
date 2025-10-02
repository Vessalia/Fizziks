#include <FizzWorld.h>
#include <Shape.h>

namespace Fizziks
{
const FizzWorld::BodyData FizzWorld::null_body = 
{
    fizzmax<Vector2p>(),
    fizzmax<Vector2p>(),
    fizzmax<Vector2p>(),
    fizzmax<Vector2p>(),
    fizzmax<val_t>(),
    fizzmax<val_t>(),
    fizzmax<val_t>(),
    {},
    false
};

FizzWorld::FizzWorld(size_t unitsX, size_t unitsY, size_t worldScale) 
    : unitsX(unitsX * worldScale)
    , unitsY(unitsY * worldScale)
    , grid(UniformGrid2D(unitsX, unitsY, unitsX * worldScale, unitsY * worldScale)) { }

void FizzWorld::apply_force(const RigidBody& rb, const Vector2p& force)
{
    BodyData* body = get_body(rb);
    if(body) body->accumForce += force;
}

const FizzWorld::BodyData* FizzWorld::get_body(const RigidBody& rb) const
{
    if (rb.handle.index >= activeHandles.size()) return nullptr;
    Handle activeHandle = activeHandles[rb.handle.index];
    if(activeHandle.gen != rb.handle.gen) return nullptr;

    return &activeBodies[activeHandle.index];
}

FizzWorld::BodyData* FizzWorld::get_body(const RigidBody& rb)
{
    if (rb.handle.index >= activeHandles.size()) return nullptr;
    Handle activeHandle = activeHandles[rb.handle.index];
    if(activeHandle.gen != rb.handle.gen) return nullptr;

    return &activeBodies[activeHandle.index];
}

void FizzWorld::set_body(const RigidBody& rb, const BodyDef& def)
{
    BodyData* body = get_body(rb);
    if(body) set_body(body, def);
}

void FizzWorld::set_body(BodyData* body, const BodyDef& def)
{
    body->position = def.initPosition;
    body->velocity = def.initVelocity;
    body->angularVelocity = def.initAngularVelocity;

    body->rotation = def.initRotation;

    body->mass = def.mass;

    body->shape = def.shape;

    body->isStatic = def.isStatic;
}

Vector2p FizzWorld::body_position(const RigidBody& rb) const 
{
    auto* body = get_body(rb);
    if(body) return body->position;
    else     return null_body.position;
}
void FizzWorld::body_position(const RigidBody& rb, const Vector2p& pos) 
{ 
    auto* body = get_body(rb);
    if(body) body->position = pos;
}

Vector2p FizzWorld::body_velocity(const RigidBody& rb) const 
{ 
    auto* body = get_body(rb);
    if(body) return body->velocity;
    else     return null_body.velocity;
}
void FizzWorld::body_velocity(const RigidBody& rb, const Vector2p& vel) 
{ 
    auto* body = get_body(rb);
    if(body) body->velocity = vel;
}

Vector2p FizzWorld::body_angularVelocity(const RigidBody& rb) const 
{ 
    auto* body = get_body(rb);
    if(body) return body->angularVelocity;
    else     return null_body.angularVelocity;
}
void FizzWorld::body_angularVelocity(const RigidBody& rb, const Vector2p& angVel) 
{ 
    auto* body = get_body(rb);
    if(body) body->angularVelocity = angVel;
}

val_t FizzWorld::body_mass(const RigidBody& rb) const 
{ 
    auto* body = get_body(rb);
    if(body) return body->mass;
    else     return null_body.mass;
}
void FizzWorld::body_mass(const RigidBody& rb, val_t m) 
{ 
    auto* body = get_body(rb);
    if(body) body->mass = m;
}

Shape FizzWorld::body_shape(const RigidBody& rb) const 
{ 
    auto* body = get_body(rb);
    if(body) return body->shape;
    else     return null_body.shape;
}
void FizzWorld::body_shape(const RigidBody& rb, Shape s) 
{ 
    auto* body = get_body(rb);
    if(body) body->shape = s;
}

bool FizzWorld::body_isStatic(const RigidBody& rb) const 
{ 
    auto* body = get_body(rb);
    if(body) return body->isStatic;
    else     return null_body.isStatic;
}
void FizzWorld::body_isStatic(const RigidBody& rb, bool is) 
{ 
    auto* body = get_body(rb);
    if(body) body->isStatic = is;
}

FizzWorld::CollisionInfo FizzWorld::check_collision(const BodyData& bodyA, size_t bodyAIndex, const BodyData& bodyB, size_t bodyBIndex) const
{
    CollisionInfo collision;
        
    collision.bodyAIndex = bodyAIndex; 
    collision.bodyBIndex = bodyBIndex;
    collision.contact = getShapeContact(bodyA.shape, bodyA.position, bodyB.shape, bodyB.position);

    return collision;
}

void FizzWorld::detect_collisions(const BodyData& body, size_t bodyIndex)
{
    auto neighbourhood = grid.neighbourhood(bodyIndex);
    if (!neighbourhood) return;
    
    auto msb = neighbourhood.getMSB();
    for(int i = 0; i <= msb; ++i)
    {
        if (!neighbourhood[i]) continue;
        auto& neighbour = activeBodies[i];
        CollisionInfo collision = check_collision(body, bodyIndex, neighbour, i);
        if(!collision.contact.overlaps) continue;

        collisionQueue.push(collision);
    }
}

void FizzWorld::handle_collisions()
{
    for(size_t i = 0; i < activeBodies.size(); ++i)
    {
        detect_collisions(activeBodies[i], i);
    }

    // resolve_collisions();
}

const Vector2p FizzWorld::Gravity = {0, -9.81};
void FizzWorld::simulate_bodies(val_t dt)
{
    for (size_t i = 0; i < activeBodies.size(); ++i)
    {
        auto& body = activeBodies[i];
        body.accumForce += Gravity;
        if (body.isStatic) body.accumForce.setZero();
        body.velocity += body.accumForce * dt;
        body.position += body.velocity * dt;
        grid.update(i, body.position /* need to compute aabb if one not given */);
        body.accumForce.setZero();
    }
}

void FizzWorld::destroy_bodies()
{
    while (!destructionQueue.empty())
    {
        RigidBody rb = destructionQueue.front(); destructionQueue.pop();

        Handle* activeHandle = &activeHandles[rb.handle.index];
        if(activeHandle->gen == rb.handle.gen)
        {
            //swap remove
            uint32_t swapIndex = activeHandle->index;
            std::swap(activeBodies[swapIndex], activeBodies.back());
            std::swap(activeList[swapIndex], activeList.back());
            activeBodies.pop_back();
            activeList.pop_back();
            grid.remove(swapIndex);
            grid.replace(activeBodies.size(), swapIndex);
            if(swapIndex < activeBodies.size())
            {
                uint32_t movedIndex = activeList[swapIndex];
                activeHandles[movedIndex].index = swapIndex;
            }
            activeHandle->gen++;
            freeList.push_back(rb.handle.index);
        }
    }
}

RigidBody FizzWorld::createBody(const BodyDef& def)
{
    Handle handle{ static_cast<uint32_t>(activeHandles.size()), 0 }; // this will definitely not cause issues
    if(freeList.size() > 0)
    {
        uint32_t freeIndex = freeList.back(); freeList.pop_back();
        activeList.push_back(freeIndex);
        handle.index = freeIndex;
        handle.gen = activeHandles[freeIndex].gen;
        activeHandles[freeIndex].index = activeBodies.size();
    }
    else
    {
        activeList.push_back(activeHandles.size());
        activeHandles.push_back(handle);
    }

    BodyData b;
    b.accumForce = Vector2p::Zero();
    b.accumTorque = 0;
    RigidBody rb(handle, this);
    grid.insert(activeBodies.size(), def.initPosition /* need to compute aabb if one not given */);
    activeBodies.push_back(b);
    set_body(rb, def);
    return rb;
}

void FizzWorld::destroyBody(RigidBody& rb)
{
    destructionQueue.push(rb);
}

void FizzWorld::tick(val_t dt)
{
    simulate_bodies(dt);
    handle_collisions();
    destroy_bodies();
}
};
