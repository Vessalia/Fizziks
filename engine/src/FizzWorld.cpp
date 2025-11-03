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
    fizzmax<val_t>(),
    {},
    true
};

FizzWorld::FizzWorld(size_t unitsX, size_t unitsY, size_t worldScale, int collisionResolution, val_t timestep) 
    : unitsX(unitsX * worldScale)
    , unitsY(unitsY * worldScale)
    , accumulator(0)
    , timestep(timestep)
    , grid(UniformGrid2D(unitsX, unitsY, unitsX * worldScale, unitsY * worldScale))
    , collisionResolution(collisionResolution) { }

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

    body->isStatic = def.isStatic;

    if(def.isStatic)
        body->invMass = 0;
    else if(def.mass == 0)
        body->invMass = fizzmax<val_t>();
    else
        body->invMass = 1 / abs(def.mass);
    body->gravityScale = def.gravityScale;

    body->shape = def.shape;
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
    if(body) return 1 / body->invMass;
    else     return 1 / null_body.invMass;
}
void FizzWorld::body_mass(const RigidBody& rb, val_t m) 
{ 
    auto* body = get_body(rb);
    if(body) body->invMass = 1 / m;
}

val_t FizzWorld::body_gravityScale(const RigidBody& rb) const
{
    auto* body = get_body(rb);
    if (body) return body->gravityScale;
    else      return null_body.gravityScale;
}
void FizzWorld::body_gravityScale(const RigidBody& rb, val_t gs)
{
    auto* body = get_body(rb);
    if (body) body->gravityScale = gs;
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

val_t slop = 0.01;
val_t contactEps = 2 * slop;
val_t beta = 0.2;
val_t bounceThreshold = 0.5;
val_t velEps = 0.05;
val_t tangentEps = 0.001;
void FizzWorld::detect_collisions(const BodyData& body, size_t bodyIndex)
{
    auto neighbourhood = grid.neighbourhood(bodyIndex);
    if (!neighbourhood) return;

    for(int i = 0; i < bodyIndex; ++i)
    {
        if (!neighbourhood[i]) continue;

        auto& neighbour = activeBodies[i];
        CollisionInfo collision = check_collision(body, bodyIndex, neighbour, i);
        if(!collision.contact.overlaps || collision.contact.penetration < contactEps) continue;

        collisionQueue.push(collision);
    }
}

void FizzWorld::resolve_collisions()
{
    while(!collisionQueue.empty())
    {
        CollisionInfo collision = collisionQueue.front(); collisionQueue.pop();
        auto& body  = activeBodies[collision.bodyAIndex];
        auto& other = activeBodies[collision.bodyBIndex];
        val_t m1 = body.invMass;
        val_t m2 = other.invMass;
        if (m1 + m2 == 0) continue;

        Contact contact = collision.contact;
        // Baumgarte stabilization
        val_t mFactor = 1 / (m1 + m2);
        Vector2p normal = contact.normal;
        Vector2p correction = normal * std::max(contact.penetration - slop, 0.f) * beta * mFactor;

        Vector2p normalImpulse = Vector2p::Zero();
        Vector2p frictionImpulse = Vector2p::Zero();
        Vector2p relVel = other.velocity - body.velocity;
        val_t velFactor = relVel.dot(contact.normal);
        if (velFactor <= 0 /* should be an eps not 0 */) // only apply resolving forces if moving towards or tangent
        {
            // Restitution impulse
            val_t restitution = std::min(body.restitution, other.restitution);
            if (std::abs(relVel.norm()) < bounceThreshold)
                restitution = 0;
            val_t impulse = -(1 + restitution) * velFactor * mFactor;
            normalImpulse = normal * impulse;

            // Frictional impulse
            Vector2p tangent = (relVel - contact.normal * velFactor);
            if (tangent.norm() > tangentEps) // ignore tiny tangents
            {
                tangent = tangent.normalized();
                val_t friction = -relVel.dot(tangent) * mFactor;
                val_t muS = std::min(body.staticFriction, other.staticFriction);
                val_t muD = std::min(body.dynamicFriction, other.dynamicFriction);

                friction = std::clamp(friction, -impulse * muS, impulse * muS);
                if (std::abs(friction) < impulse * muS) // not sliding yet
                    frictionImpulse = friction * tangent;
                else
                    frictionImpulse = -impulse * muD * tangent;
            }
        }

        collisionResolveQueue.push(
        {
            &body,
            -correction * m1,
            -(normalImpulse + frictionImpulse) * m1
        });
        collisionResolveQueue.push(
        {
            &other,
            correction * m2,
            (normalImpulse + frictionImpulse) * m2
        });
    }

    while (!collisionResolveQueue.empty())
    {
        CollisionResolution resolution = collisionResolveQueue.front(); collisionResolveQueue.pop();
        auto& body = *resolution.body;
        body.position += resolution.correction;
        body.velocity += resolution.impulse;
    }
}

void FizzWorld::handle_collisions()
{
    for(size_t i = 0; i < activeBodies.size(); ++i)
    {
        detect_collisions(activeBodies[i], i);
    }

    resolve_collisions();
}

void FizzWorld::simulate_bodies(val_t dt)
{
    for (size_t i = 0; i < activeBodies.size(); ++i)
    {
        auto& body = activeBodies[i];
        if (body.isStatic) continue;

        Vector2p accel = body.accumForce * body.invMass;
        accel += Gravity * body.gravityScale;
        body.velocity *= std::max(0.f, (1 - body.linearDamping * dt));
        body.velocity += accel * dt;
        Vector2p prevPos = body.position;
        body.position += body.velocity * dt;

        body.angularVelocity *= std::max(0.f, (1 - body.angularDamping * dt));

        if(prevPos != body.position) grid.update(i, body.position, getInscribingAABB(body.shape));
        body.accumForce.setZero();
    }
}

void FizzWorld::destroy_bodies()
{
    while (!destructionQueue.empty())
    {
        RigidBody rb = destructionQueue.front(); destructionQueue.pop();

        Handle* activeHandle = &activeHandles[rb.handle.index];
        if (activeHandle->gen == rb.handle.gen)
        {
            //swap remove
            uint32_t swapIndex = activeHandle->index;
            std::swap(activeBodies[swapIndex], activeBodies.back());
            std::swap(activeList[swapIndex], activeList.back());
            activeBodies.pop_back();
            activeList.pop_back();
            grid.remove(swapIndex);
            grid.replace(activeBodies.size(), swapIndex);
            if (swapIndex < activeBodies.size())
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
    if (freeList.size() > 0)
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
    grid.insert(activeBodies.size(), def.initPosition, getInscribingAABB(def.shape));
    activeBodies.push_back(b);
    set_body(&activeBodies[activeBodies.size() - 1], def);
    return rb;
}

void FizzWorld::destroyBody(RigidBody& rb)
{
    destructionQueue.push(rb);
}

Vector2p FizzWorld::worldScale() const
{
    return { unitsX, unitsY };
}

void FizzWorld::tick(val_t dt)
{
    accumulator += dt;
    while (accumulator >= timestep)
    {
        accumulator -= timestep;

        simulate_bodies(timestep);
        for (int i = 0; i < collisionResolution; ++i)
            handle_collisions();
        destroy_bodies();
    }
}
};
