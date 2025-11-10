#include <FizzWorld.h>
#include <Shape.h>
#include <SimpleBP.h>

namespace Fizziks
{
const FizzWorld::BodyData FizzWorld::null_body = 
{
    fizzmax<Vector2p>(),
    fizzmax<Vector2p>(), fizzmax<Vector2p>(), fizzmax<Vector2p>(),
    fizzmax<val_t>(), fizzmax<val_t>(), fizzmax<val_t>(),
    fizzmax<val_t>(), fizzmax<val_t>(),
    fizzmax<val_t>(), fizzmax<val_t>(),
    fizzmax<val_t>(),
    {},
    {},
    true
};

FizzWorld::FizzWorld(size_t unitsX, size_t unitsY, size_t worldScale, int collisionResolution, val_t timestep) 
    : unitsX(unitsX * worldScale)
    , unitsY(unitsY * worldScale)
    , accumulator(0)
    , timestep(timestep)
    , broadphase(new SimpleBP())
    , collisionResolution(collisionResolution) { }

FizzWorld::~FizzWorld() { delete broadphase; }

void FizzWorld::apply_force(const RigidBody& rb, const Vector2p& force, const Vector2p& at)
{
    BodyData* body = get_body(rb);
    if(body) 
    {
        body->accumForce += force;
        Vector2p r = at - body->centroid;
        body->accumTorque += r.x() * force.y() - r.y() * force.x();
    }
}

void FizzWorld::add_collider(const RigidBody& rb, const Collider& collider, const Vector2p& at)
{
    BodyData* body = get_body(rb);
    if(body) add_collider(body, collider, at);
}

void FizzWorld::add_collider(BodyData* body, const Collider& collider, const Vector2p& _at)
{
    body->colliders.push_back({ collider, _at });
    
    if(!body->isStatic)
    {
        body->mass += collider.mass;
        body->invMass = 1 / body->mass;
    }

    Vector2p centroid;
    for(auto [coll, at] : body->colliders)
    {
        centroid += coll.mass * at;
    }

    body->centroid = centroid * body->invMass;

    val_t MoI = 0;
    for(auto [coll, at]: body->colliders)
    {
        // parallel axis theorem
        Vector2p r = centroid - at;
        MoI += coll.MoI + coll.mass * r.dot(r);
    }

    body->MoI = MoI;
    body->invMoI = 1 / MoI;
}

const AABB FizzWorld::getBounds(BodyData* body, bool compute) const
{
    if (compute) return computeBounds(body);
    else         return body->bounds;
}

const AABB FizzWorld::computeBounds(BodyData* body) const
{
    AABB bounds;

    Vector2p min = fizzmax<Vector2p>(), max = fizzmax<Vector2p>();
    for (auto [collider, at] : body->colliders)
    {
        AABB minorBounds = getInscribingAABB(collider.shape, at, body->rotation);
        Vector2p pos = body->position + at + minorBounds.offset;

        min.x() = std::min(min.x(), pos.x() - minorBounds.halfWidth);
        min.y() = std::min(min.y(), pos.y() - minorBounds.halfHeight);

        max.x() = std::max(max.x(), pos.x() + minorBounds.halfWidth);
        max.y() = std::max(max.y(), pos.y() + minorBounds.halfHeight);
    }
    bounds.halfWidth  = (max.x() - min.x()) / 2;
    bounds.halfHeight = (max.y() - min.y()) / 2;
    bounds.offset = min + Vector2p(bounds.halfWidth, bounds.halfHeight) - (body->position + body->centroid);

    return bounds;
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

    body->rotation = def.initRotation;
    body->angularVelocity = def.initAngularVelocity;

    body->gravityScale = def.gravityScale;

    body->isStatic = def.isStatic;

    for(auto [collider, at] : def.colliderDefs)
    {
        add_collider(body, collider, at);
    }

    if(def.isStatic)
    {
        body->mass = fizzmax<val_t>();
        body->invMass = 0;
    }

    computeBounds(body);
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

val_t FizzWorld::body_angularVelocity(const RigidBody& rb) const 
{ 
    auto* body = get_body(rb);
    if(body) return body->angularVelocity;
    else     return null_body.angularVelocity;
}
void FizzWorld::body_angularVelocity(const RigidBody& rb, const val_t& angVel) 
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

FizzWorld::CollisionInfo FizzWorld::check_collision(uint32_t idA, uint32_t idB) const
{
    CollisionInfo collision;
        
    collision.bodyAId = idA; 
    collision.bodyBId = idB;
    //collision.contact = getShapeContact(bodyA.shape, bodyA.position, bodyB.shape, bodyB.position);

    return collision;
}

val_t slop = 0.01;
val_t contactEps = 2 * slop;
void FizzWorld::detect_collisions()
{
    // broadphase detection
    auto broadPairs = broadphase->computePairs();
    if (broadPairs.size() == 0) return;

    // nearphase detection
    for (auto [idA, idB] : broadPairs)
    {
        CollisionInfo collision = check_collision(idA, idB);
        if(!collision.contact.overlaps || collision.contact.penetration < contactEps) continue;

        collisionQueue.push(collision);
    }
}

val_t beta = 0.2;
val_t bounceThreshold = 0.5;
val_t velEps = 0.05;
val_t tangentEps = 0.001;
void FizzWorld::resolve_collisions()
{
    while(!collisionQueue.empty())
    {
        CollisionInfo collision = collisionQueue.front(); collisionQueue.pop();
        auto& body  = activeBodies[collision.bodyAId];
        auto& other = activeBodies[collision.bodyBId];
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
    detect_collisions();
    resolve_collisions();
}

void FizzWorld::simulate_bodies(val_t dt)
{
    for (size_t ID = 0; ID < activeBodies.size(); ++ID)
    {
        auto& body = activeBodies[ID];
        if (body.isStatic) continue;

        Vector2p prevPos = body.position;
        val_t prevRot = body.rotation;

        Vector2p accel = body.accumForce * body.invMass + Gravity * body.gravityScale;
        body.velocity = body.velocity * std::max(0.f, (1 - body.linearDamping * dt)) + accel * dt;
        body.position += body.velocity * dt;
        body.accumForce.setZero();

        val_t angularAccel = body.accumTorque * body.invMoI;
        body.angularVelocity = body.angularVelocity * std::max(0.f, (1 - body.angularDamping * dt)) + angularAccel * dt;
        body.rotation += body.angularVelocity * dt;
        body.accumTorque = 0;

        bool posUpdate = prevPos != body.position;
        bool rotUpdate = prevRot != body.rotation; 
        if(posUpdate || rotUpdate) 
            broadphase->update(ID, getBounds(&body, rotUpdate), body.position + body.centroid);
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
            broadphase->remove(swapIndex);
            broadphase->replace(activeBodies.size(), swapIndex);
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
        handle.index = freeIndex;
        handle.gen = activeHandles[freeIndex].gen;
        activeHandles[freeIndex].index = activeBodies.size();
        activeList.push_back(freeIndex);
    }
    else
    {
        activeList.push_back(activeHandles.size());
        activeHandles.push_back(handle);
    }

    BodyData b;
    b.accumForce = Vector2p::Zero();
    b.accumTorque = 0;
    uint32_t ID = activeBodies.size();
    activeBodies.push_back(b);
    set_body(&activeBodies[ID], def);

    broadphase->add(ID, computeBounds(&b), b.position + b.centroid);

    return { handle, this };
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
