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

FizzWorld::FizzWorld(size_t unitsX, size_t unitsY, size_t worldScale, int collisionIterations, val_t timestep) 
    : unitsX(unitsX * worldScale)
    , unitsY(unitsY * worldScale)
    , accumulator(0)
    , timestep(timestep)
    , broadphase(new SimpleBP())
    , collisionIterations(collisionIterations) { }

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

const AABB FizzWorld::get_bounds(BodyData* body, bool compute) const
{
    if (compute) return compute_bounds(body);
    else         return body->bounds;
}

const AABB FizzWorld::compute_bounds(BodyData* body) const
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

    compute_bounds(body);
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

Vector2p FizzWorld::get_worldPos(const BodyData& body, const Vector2p& colliderPos) const
{
    return body.position + body.centroid + Rotation2p(body.rotation) * colliderPos;
}

val_t FizzWorld::get_worldRotation(const BodyData& body, const Collider& collider) const
{
    return body.rotation + collider.rotation;
}

val_t slop = 0.01;
val_t contactEps = 2 * slop;
FizzWorld::CollisionManifold FizzWorld::get_manifold(const size_t idA, const size_t idB) const
{
    CollisionManifold manifold;
    manifold.bodyAId = idA;
    manifold.bodyBId = idB;

    const auto& bodyA = activeBodies[idA];
    const auto& bodyB = activeBodies[idA];
    for (const auto& [s1, p1] : bodyA.colliders)
    {
        Vector2p posA = get_worldPos(bodyA, p1);
        val_t rotA = get_worldRotation(bodyA, s1);
        for (const auto& [s2, p2] : bodyB.colliders)
        {
            Vector2p posB = get_worldPos(bodyB, p2);
            val_t rotB = get_worldRotation(bodyB, s2);
            Contact contact = getShapeContact(s1.shape, posA, rotA, s2.shape, posB, rotB);

            if (contact.overlaps || contact.penetration >= contactEps) manifold.contacts.push_back(contact);
        }
    }

    return manifold;
}

void FizzWorld::detect_collisions()
{
    // broadphase detection
    auto broadPairs = broadphase->computePairs();
    if (broadPairs.size() == 0) return;

    // nearphase detection
    for (auto [idA, idB] : broadPairs)
    {
        CollisionManifold manifold = get_manifold(idA, idB);
        if (manifold.contacts.size() > 0) collisionManifolds.push_back(manifold);
    }
}

val_t beta = 0.2;
// precompute inverse effective mass (JM^-1J^T)^-1 and bias term
// J = [-n^T -(rA x n)^T n^T (rB x n)^T]
// M^-1 = [Ma^-1 0 0 0]
//        [0 Ia^-1 0 0]
//        [0 0 Mb^-1 0]
//        [0 0 0 Ib^-1]
// JM^-1    = [-n^TMa^-1 -(rA x n)^TIa^-1 n^TMb^-1 (rB x n)^TIb^-1]
// JM^-1J^T = |n|^2Ma^-1 + |rA x n|^2 Ia^-1 + |n|^2Mb^-1 + |rB x n|^2 Ib^-1
//          = Ma^-1 + |rA x n|^2 Ia^-1 + Mb^-1 + |rB x n|^2 Ib^-1
FizzWorld::CollisionResolution FizzWorld::collision_preStep(const size_t idA, const size_t idB, const Contact& contact, val_t dt) const
{
    CollisionResolution resolution;
    resolution.bodyAId = idA;
    resolution.bodyBId = idB;

    const auto& bodyA = activeBodies[idA];
    const auto& bodyB = activeBodies[idB];

    Vector2p rA = contact.contactPointWorldA - (bodyA.position + bodyA.centroid);
    Vector2p rB = contact.contactPointWorldB - (bodyB.position + bodyB.centroid);

    val_t rnA = crossproduct(rA, contact.normal);
    val_t rtA = crossproduct(rA, contact.tangent);
    val_t rnB = crossproduct(rB, contact.normal);
    val_t rtB = crossproduct(rB, contact.tangent);

    val_t effMass        = bodyA.invMass + bodyA.invMoI * rnA * rnA +
                           bodyB.invMass + bodyB.invMoI * rnB * rnB;
    val_t effTangentMass = bodyA.invMass + bodyA.invMoI * rtA * rtA +
                           bodyB.invMass + bodyB.invMoI * rtB * rtB;

    resolution.invEffMass = effMass > 0 ? 1 / effMass : 0;
    resolution.invEffTangentMass = effTangentMass > 0 ? 1 / effTangentMass : 0;

    val_t baumgarte = -beta / dt * std::max(contact.penetration - slop, static_cast<val_t>(0));
    Vector2p vA = bodyA.velocity + crossproduct(bodyA.angularVelocity, rA);
    Vector2p vB = bodyB.velocity + crossproduct(bodyB.angularVelocity, rB);
    Vector2p dv = vB - vA;
    val_t closingVel = contact.normal.dot(dv);
    val_t restitution = std::min(bodyA.restitution, bodyB.restitution) * closingVel;
    resolution.bias = baumgarte + restitution;

    return resolution;
}

// dv  = vb + rb x wb - (va + ra x wa)
// Jv  = n . (vb - va) + (rb x n)wb - (ra x n)wa
//     = n . (dv)
// lam = (-Jv + b)/effMass
void FizzWorld::solve_normalConstraint(CollisionResolution& resolution)
{
    auto& bodyA = activeBodies[resolution.bodyAId];
    auto& bodyB = activeBodies[resolution.bodyBId];
    const auto& contact = resolution.contact;

    Vector2p rA = contact.contactPointWorldA - (bodyA.position + bodyA.centroid);
    Vector2p rB = contact.contactPointWorldB - (bodyB.position + bodyB.centroid);
    Vector2p vA = bodyA.velocity + crossproduct(bodyA.angularVelocity, rA);
    Vector2p vB = bodyB.velocity + crossproduct(bodyB.angularVelocity, rB);
    Vector2p dv = vB - vA;
    val_t Jv = dv.dot(contact.normal);

    val_t lambda = resolution.invEffMass * (-Jv + resolution.bias);

    val_t oldImpulse = resolution.normalImpulse;
    resolution.normalImpulse = std::max(static_cast<val_t>(0), oldImpulse + lambda);
    lambda = resolution.normalImpulse - oldImpulse;
    Vector2p impulse = lambda * contact.normal;

    bodyA.velocity -= impulse * bodyA.invMass;
    bodyA.angularVelocity -= crossproduct(rA, impulse) * bodyA.invMoI;

    bodyB.velocity += impulse * bodyA.invMass;
    bodyB.angularVelocity -= crossproduct(rB, impulse) * bodyB.invMoI;
}

// recompute is intentional
void FizzWorld::solve_frictionConstraint(CollisionResolution& resolution)
{
    auto& bodyA = activeBodies[resolution.bodyAId];
    auto& bodyB = activeBodies[resolution.bodyBId];
    const auto& contact = resolution.contact;

    Vector2p rA = contact.contactPointWorldA - (bodyA.position + bodyA.centroid);
    Vector2p rB = contact.contactPointWorldB - (bodyB.position + bodyB.centroid);
    Vector2p vA = bodyA.velocity + crossproduct(bodyA.angularVelocity, rA);
    Vector2p vB = bodyB.velocity + crossproduct(bodyB.angularVelocity, rB);
    Vector2p dv = vB - vA;
    val_t Jvt = dv.dot(contact.tangent);

    val_t lambdaT = resolution.invEffMass * (-Jvt);
    val_t oldImpulse = resolution.tangentImpulse;
    val_t newImpulse = resolution.tangentImpulse + lambdaT;

    val_t maxStatic = std::min(bodyA.staticFriction, bodyB.staticFriction) * resolution.normalImpulse;
    val_t maxDynamic = std::min(bodyA.dynamicFriction, bodyB.dynamicFriction) * resolution.normalImpulse;

    if (std::abs(newImpulse) <= maxStatic)
    {
        // static case
        resolution.tangentImpulse = newImpulse;
    }
    else
    {
        // sliding case
        resolution.tangentImpulse = std::clamp(newImpulse, -maxDynamic, maxDynamic);
    }
    lambdaT = resolution.tangentImpulse - oldImpulse;
    Vector2p impulse = lambdaT * contact.tangent;

    bodyA.velocity -= impulse * bodyA.invMass;
    bodyA.angularVelocity -= crossproduct(rA, impulse) * bodyA.invMoI;

    bodyB.velocity += impulse * bodyA.invMass;
    bodyB.angularVelocity -= crossproduct(rB, impulse) * bodyB.invMoI;
}

void FizzWorld::solve_contactConstraints(CollisionResolution& resolution)
{
    solve_normalConstraint(resolution);
    solve_frictionConstraint(resolution);
}

void FizzWorld::resolve_collisions(val_t dt)
{
    for (auto& manifold : collisionManifolds)
    {
        const auto& bodyAId = manifold.bodyAId;
        const auto& bodyBId = manifold.bodyBId;
        for (auto& contact : manifold.contacts)
        {
            collisionResolutions.push_back(collision_preStep(bodyAId, bodyBId, contact, dt));
        }
    }

    for (int i = 0; i < collisionIterations; ++i)
    {
        for (auto& resolution : collisionResolutions)
        {
            solve_contactConstraints(resolution);
        }
    }
}

void FizzWorld::handle_collisions(val_t dt)
{
    detect_collisions();
    resolve_collisions(dt);
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
        body.velocity = body.velocity * std::max(static_cast<val_t>(0), (1 - body.linearDamping * dt)) + accel * dt;
        body.position += body.velocity * dt;
        body.accumForce.setZero();

        val_t angularAccel = body.accumTorque * body.invMoI;
        body.angularVelocity = body.angularVelocity * std::max(static_cast<val_t>(0), (1 - body.angularDamping * dt)) + angularAccel * dt;
        body.rotation += body.angularVelocity * dt;
        body.accumTorque = 0;

        bool posUpdate = prevPos != body.position;
        bool rotUpdate = prevRot != body.rotation; 
        if(posUpdate || rotUpdate) 
            broadphase->update(ID, get_bounds(&body, rotUpdate), body.position + body.centroid);
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

    broadphase->add(ID, compute_bounds(&b), b.position + b.centroid);

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
        handle_collisions(timestep);
        destroy_bodies();
    }
}
};
