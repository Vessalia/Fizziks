#include <FizzWorld.h>
#include <Shape.h>
#include <SimpleBP.h>

namespace Fizziks
{
const FizzWorld::BodyData FizzWorld::null_body = 
{
    vec_max(),
    vec_max(), vec_max(), vec_max(),
    fizzmax<val_t>(), fizzmax<val_t>(), fizzmax<val_t>(),
    fizzmax<val_t>(), fizzmax<val_t>(),
    fizzmax<val_t>(), fizzmax<val_t>(),
    fizzmax<val_t>(),
    fizzmax<val_t>(), fizzmax<val_t>(), fizzmax<val_t>(), fizzmax<val_t>(), fizzmax<val_t>(),
    {},
    {},
    BodyType::STATIC
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
        Vector2p r = at - (body->position + body->centroid);
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
    
    if(body->bodyType != BodyType::STATIC)
    {
        body->mass += collider.mass;
        body->invMass = 1 / body->mass;
    }

    Vector2p centroid = Vector2p::Zero();
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

std::vector<std::pair<Collider, Vector2p>> FizzWorld::body_colliders(const RigidBody& rb) const
{
    auto* body = get_body(rb);
    if (body) return body->colliders;
    else      return null_body.colliders;
}

const AABB FizzWorld::get_bounds(BodyData* body, bool compute) const
{
    if (compute) return compute_bounds(body);
    else         return body->bounds;
}

const AABB FizzWorld::compute_bounds(BodyData* body) const
{
    AABB bounds{ 0, 0, Vector2p::Zero() };
    if (body->colliders.size() == 0) return bounds;

    Vector2p min = vec_max(), max = vec_min();
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

    body->bodyType = def.bodyType;

    body->mass = 0;
    body->MoI = 0;

    for(auto [collider, at] : def.colliderDefs)
    {
        add_collider(body, collider, at);
    }

    if(def.bodyType == BodyType::STATIC)
    {
        body->mass = fizzmax<val_t>();
        body->invMass = 0;
        body->MoI = fizzmax<val_t>();
        body->invMoI = 0;
    }

    body->restitution = def.restitution;
    body->staticFriction = def.staticFrictionCoeff;
    body->dynamicFriction = def.dynamicFrictionCoeff;
    body->linearDamping = def.linearDamping;
    body->angularDamping = def.angularDamping;

    body->bounds = compute_bounds(body);
}

Vector2p FizzWorld::body_position(const RigidBody& rb) const 
{
    auto* body = get_body(rb);
    if (body) return body->position;
    else      return null_body.position;
}
void FizzWorld::body_position(const RigidBody& rb, const Vector2p& pos) 
{ 
    auto* body = get_body(rb);
    if (body) body->position = pos;
}

val_t FizzWorld::body_rotation(const RigidBody& rb) const
{
    auto* body = get_body(rb);
    if (body) return body->rotation;
    else      return null_body.rotation;
}
void FizzWorld::body_rotation(const RigidBody& rb, const val_t rot)
{
    auto* body = get_body(rb);
    if (body) body->rotation = clamp_angle(rot);
}

Vector2p FizzWorld::body_centroidPosition(const RigidBody& rb) const
{
    auto* body = get_body(rb);
    if (body) return body->position + body->centroid;
    else      return null_body.position;
}

Vector2p FizzWorld::body_velocity(const RigidBody& rb) const 
{ 
    auto* body = get_body(rb);
    if (body) return body->velocity;
    else      return null_body.velocity;
}
void FizzWorld::body_velocity(const RigidBody& rb, const Vector2p& vel) 
{ 
    auto* body = get_body(rb);
    if (body) body->velocity = vel;
}

val_t FizzWorld::body_angularVelocity(const RigidBody& rb) const 
{ 
    auto* body = get_body(rb);
    if (body) return body->angularVelocity;
    else      return null_body.angularVelocity;
}
void FizzWorld::body_angularVelocity(const RigidBody& rb, const val_t& angVel) 
{ 
    auto* body = get_body(rb);
    if (body) body->angularVelocity = angVel;
}

val_t FizzWorld::body_mass(const RigidBody& rb) const 
{ 
    auto* body = get_body(rb);
    if (body) return 1 / body->invMass;
    else      return 1 / null_body.invMass;
}
void FizzWorld::body_mass(const RigidBody& rb, val_t m) 
{ 
    auto* body = get_body(rb);
    if (body) body->invMass = 1 / m;
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

BodyType FizzWorld::body_bodyType(const RigidBody& rb) const 
{ 
    auto* body = get_body(rb);
    if (body) return body->bodyType;
    else      return null_body.bodyType;
}

void FizzWorld::body_bodyType(const RigidBody& rb, const BodyType& type)
{
    auto* body = get_body(rb);
    if (body) body->bodyType = type;
}

Vector2p FizzWorld::get_worldPos(const BodyData& body, const Vector2p& colliderPos) const
{
    return body.position + body.centroid + Rotation2p(body.rotation) * colliderPos;
}

val_t FizzWorld::get_worldRotation(const BodyData& body, const Collider& collider) const
{
    return clamp_angle(body.rotation + collider.rotation);
}

val_t FizzWorld::clamp_angle(val_t rot) const
{
    rot -= TWO_PI * std::floor(rot / TWO_PI);
    return rot;
}

FizzWorld::CollisionManifold FizzWorld::get_manifold(const size_t idA, const size_t idB) const
{
    CollisionManifold manifold;
    manifold.bodyAId = idA;
    manifold.bodyBId = idB;

    const auto& bodyA = activeBodies[idA];
    const auto& bodyB = activeBodies[idB];
    for (uint32_t i = 0; i < bodyA.colliders.size(); ++i)
    {
        const auto& [s1, p1] = bodyA.colliders[i];
        Vector2p posA = get_worldPos(bodyA, p1);
        val_t rotA = get_worldRotation(bodyA, s1);
        for (uint32_t j = 0; j < bodyB.colliders.size(); ++j)
        {
            const auto& [s2, p2] = bodyB.colliders[j];
            Vector2p posB = get_worldPos(bodyB, p2);
            val_t rotB = get_worldRotation(bodyB, s2);
            Contact contact = getShapeContact(s1.shape, posA, rotA, s2.shape, posB, rotB);

            if (contact.overlaps) manifold.contacts.push_back({ i, j, contact } );
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

ContactKey FizzWorld::makeContactKey(const CollisionResolution& resolution) const
{
    return { resolution.bodyAId, resolution.bodyBId, resolution.collIdA, resolution.collIdB, resolution.contact.featureA, resolution.contact.featureB };
}

val_t beta = 0.2;
val_t restitutionThreshold = 0.001;
val_t slopPen = 0.01;
val_t slopRes = 0.01;
val_t warmStart = 0.75;
// precompute inverse effective mass (JM^-1J^T)^-1 and bias term
// J = [-n^T -(rA x n)^T n^T (rB x n)^T]
// M^-1 = [Ma^-1 0 0 0]
//        [0 Ia^-1 0 0]
//        [0 0 Mb^-1 0]
//        [0 0 0 Ib^-1]
// JM^-1    = [-n^TMa^-1 -(rA x n)^TIa^-1 n^TMb^-1 (rB x n)^TIb^-1]
// JM^-1J^T = |n|^2Ma^-1 + |rA x n|^2 Ia^-1 + |n|^2Mb^-1 + |rB x n|^2 Ib^-1
//          = Ma^-1 + |rA x n|^2 Ia^-1 + Mb^-1 + |rB x n|^2 Ib^-1
FizzWorld::CollisionResolution FizzWorld::collision_preStep(const uint32_t idA, const uint32_t idB, const uint32_t collIdA, const uint32_t collIdB, const Contact& contact, const val_t dt)
{
    CollisionResolution resolution;
    resolution.bodyAId = idA;
    resolution.bodyBId = idB;
    resolution.collIdA = collIdA;
    resolution.collIdB = collIdB;
    resolution.contact = contact;

    auto& bodyA = activeBodies[idA];
    auto& bodyB = activeBodies[idB];

    Vector2p rA = contact.contactPointWorldA - (bodyA.position + bodyA.centroid);
    Vector2p rB = contact.contactPointWorldB - (bodyB.position + bodyB.centroid);

    // warm starting
    ContactKey key = makeContactKey(resolution);
    auto it = warmStartCache.find(key);
    if (it != warmStartCache.end())
    {
        resolution.normalImpulse = warmStart * it->second.normalImpulse;
        resolution.tangentImpulse = warmStart * it->second.tangentImpulse;

        // apply impulses immediately (this is the actual warm start)
        Vector2p normalImpulse = resolution.normalImpulse * contact.normal;
        Vector2p tangentImpulse = resolution.tangentImpulse * contact.tangent;
        Vector2p impulse = normalImpulse + tangentImpulse;

        if (bodyA.bodyType == BodyType::DYNAMIC)
        {
            bodyA.velocity -= impulse * bodyA.invMass;
            bodyA.angularVelocity -= crossproduct(rA, impulse) * bodyA.invMoI;
        }

        if (bodyB.bodyType == BodyType::DYNAMIC)
        {
            bodyB.velocity += impulse * bodyB.invMass;
            bodyB.angularVelocity += crossproduct(rB, impulse) * bodyB.invMoI;
        }
    }
    else
    {
        resolution.normalImpulse = 0;
        resolution.tangentImpulse = 0;
    }

    val_t rnA = crossproduct(rA, contact.normal);
    val_t rtA = crossproduct(rA, contact.tangent);
    val_t rnB = crossproduct(rB, contact.normal);
    val_t rtB = crossproduct(rB, contact.tangent);

    val_t effMass        = bodyA.invMass + bodyA.invMoI * rnA * rnA +
                           bodyB.invMass + bodyB.invMoI * rnB * rnB;
    val_t effTangentMass = bodyA.invMass + bodyA.invMoI * rtA * rtA +
                           bodyB.invMass + bodyB.invMoI * rtB * rtB;

    resolution.invEffMass = effMass > 0 ? 1 / effMass : fizzmax<val_t>();
    resolution.invEffTangentMass = effTangentMass > 0 ? 1 / effTangentMass : fizzmax<val_t>();

    val_t baumgarte = -beta / dt * std::max(contact.penetration - slopPen, static_cast<val_t>(0));
    Vector2p vA = bodyA.velocity + crossproduct(bodyA.angularVelocity, rA);
    Vector2p vB = bodyB.velocity + crossproduct(bodyB.angularVelocity, rB);
    Vector2p dv = vB - vA;
    val_t closingVel = std::max(contact.normal.dot(dv) - slopRes, static_cast<val_t>(0));
    val_t restitution = (closingVel > restitutionThreshold) ? closingVel * std::min(bodyA.restitution, bodyB.restitution) : 0;
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

    val_t lambda = resolution.invEffMass * -(Jv + resolution.bias);

    val_t oldImpulse = resolution.normalImpulse;
    resolution.normalImpulse = std::max(static_cast<val_t>(0), oldImpulse + lambda);
    lambda = resolution.normalImpulse - oldImpulse;
    Vector2p impulse = lambda * contact.normal;

    if (bodyA.bodyType == BodyType::DYNAMIC)
    {
        bodyA.velocity -= impulse * bodyA.invMass;
        bodyA.angularVelocity -= crossproduct(rA, impulse) * bodyA.invMoI;
    }

    if (bodyB.bodyType == BodyType::DYNAMIC)
    {
        bodyB.velocity += impulse * bodyB.invMass;
        bodyB.angularVelocity += crossproduct(rB, impulse) * bodyB.invMoI;
    }
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

    val_t lambdaT = resolution.invEffTangentMass * (-Jvt);
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

    if (bodyA.bodyType == BodyType::DYNAMIC)
    {
        bodyA.velocity -= impulse * bodyA.invMass;
        bodyA.angularVelocity -= crossproduct(rA, impulse) * bodyA.invMoI;
    }

    if (bodyB.bodyType == BodyType::DYNAMIC)
    {
        bodyB.velocity += impulse * bodyB.invMass;
        bodyB.angularVelocity += crossproduct(rB, impulse) * bodyB.invMoI;
    }
}

void FizzWorld::solve_contactConstraints(CollisionResolution& resolution)
{
    solve_normalConstraint(resolution);
    solve_frictionConstraint(resolution);
}

void FizzWorld::resolve_collisions(val_t dt)
{
    if (collisionManifolds.size() == 0) return;

    for (auto& manifold : collisionManifolds)
    {
        const auto& bodyAId = manifold.bodyAId;
        const auto& bodyBId = manifold.bodyBId;
        for (auto& [collIdA, collIdB, contact] : manifold.contacts)
        {
            collisionResolutions.push_back(collision_preStep(bodyAId, bodyBId, collIdA, collIdB, contact, dt));
        }
    }
    collisionManifolds.clear();

    for (int i = 0; i < collisionIterations; ++i)
    {
        for (auto& resolution : collisionResolutions)
        {
            solve_contactConstraints(resolution);
        }
    }

    std::unordered_map<ContactKey, CollisionResolution> newCache;
    for (auto& resolution : collisionResolutions)
    {
        ContactKey key = makeContactKey(resolution);
        newCache[key] = resolution;
    }
    warmStartCache = std::move(newCache);
    collisionResolutions.clear();
}

void FizzWorld::handle_collisions(val_t dt)
{
    if (currstep == 952)
    {
        int x = 1;
    }

    detect_collisions();
    resolve_collisions(dt);
}

void FizzWorld::simulate_bodies(val_t dt)
{
    for (size_t ID = 0; ID < activeBodies.size(); ++ID)
    {
        auto& body = activeBodies[ID];
        if (body.bodyType == BodyType::STATIC) continue;

        Vector2p prevPos = body.position;
        val_t prevRot = body.rotation;

        Vector2p accel = body.accumForce * body.invMass + Gravity * body.gravityScale;
        body.velocity = body.velocity * std::max(static_cast<val_t>(0), (1 - body.linearDamping * dt)) + accel * dt;
        body.position += body.velocity * dt;
        body.accumForce.setZero();

        val_t angularAccel = body.accumTorque * body.invMoI;
        body.angularVelocity = body.angularVelocity * std::max(static_cast<val_t>(0), (1 - body.angularDamping * dt)) + angularAccel * dt;
        body.rotation = clamp_angle(body.rotation + body.angularVelocity * dt);
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
    uint32_t ID = activeBodies.size();
    BodyData b;
    b.accumForce = Vector2p::Zero();
    b.accumTorque = 0;
    set_body(&b, def);
    broadphase->add(ID, b.bounds, b.position + b.centroid);
    activeBodies.push_back(std::move(b));

    Handle handle{ static_cast<uint32_t>(activeHandles.size()), 0 };
    if (!freeList.empty()) 
    {
        uint32_t freeIndex = freeList.back(); freeList.pop_back();
        handle.index = freeIndex;
        handle.gen = activeHandles[freeIndex].gen;
        activeHandles[freeIndex].index = ID;
        activeList.push_back(freeIndex);
    }
    else 
    {
        activeHandles.push_back(handle);
        activeList.push_back(handle.index);
    }

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
        currstep++;
    }
}
};
