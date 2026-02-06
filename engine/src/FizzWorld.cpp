#include <Fizziks/FizzWorld.h>
#include <Fizziks/FizzWorldImpl.h>
#include <Fizziks/MathUtils.h>

#include <Fizziks/SimpleBP.h>
#include <Fizziks/BVH.h>

#include <Fizziks/LinearAllocator.h>

#include <algorithm>

#include <Fizziks/ScopeProfiler.h>

namespace Fizziks::internal
{
const FizzWorldImpl::BodyData FizzWorldImpl::null_body = 
{
    fizzmax<uint32_t>(),
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

FizzWorldImpl::FizzWorldImpl(size_t unitsX, size_t unitsY, int collisionIterations, val_t timestep, FizzWorld::AccelStruct accel)
    : unitsX(unitsX)
    , unitsY(unitsY)
    , accumulator(0)
    , timestep(timestep)
    , collisionIterations(collisionIterations) 
    {
        switch (accel)
        {
            case FizzWorld::AccelStruct::SIMPLE: 
                broadphase = new SimpleBP();
                break;
            case FizzWorld::AccelStruct::BVH:
                broadphase = new BVH();
                break;
        }

        for (int i = 0; i < threads.size(); ++i)
        {
            threadAllocators.push_back(new LinearAllocator(1000 * sizeof(ColliderContact))); // hard code 1000 contact slots per thread for now
        }
    }

FizzWorldImpl::~FizzWorldImpl() 
{ 
    delete broadphase; 
    for (auto* alloc : threadAllocators)
    {
        delete alloc;
    }
}

void FizzWorldImpl::apply_force(const RigidBodyImpl& rb, const Vec2& force, const Vec2& at)
{
    BodyData* body = get_body(rb);
    if(body) 
    {
        body->accumForce += force;
        Vec2 r = at - (body->position + body->centroid);
        body->accumTorque += crossproduct(r, force);
    }
}

void FizzWorldImpl::add_collider(const RigidBodyImpl& rb, const Collider& collider)
{
    BodyData* body = get_body(rb);
    if(body) add_collider(body, collider);
}

void FizzWorldImpl::add_collider(BodyData* body, const Collider& collider)
{
    body->colliders.push_back(collider);
    
    if(body->bodyType != BodyType::STATIC)
    {
        body->mass += collider.mass;
        body->invMass = 1 / body->mass;
    }

    Vec2 centroid = Vec2::Zero();
    for(auto coll : body->colliders)
    {
        centroid += coll.mass * coll.pos;
    }

    body->centroid = centroid * body->invMass;

    val_t MoI = 0;
    for(auto coll : body->colliders)
    {
        // parallel axis theorem
        Vec2 r = centroid - coll.pos;
        MoI += coll.MoI + coll.mass * r.dot(r);
    }

    body->MoI = MoI;
    body->invMoI = 1 / MoI;
}

std::vector<Collider> FizzWorldImpl::body_colliders(const RigidBodyImpl& rb) const
{
    auto* body = get_body(rb);
    if (body) return body->colliders;
    else      return null_body.colliders;
}

const AABB FizzWorldImpl::get_bounds(const BodyData* body, bool compute) const
{
    if (compute) return compute_bounds(body);
    else         return body->bounds;
}

const AABB FizzWorldImpl::compute_bounds(const BodyData* body) const
{
    AABB bounds{ 0, 0, Vec2::Zero() };
    if (body->colliders.size() == 0) return bounds;

    Vec2 min = vec_max(), max = vec_min();
    for (auto collider : body->colliders)
    {
        AABB minorBounds = getEncapsulatingAABB(collider.shape, collider.pos, body->rotation);
        Vec2 pos = body->position + collider.pos + minorBounds.offset;

        min.x = std::min(min.x, pos.x - minorBounds.hw);
        min.y = std::min(min.y, pos.y - minorBounds.hh);

        max.x = std::max(max.x, pos.x + minorBounds.hw);
        max.y = std::max(max.y, pos.y + minorBounds.hh);
    }
    bounds.hw  = (max.x - min.x) / 2;
    bounds.hh = (max.y - min.y) / 2;
    bounds.offset = min + Vec2(bounds.hw, bounds.hh) - (body->position + body->centroid);

    return bounds;
}

const FizzWorldImpl::BodyData* FizzWorldImpl::get_body(const RigidBodyImpl& rb) const
{
    if (rb.handle.index >= activeHandles.size()) return nullptr;
    Handle activeHandle = activeHandles[rb.handle.index];
    if(activeHandle.gen != rb.handle.gen) return nullptr;

    return &activeBodies[activeHandle.index];
}

FizzWorldImpl::BodyData* FizzWorldImpl::get_body(const RigidBodyImpl& rb)
{
    if (rb.handle.index >= activeHandles.size()) return nullptr;
    Handle activeHandle = activeHandles[rb.handle.index];
    if(activeHandle.gen != rb.handle.gen) return nullptr;

    return &activeBodies[activeHandle.index];
}

void FizzWorldImpl::set_body(const RigidBodyImpl& rb, const BodyDef& def)
{
    BodyData* body = get_body(rb);
    if(body) set_body(body, def);
}

void FizzWorldImpl::set_body(BodyData* body, const BodyDef& def)
{
    body->layer = def.layer;

    body->position = def.initPosition;
    body->velocity = def.initVelocity;

    body->rotation = def.initRotation;
    body->angularVelocity = def.initAngularVelocity;

    body->gravityScale = def.gravityScale;

    body->bodyType = def.bodyType;

    body->mass = 0;
    body->MoI = 0;

    for(auto collider : def.colliderDefs)
    {
        add_collider(body, collider);
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

Vec2 FizzWorldImpl::body_position(const RigidBodyImpl& rb) const 
{
    auto* body = get_body(rb);
    if (body) return body->position;
    else      return null_body.position;
}
void FizzWorldImpl::body_position(const RigidBodyImpl& rb, const Vec2& pos) 
{ 
    auto* body = get_body(rb);
    if (body) body->position = pos;
}

val_t FizzWorldImpl::body_rotation(const RigidBodyImpl& rb) const
{
    auto* body = get_body(rb);
    if (body) return body->rotation;
    else      return null_body.rotation;
}
void FizzWorldImpl::body_rotation(const RigidBodyImpl& rb, val_t rot)
{
    auto* body = get_body(rb);
    if (body) body->rotation = clamp_angle(rot);
}

Vec2 FizzWorldImpl::body_centroidPosition(const RigidBodyImpl& rb) const
{
    auto* body = get_body(rb);
    if (body) return body->position + body->centroid;
    else      return null_body.position;
}

Vec2 FizzWorldImpl::body_velocity(const RigidBodyImpl& rb) const 
{ 
    auto* body = get_body(rb);
    if (body) return body->velocity;
    else      return null_body.velocity;
}
void FizzWorldImpl::body_velocity(const RigidBodyImpl& rb, const Vec2& vel) 
{ 
    auto* body = get_body(rb);
    if (body) body->velocity = vel;
}

val_t FizzWorldImpl::body_angularVelocity(const RigidBodyImpl& rb) const 
{ 
    auto* body = get_body(rb);
    if (body) return body->angularVelocity;
    else      return null_body.angularVelocity;
}
void FizzWorldImpl::body_angularVelocity(const RigidBodyImpl& rb, val_t angVel) 
{ 
    auto* body = get_body(rb);
    if (body) body->angularVelocity = angVel;
}

val_t FizzWorldImpl::body_mass(const RigidBodyImpl& rb) const 
{ 
    auto* body = get_body(rb);
    if (body) return 1 / body->invMass;
    else      return 1 / null_body.invMass;
}
void FizzWorldImpl::body_mass(const RigidBodyImpl& rb, val_t m) 
{ 
    auto* body = get_body(rb);
    if (body) body->invMass = 1 / m;
}

val_t FizzWorldImpl::body_gravityScale(const RigidBodyImpl& rb) const
{
    auto* body = get_body(rb);
    if (body) return body->gravityScale;
    else      return null_body.gravityScale;
}
void FizzWorldImpl::body_gravityScale(const RigidBodyImpl& rb, val_t gs)
{
    auto* body = get_body(rb);
    if (body) body->gravityScale = gs;
}

BodyType FizzWorldImpl::body_bodyType(const RigidBodyImpl& rb) const 
{ 
    auto* body = get_body(rb);
    if (body) return body->bodyType;
    else      return null_body.bodyType;
}

void FizzWorldImpl::body_bodyType(const RigidBodyImpl& rb, const BodyType& type)
{
    auto* body = get_body(rb);
    if (body) body->bodyType = type;
}

uint32_t FizzWorldImpl::body_layer(const RigidBodyImpl& rb) const
{
    auto* body = get_body(rb);
    if (body) return body->layer;
    else      return null_body.layer;
}
void FizzWorldImpl::body_layer(const RigidBodyImpl& rb, uint32_t layer)
{
    auto* body = get_body(rb);
    if (body) body->layer = layer;
}

Vec2 FizzWorldImpl::get_worldPos(const BodyData& body, const Vec2& colliderPos) const
{
    return body.position + body.centroid + colliderPos.rotated(body.rotation);
}

val_t FizzWorldImpl::get_worldRotation(const BodyData& body, const Collider& collider) const
{
    return clamp_angle(body.rotation + collider.rotation);
}

FizzWorldImpl::CollisionManifold FizzWorldImpl::get_manifold(size_t idA, size_t idB, size_t allocIndex) const
{
    CollisionManifold manifold;
    manifold.bodyAId = idA;
    manifold.bodyBId = idB;
    manifold.allocIndex = allocIndex;
    bool first = true;

    const auto& bodyA = activeBodies[idA];
    const auto& bodyB = activeBodies[idB];
    for (uint32_t i = 0; i < bodyA.colliders.size(); ++i)
    {
        const auto& coll1 = bodyA.colliders[i];
        const Vec2 posA = get_worldPos(bodyA, coll1.pos);
        val_t rotA = get_worldRotation(bodyA, coll1);
        for (uint32_t j = 0; j < bodyB.colliders.size(); ++j)
        {
            const auto& coll2 = bodyB.colliders[j];
            const Vec2 posB = get_worldPos(bodyB, coll2.pos);
            val_t rotB = get_worldRotation(bodyB, coll2);
            const Contact contact = getShapeContact(coll1.shape, posA, rotA, coll2.shape, posB, rotB);

            if (contact.overlaps)
            {
                ColliderContact collContact = { i, j, contact };
                Allocator::Block block = threadAllocators[allocIndex]->write(&collContact, sizeof(ColliderContact));
                if (first) 
                {
                    manifold.allocContact = block; 
                    first = false;
                }
                ++manifold.numBlocks;
            }
        }
    }

    return manifold;
}

void FizzWorldImpl::detect_collisions()
{
    PROFILE_FUNCTION_AVG();
    // broadphase detection
    const CollisionPairs broadPairs = broadphase->computePairs();
    if (broadPairs.size() == 0) return;

    // nearphase detection
    std::vector<CollisionManifold> manifolds(broadPairs.size());

    auto processRange = [&](size_t begin, size_t end, size_t allocIndex)
    {
        for (size_t i = begin; i < end; ++i)
        {
            const auto [idA, idB] = broadPairs[i];
            manifolds[i] = get_manifold(idA, idB, allocIndex);
        }
    };

    if (broadPairs.size() > THREAD_THRESHOLD)
    {
        const size_t threadCount = std::min(threads.size(), broadPairs.size());
        const size_t base  = broadPairs.size() / threadCount;
        const size_t extra = broadPairs.size() % threadCount;

        size_t offset = 0;
        for (size_t t = 0; t < threadCount; ++t)
        {
            const size_t count = base + (t < extra ? 1 : 0);
            const size_t begin = offset;
            const size_t end = begin + count;
            offset = end;

            threadAllocators[t]->reset();
            threads.submit([t, begin, end, &processRange] 
            { 
                processRange(begin, end, t); 
            });
        }

        threads.wait();
    }
    else
    {
        threadAllocators[1]->reset();
        processRange(0, broadPairs.size(), 1);
    }

    collisionManifolds.reserve(collisionManifolds.size() + manifolds.size());
    for (auto& manifold : manifolds)
    {
        if (manifold.allocContact.byte_count) collisionManifolds.emplace_back(manifold);
    }
}

ContactKey FizzWorldImpl::makeContactKey(const CollisionResolution& resolution) const
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
FizzWorldImpl::CollisionResolution FizzWorldImpl::collision_preStep(uint32_t idA, uint32_t idB, uint32_t collIdA, uint32_t collIdB, const Contact& contact, val_t dt)
{
    CollisionResolution resolution;
    resolution.bodyAId = idA;
    resolution.bodyBId = idB;
    resolution.collIdA = collIdA;
    resolution.collIdB = collIdB;
    resolution.contact = contact;

    auto& bodyA = activeBodies[idA];
    auto& bodyB = activeBodies[idB];

    Vec2 rA = contact.contactPointWorldA - (bodyA.position + bodyA.centroid);
    Vec2 rB = contact.contactPointWorldB - (bodyB.position + bodyB.centroid);

    // warm starting
    ContactKey key = makeContactKey(resolution);
    auto it = warmStartCache.find(key);
    if (it != warmStartCache.end())
    {
        resolution.normalImpulse = warmStart * it->second.normalImpulse;
        resolution.tangentImpulse = warmStart * it->second.tangentImpulse;

        // apply impulses immediately (this is the actual warm start)
        Vec2 normalImpulse = resolution.normalImpulse * contact.normal;
        Vec2 tangentImpulse = resolution.tangentImpulse * contact.tangent;
        Vec2 impulse = normalImpulse + tangentImpulse;

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
    Vec2 vA = bodyA.velocity + crossproduct(bodyA.angularVelocity, rA);
    Vec2 vB = bodyB.velocity + crossproduct(bodyB.angularVelocity, rB);
    Vec2 dv = vB - vA;
    val_t closingVel = std::max(contact.normal.dot(dv) - slopRes, static_cast<val_t>(0));
    val_t restitution = (closingVel > restitutionThreshold) ? closingVel * std::min(bodyA.restitution, bodyB.restitution) : 0;
    resolution.bias = baumgarte + restitution;

    return resolution;
}

// dv  = vb + rb x wb - (va + ra x wa)
// Jv  = n . (vb - va) + (rb x n)wb - (ra x n)wa
//     = n . (dv)
// lam = (-Jv + b)/effMass
void FizzWorldImpl::solve_normalConstraint(CollisionResolution& resolution)
{
    auto& bodyA = activeBodies[resolution.bodyAId];
    auto& bodyB = activeBodies[resolution.bodyBId];
    const auto& contact = resolution.contact;

    Vec2 rA = contact.contactPointWorldA - (bodyA.position + bodyA.centroid);
    Vec2 rB = contact.contactPointWorldB - (bodyB.position + bodyB.centroid);
    Vec2 vA = bodyA.velocity + crossproduct(bodyA.angularVelocity, rA);
    Vec2 vB = bodyB.velocity + crossproduct(bodyB.angularVelocity, rB);
    Vec2 dv = vB - vA;
    val_t Jv = dv.dot(contact.normal);

    val_t lambda = resolution.invEffMass * -(Jv + resolution.bias);

    val_t oldImpulse = resolution.normalImpulse;
    resolution.normalImpulse = std::max(static_cast<val_t>(0), oldImpulse + lambda);
    lambda = resolution.normalImpulse - oldImpulse;
    Vec2 impulse = lambda * contact.normal;

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
void FizzWorldImpl::solve_frictionConstraint(CollisionResolution& resolution)
{
    auto& bodyA = activeBodies[resolution.bodyAId];
    auto& bodyB = activeBodies[resolution.bodyBId];
    const auto& contact = resolution.contact;

    Vec2 rA = contact.contactPointWorldA - (bodyA.position + bodyA.centroid);
    Vec2 rB = contact.contactPointWorldB - (bodyB.position + bodyB.centroid);
    Vec2 vA = bodyA.velocity + crossproduct(bodyA.angularVelocity, rA);
    Vec2 vB = bodyB.velocity + crossproduct(bodyB.angularVelocity, rB);
    Vec2 dv = vB - vA;
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
    Vec2 impulse = lambdaT * contact.tangent;

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

void FizzWorldImpl::solve_contactConstraints(CollisionResolution& resolution)
{
    solve_normalConstraint(resolution);
    solve_frictionConstraint(resolution);
}

void FizzWorldImpl::resolve_collisions(val_t dt)
{
    if (collisionManifolds.size() == 0) return;

    for (auto& manifold : collisionManifolds)
    {
        const auto bodyAId = manifold.bodyAId;
        const auto bodyBId = manifold.bodyBId;
        const auto* alloc = threadAllocators[manifold.allocIndex];
        const ColliderContact* collContacts = (ColliderContact*)alloc->read(manifold.allocContact);
        for (int i = 0; i < manifold.numBlocks; ++i)
        {
            const auto& collContact = collContacts[i];
            collisionResolutions.push_back(collision_preStep(bodyAId, bodyBId, 
                                                             collContact.collAId, collContact.collBId, 
                                                             collContact.contact, dt));
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

void FizzWorldImpl::handle_collisions(val_t dt)
{
    detect_collisions();
    resolve_collisions(dt);
}

void FizzWorldImpl::simulate_bodies(val_t dt, const Vec2& gravity)
{
    for (size_t ID = 0; ID < activeBodies.size(); ++ID)
    {
        auto& body = activeBodies[ID];
        if (body.bodyType == BodyType::STATIC) continue;

        Vec2 prevPos = body.position;
        val_t prevRot = body.rotation;

        Vec2 accel = body.accumForce * body.invMass + gravity * body.gravityScale;
        body.velocity = body.velocity * std::max(static_cast<val_t>(0), (1 - body.linearDamping * dt)) + accel * dt;
        body.position += body.velocity * dt;
        body.accumForce = Vec2::Zero();

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

void FizzWorldImpl::destroy_bodies()
{
    while (!destructionQueue.empty())
    {
        RigidBodyImpl rb = destructionQueue.front(); destructionQueue.pop();

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

RigidBodyImpl FizzWorldImpl::createBody(const BodyDef& def, FizzWorld* parent)
{
    uint32_t ID = activeBodies.size();
    BodyData b;
    b.accumForce = Vec2::Zero();
    b.accumTorque = 0;
    set_body(&b, def);
    broadphase->add(ID, b.bounds, b.position + b.centroid + b.bounds.offset);
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

    return { handle, parent };
}

void FizzWorldImpl::tick(val_t dt, const Vec2& gravity)
{
    accumulator += dt;
    while (accumulator >= timestep)
    {
        accumulator -= timestep;

        simulate_bodies(timestep, gravity);
        handle_collisions(timestep);
        destroy_bodies();
        currstep++;
    }
}
}

namespace Fizziks
{
FizzWorld::FizzWorld(size_t unitsX, size_t unitsY, int collisionIterations, val_t timestep, AccelStruct accel) 
    : impl(new internal::FizzWorldImpl(unitsX, unitsY, collisionIterations, timestep, accel)) { }

FizzWorld::~FizzWorld() { if (impl) { delete impl; } impl = nullptr;  }

RigidBody FizzWorld::createBody(const BodyDef& def)
{
    RigidBody rb;
    rb.impl = new internal::RigidBodyImpl();
    *rb.impl = impl->createBody(def, this);
    return rb;
}

void FizzWorld::destroyBody(RigidBody& rb)
{
    impl->destructionQueue.push(*rb.impl);
}

Vec2 FizzWorld::worldScale() const
{
    return { (val_t)impl->unitsX, (val_t)impl->unitsY };
}

void FizzWorld::tick(val_t dt)
{
    impl->tick(dt, Gravity);
}

std::vector<std::pair<AABB, Vec2>> FizzWorld::getBroadphaseDebugInfo() const
{
    return impl->broadphase->getDebugInfo();
}
}
