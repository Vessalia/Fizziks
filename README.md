# Fizziks
Fizziks is a real-time 2d rigid body physics engine written in C++. Designed for deterministic simulation, educational clarity, and easy integration into games and simulations.

## Features
- Broadphase and narrowphase collision detection
- Convex collision detection (GJK + EPA)
- Impulse-based + warm-started collision resolution
- Constraint solving (currently just for contacts)
- Material properties (restitution, friction, etc.)
- No external dependencies

## Building
### Requirements
- C++17 compatible compiler
- CMake 3.28 - 4.2
- Only tested on Windows

### Build Steps
Fizziks is built using CMake
```bash
git clone https://github.com/Vessalia/Fizziks.git
cd Fizziks
mkdir build
cmake -S . -B build
cmake --build build
```

### Build Options
- `BUILD_SHARED_LIBS=ON` - build as a shared library
- `FIZZIKS_BUILD_DIST=ON` - build a distributable

## Usage
- **RigidBody** - A user facing representation of a physics body. Implemented as a handle to the actual data stored in the `FizzWorld` it was created from.
- **FizzWorld** - Owns bodies and simulation state. Responsible for creating, simulating, and destroying `RigidBody`s.
- **Vec** - An alias to the underlying math library implementation of vectors and their math.
- **Shape** - Represents a generic shape. Does not store positional or rotational information. Polygon vertices are with respect to the shapes centroid.
- **Collider** - Represents a collidable shape in the `FizzWorld`. Since a `RigidBody` can contain multiple colliders, physical/spatial information needs to be assigned per collider.
- **BodyDef** - Used to initialize `RigidBody`s. 

## Example Usage
```c++
#include "Fizziks/Fizziks.h"
#include "Fizziks/FizzWorld.h"
#include "Fizziks/RigidBody.h"
#include "Fizziks/RigidDef.h"
#include "Fizziks/Shape.h"
#include "Fizziks/Vec.h"
#include "Fizziks/MathUtils.h"

using namespace Fizziks;

int main(int argc, char** argv) 
{
    FizzWorld world;

    BodyDef def;
    small.colliderDefs.push_back({ createCollider(createRect(0.25, 0.25), 1, 0), Vec2::Zero() });
    bodies.push_back(world.createBody(small));

    BodyDef big;
    big.colliderDefs.push_back({ createCollider(createCircle(1.4), 10, 0), Vec2::Zero() });
    big.initPosition = { 20, 5 };
    big.initVelocity = { -3, 0 };
    bodies.push_back(world.createBody(big));

    BodyDef stat;
    stat.colliderDefs.push_back({ createCollider(createPolygon({ Vec2(0, 0), Vec2(0, 1), Vec2(1, 0) }, 1, deg2rad(10)), Vec2::Zero()) });
    stat.BodyType = BodyType::STATIC;
    stat.initPosition = { 10, 2 };
    bodies.push_back(world.createBody(stat));

    float dt = 1 / 60.f;

    while(true)
    {
        world.tick(dt);
    }

    return 0;
}
```

## Demo
A demo of how to use this library can be found [here](https://github.com/Vessalia/PlayFizziks).

## Future Work
- Improve broadphase from brute-force to BVH or dynamic AABB tree (linked vs cache friendly array)
- RigidBody layermasking
- Logging (currently none)
- Nearphase groups for multi-threading
- Collision event callbacks
- Testing
- Contact joints
- Improved CMake
- Math backend selection

## References
- [Ming-Lun "Allen" Chou's Game Physics Series](https://allenchou.net/game-physics-series/)
- [Fantastic explanation of GJK from Casey Muratori](https://www.youtube.com/watch?v=XIavUJ848Mk&list=LL&index=15)
