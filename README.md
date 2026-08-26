# Fizziks
Fizziks is a real-time 2d rigid body physics engine written in C++. Designed for deterministic simulation, educational clarity, and easy integration into games and simulations.

## Features
- Broadphase and narrowphase collision detection
- BVH acceleration structure for broadphase detection
- Per-step Iterative collision candidate pair detection in BVH 
- Convex collision detection (GJK + EPA)
- Concave to convex shape decomposition
- Impulse-based + warm-started collision resolution
- Constraint solving (currently just for contacts)
- Material properties (restitution, friction, etc.)
- No external dependencies required for integration
- Thread safe logging
- Google test suite

## Building
### Requirements
- C++20 compatible compiler
- CMake 3.28 - 4.2
- Only tested on Windows

### Build Steps
Fizziks is built using CMake. Use the provided build script or:
```bash
git clone https://github.com/Vessalia/Fizziks.git
cd Fizziks
mkdir build
cmake -S . -B build
cmake --build build
```

### Build Options
- `./build.sh [-h|--help]` - `-h` displays build options for the build script
- `BUILD_SHARED_LIBS=ON` - build as a shared library
- `FIZZIKS_BUILD_DIST=ON` - build a distributable
- `FIZZIKS_USE_GLM=ON` - switch the math backend from Eigen to GLM (this is pretty unnecessary, but I did it so it's here now)
- `ACTIVE_LOG_LEVEL=X` - sets the active log level. Levels are inclusive, meaning level 3 includes levels 1 and 2. Valid levels are:
  - `X = -1`, corresponding to the default logging level for your config
  - `X = 0`, corresponding to no logging
  - `X = 1`, corresponding to only critical failure messages
  - `X = 2`, corresponding to error messages
  - `X = 3`, corresponding to warning messages
  - `X = 4`, corresponding to info messages
  - `X = 5`, corresponding to debug messages
  - The build will fail if X is not one of these values

## Release Builds
- Aimed to work using either `use_subdirectory`, or `use_package`. If you desire debug symbols, then build using the `build.sh` script for the 4 most common configurations.
- The library uses the CMake alias `Fizziks::Fizziks`.

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
	Fizziks::SinkOptions options;
	options.threadSafe = true;
	addLogSink([](Fizziks::LogLevel level, std::string_view msg, std::string_view file, int line)
		{ 
			std::cout << "level = " << toString(level) << ": msg = " << msg << ": file = " << file << ": line = " << line << std::endl;
		}, options
	);

	FizzWorld world;

	BodyDef left = BodyDefBuilder()
		.setInitPosition({ 0, 0 })
		.setBodyType(BodyType::STATIC)
		.setColliderDefs({ ColliderDefBuilder().setShape(createRect(1, 20)).build() })
		.setRestitution(0.0f)
		.build();

	BodyDef right = BodyDefBuilder()
		.setInitPosition({ 20, 0 })
		.setBodyType(BodyType::STATIC)
		.setColliderDefs({ ColliderDefBuilder().setShape(createRect(1, 20)).build() })
		.setRestitution(0.0f)
		.build();

	BodyDef bottom = BodyDefBuilder()
		.setInitPosition({ 10, 0.5 })
		.setBodyType(BodyType::STATIC)
		.setColliderDefs({ ColliderDefBuilder().setShape(createRect(20, 1)).build() })
		.setRestitution(0.1f)
		.build();

	BodyDef poly = BodyDefBuilder()
		.setInitPosition({ 10, 10 })
		.setColliderDefs({
			ColliderDefBuilder()
				.setShape(createPolygon({
					Vec2(0, 0), Vec2(1, 0), Vec2(1, 1)
				}))
				.setMass(2)
				.build()
		})
		.build();

	world.createBody(left);
	world.createBody(right);
	world.createBody(bottom);
	world.createBody(poly);

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
- Contact manifolds
  - Concave shapes are *actually* handled now (GJK/EPA use convex hulls, so before we just broke things up and operated on their convex hull), but stability is poor since we don't operate on the contact manifold, just the face with the largest penetration
- Prevent deltatime debt spiral of death
  - sometimes when given a large dt, we can take as long or longer to simulate that elapsed time. Have max time spent draining accumulator before surrendering
- BVH raycasting/user data + callback abstraction
  - Collision event callbacks (collisionOnEnter/Exit/Stay)
  - RigidBody layermasking
- Collision Prediction
- Nearphase groups for multi-threading
- Contact joints
- Springs and strings

## References
- [Ming-Lun "Allen" Chou's Game Physics Series](https://allenchou.net/game-physics-series/)
- [Fantastic explanation of GJK from Casey Muratori](https://www.youtube.com/watch?v=Qupqu1xe7Io)
- [Erin Catto's GDC talk about using BVHs in Overwatch for ray casts](https://gdcvault.com/play/1025909/Math-for-Game-Developers-Dynamic) ([and the associated slides](https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf))
- [Andrew Kensler's paper on using tree rotations in BVHs to reduce the cost of the tree while minimizing the effect on construction times](http://eastfarthing.com/publications/tree.pdf)
- Classical Mechanics by John R. Taylor
