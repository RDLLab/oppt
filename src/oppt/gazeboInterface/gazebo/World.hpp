#ifndef _GZ_WORLD_WRPR_HPP_
#define _GZ_WORLD_WRPR_HPP_
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/common/common.hh>

namespace oppt {
class World {
public:
	World(gazebo::physics::WorldPtr &world);

	std::string GetName() const;

	gazebo::physics::WorldPtr GetGazeboWorld() const;

	gazebo::physics::PhysicsEnginePtr GetPhysicsEngine() const;

	gazebo::physics::Model_V GetModels() const;

	gazebo::common::Time GetSimTime() const;

	unsigned int GetModelCount() const;

	void EnablePhysicsEngine(const bool &enable);

	bool GetEnablePhysicsEngine() const;

private:
	gazebo::physics::WorldPtr world_ = nullptr;
};
}

#endif