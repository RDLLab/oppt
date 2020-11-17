#include "World.hpp"
#include <gazebo/physics/World.hh>

namespace oppt {
World::World(gazebo::physics::WorldPtr &world):
	world_(world) {

}

std::string World::GetName() const {
#ifdef GZ_GT_7
	return world_->Name();
#else
	return world_->GetName();
#endif

}

gazebo::physics::WorldPtr World::GetGazeboWorld() const {
	return world_;
}

gazebo::physics::PhysicsEnginePtr World::GetPhysicsEngine() const {
#ifdef GZ_GT_7
	return world_->Physics();
#else
	return world_->GetPhysicsEngine();
#endif
}

gazebo::physics::Model_V World::GetModels() const {
#ifdef GZ_GT_7
	return world_->Models();
#else
	return world_->GetModels();
#endif
}

gazebo::common::Time World::GetSimTime() const {
#ifdef GZ_GT_7
	return world_->SimTime();
#else
	return world_->GetSimTime();
#endif
}

unsigned int World::GetModelCount() const {
#ifdef GZ_GT_7
	return world_->ModelCount();
#else
	return world_->GetModelCount();
#endif

}

void World::EnablePhysicsEngine(const bool &enable) {
#ifdef GZ_GT_7
	return world_->SetPhysicsEnabled(enable);
#else
	return world_->EnablePhysicsEngine(enable);
#endif

}

bool World::GetEnablePhysicsEngine() const {
#ifdef GZ_GT_7
	return world_->PhysicsEnabled();
#else
	return world_->GetEnablePhysicsEngine();
#endif

}

}