#include "oppt/opptCore/core.hpp"
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/distance.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"

using namespace fcl;

namespace oppt {
bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata)
{
	CollisionData* cdata_ = static_cast<CollisionData*>(cdata);
	const fcl::CollisionRequest& request = cdata_->request;
	fcl::CollisionResult& result = cdata_->result;

	if (cdata_->done) {
		cout << "DONE" << endl;
		return true;
	}

	size_t currentNumContacts = result.numContacts();
	fcl::collide(o1, o2, request, result);
	if (result.isCollision()) {
		cdata_->o1.push_back(o1);
		cdata_->o2.push_back(o2);
	}

	if (result.numContacts() > currentNumContacts) {
		//cdata_->o1.push_back(o1);
		//cdata_->o2.push_back(o2);
		if (!(request.enable_contact)) {
			cdata_->done = true;
		} else {
			if (!request.enable_cost && (result.numContacts() >= request.num_max_contacts)) {
				cdata_->done = true;
			}
		}
	}

	return cdata_->done;
}
}