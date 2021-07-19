#include "oppt/opptCore/Body.hpp"

namespace oppt {
Body::Body(const std::string& name, const geometric::Pose &worldPose):
    name_(name),
    scopedName_(name + "::link"),
    worldPose_(worldPose),
    enabled_(true),
    static_(true) {

}

BodyUniquePtr Body::clone() const {
    geometric::Pose worldPose(worldPose_);
    BodyUniquePtr clonedBody(new Body(getName(), worldPose));
    clonedBody->scopedName_ = scopedName_;
    for (size_t i = 0; i != visualGeometries_.size(); ++i) {
        clonedBody->addVisualGeometry(std::move(visualGeometries_[i]->copy()));
    }

    for (size_t i = 0; i != opptCollisionObjects_.size(); ++i) {
        clonedBody->addCollisionGeometry(std::move(opptCollisionObjects_[i]->getCollisionGeometry()->copy()));
    }

    clonedBody->updateCollisionObjects();
    clonedBody->setEnabled(isEnabled());
    clonedBody->setStatic(isStatic());
    return std::move(clonedBody);
}

void Body::addCollisionGeometry(GeometryUniquePtr collisionGeometry) {
    if (!collisionGeometry)
        return;
    collisionGeometry->parentBody_ = this;
    OpptCollisionObjectUniquePtr opptCollisionObject =
        OpptCollisionObjectUniquePtr(new OpptCollisionObject(std::move(collisionGeometry)));
    opptCollisionObjects_.push_back(std::move(opptCollisionObject));
}

void Body::updateCollisionObjects() {
    for (auto &collisionObject : opptCollisionObjects_) {
        geometric::Pose pose = collisionObject->getCollisionGeometry()->getWorldPose();
        fcl::Quaternion3f q(pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z());
        fcl::Vec3f trans_vec(pose.position.x(), pose.position.y(), pose.position.z());
        collisionObject->getFCLCollisionObject()->setTransform(q, trans_vec);
        collisionObject->getFCLCollisionObject()->computeAABB();
    }
}

void Body::addVisualGeometry(GeometryUniquePtr visualGeometry) {
    if (!visualGeometry)
        return;
    visualGeometry->parentBody_ = this;
    visualGeometries_.push_back(std::move(visualGeometry));
}

CollisionReportSharedPtr Body::collides(const CollisionRequest *collisionRequest) const {
    CollisionReportSharedPtr collisionReport(new CollisionReport);
    for (auto &collisionObject : opptCollisionObjects_) {
        CollisionReportSharedPtr cp = collisionObject->collides(collisionRequest);            
        for (size_t i = 0; i != cp->collisionPairs.size(); ++i) {
            collisionReport->collisionPairs.push_back(cp->collisionPairs[i]);
        }
        for (size_t i = 0; i != cp->contacts.size(); ++i) {
            collisionReport->contacts.push_back(std::move(cp->contacts[i]));
        }
    }

    collisionReport->collides = collisionReport->collisionPairs.size() != 0 ? true : false;

    return collisionReport;
}

void Body::setWorldPose(const geometric::Pose &pose) {
    for (auto &opptCollisionObject : opptCollisionObjects_) {
        geometric::Pose relativeCollisionPose = opptCollisionObject->getCollisionGeometry()->getWorldPose() - worldPose_;
        opptCollisionObject->getCollisionGeometry()->setWorldPose(relativeCollisionPose + pose);
    }

    for (auto &visualGeometry : visualGeometries_) {
        geometric::Pose relativeVisualPose = visualGeometry->getWorldPose() - worldPose_;
        visualGeometry->setWorldPose(relativeVisualPose + pose);
    }

    worldPose_ = pose;
}

void Body::setEnabled(const bool& enabled) {
    enabled_ = enabled;
}

bool Body::isEnabled() const {
    return enabled_;
}

void Body::setStatic(const bool& isStatic) {
    static_ = isStatic;
}

bool Body::isStatic() const {
    return static_;
}

void Body::serializeToSDF(std::ostream& os) const
{
    std::string sdfString = toSDFString();
    os << sdfString;
}

std::string Body::toSDFString() const {
    std::string serString = "";
    geometric::Pose pose = getWorldPose();
    Vector3f eulerAngles = math::quaternionToEulerAngles(pose.orientation);

    //serString += "<sdf version='1.6'>";
    serString += "<model name=\"" + name_ + "\">";
    serString += "<pose frame=''>";
    serString += std::to_string(pose.position.x()) + " ";
    serString += std::to_string(pose.position.y()) + " ";
    serString += std::to_string(pose.position.z()) + " ";
    serString += std::to_string(eulerAngles[0]) + " ";
    serString += std::to_string(eulerAngles[1]) + " ";
    serString += std::to_string(eulerAngles[2]);
    serString += "</pose>";
    serString += "<static>";
    if (static_) {
        serString += "1";
    } else {
        serString += "0";
    }
    serString += "</static>";
    serString += "<link name='link'>";
    for (auto &collisionObject : opptCollisionObjects_) {
        serString += "<collision name='" + collisionObject->getCollisionGeometry()->getName() + "'>";        
        auto gzPose = (collisionObject->getCollisionGeometry()->getWorldPose() - worldPose_).toGZPose();
        serString += "<pose frame=''>";
#ifdef GZ_GT_7
        serString += std::to_string(gzPose.Pos().X()) + " ";
        serString += std::to_string(gzPose.Pos().Y()) + " ";
        serString += std::to_string(gzPose.Pos().Z()) + " ";
        serString += std::to_string(gzPose.Rot().X()) + " ";
        serString += std::to_string(gzPose.Rot().Y()) + " ";
        serString += std::to_string(gzPose.Rot().Z());
#else
        serString += std::to_string(gzPose.pos.x) + " ";
        serString += std::to_string(gzPose.pos.y) + " ";
        serString += std::to_string(gzPose.pos.z) + " ";
        serString += std::to_string(gzPose.rot.x) + " ";
        serString += std::to_string(gzPose.rot.y) + " ";
        serString += std::to_string(gzPose.rot.z);
#endif
        serString += "</pose>";
        serString += collisionObject->getCollisionGeometry()->toSDFString();
        serString += "</collision>";
    }

    for (auto &visualGeometry : visualGeometries_) {
        serString += "<visual name='" + visualGeometry->getName() + "'>";
        auto gzPose = (visualGeometry->getWorldPose() - worldPose_).toGZPose();
        serString += "<pose frame=''>";
#ifdef GZ_GT_7
        serString += std::to_string(gzPose.Pos().X()) + " ";
        serString += std::to_string(gzPose.Pos().Y()) + " ";
        serString += std::to_string(gzPose.Pos().Z()) + " ";
        serString += std::to_string(gzPose.Rot().X()) + " ";
        serString += std::to_string(gzPose.Rot().Y()) + " ";
        serString += std::to_string(gzPose.Rot().Z());
#else
        serString += std::to_string(gzPose.pos.x) + " ";
        serString += std::to_string(gzPose.pos.y) + " ";
        serString += std::to_string(gzPose.pos.z) + " ";
        serString += std::to_string(gzPose.rot.x) + " ";
        serString += std::to_string(gzPose.rot.y) + " ";
        serString += std::to_string(gzPose.rot.z);
#endif
        serString += "</pose>";
        serString += visualGeometry->toSDFString();
        serString += "</visual>";
    }

    serString += "</link>";
    serString += "</model>";
    return serString;
}

std::vector<OpptCollisionObject*> Body::getOpptCollisionObjects() const {
    std::vector<OpptCollisionObject*> collisionObjects;
    collisionObjects.reserve(opptCollisionObjects_.size());
    for (auto &collisionObject : opptCollisionObjects_) {
        collisionObjects.push_back(collisionObject.get());
    }

    return collisionObjects;
}

std::string Body::getName() const {
    return name_;
}

std::string Body::getScopedName() const {
    return scopedName_;
}

VectorGeometryPtr Body::getVisualGeometries() const {
    VectorGeometryPtr visualGeometries(visualGeometries_.size(), nullptr);
    for (size_t i = 0; i != visualGeometries_.size(); ++i) {
        visualGeometries[i] = visualGeometries_[i].get();
    }

    return visualGeometries;
}

void Body::setColor(const VectorFloat& color) {
    for (auto &visualGeometry : visualGeometries_) {
        visualGeometry->setColor(color);
    }
}

VectorFloat Body::getColor() const {
    if (visualGeometries_.size() != 0) {
        return visualGeometries_[0]->getColor();
    }

    return VectorFloat({0.0, 0.0, 0.0, 1.0});
}

geometric::Pose Body::getWorldPose() const {
    return worldPose_;
}
}