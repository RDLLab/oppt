/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */
#include "oppt/gazeboInterface/GazeboInterface.hpp"
#include <gazebo/gazebo.hh>
#include "oppt/opptCore/resources/resources.hpp"
#include "gazebo/OpptODEPhysics.hpp"
#include "ServerInterface.hpp"
#include "GazeboSubscriber.hpp"
#include <gazebo/physics/ode/ODEJoint.hh>
#include "oppt/global.hpp"
#include "oppt/robotHeaders/RobotState.hpp"


//std::unique_ptr<gazebo::Server> server;
boost::mutex serverMutex;
std::unique_ptr<int> dummyInt;

namespace oppt
{

static boost::mutex odeMtx;

void onODEContact(const std::string& h)
{

}

GazeboInterface::GazeboInterface(std::string& gazeboWorldFile,
                                 std::string& robotName,
                                 std::string& prefix,
                                 unsigned int threadId):
    robots_(),
    mainRobotName_(""),
    initialWorldModelMap_(),
    worldJointMap_(),
    worldRobotLinkMap_(),
    worldLinkMap_(),
    contactSubs_(),
    stateSpaceInformation_(nullptr),
    actionSpaceInformation_(nullptr),
    observationSpaceInformation_(nullptr),
    stateSpaceDimension_(0),
    addEntityConnection_(nullptr),
    addBodyCallback_(nullptr),
    bodyEventQueue_(new BodyEventQueue()),
    processBodiesEventsThread_(nullptr),
    finishProcessingBodies_(false),
    threadId_(threadId),
    sensorInterface_(new SensorInterface()),
    setStateFunctions_(),
    getStateFunctions_(),
    applyActionFunctions_(),
    getObservationFunctions_(),
    collisionCheckedLinks_(),
    collisionFunction_(nullptr),
    deferredPoseChanges_(),
    prefix_(prefix),
    saveUserData_(true),
    rootJoints_()
{
    checkSDFValidity(gazeboWorldFile, robotName);
    if (!gazebo::physics::PhysicsFactory::IsRegistered("ode2")) {
        RegisterOpptODEPhysics();
    }

    mainRobotName_ = robotName;
    initWorldFromFile(gazeboWorldFile, threadId_);
    ModelPtr robotModel = findRobotModel(mainRobotName_);
    initRobotModel(world_->GetName(),
                   robotModel,
                   worldRobotLinkMap_);
    initWorldLinkMap(worldLinkMap_);
    initWorldJointMap(worldJointMap_);

    std::string topicStr = "/gazebo/" + world_->GetName() + "/physics/contacts";
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    contactSubs_[world_->GetName()] = node->Subscribe(topicStr, onODEContact);
    initialCumulativeAngles_ =
        static_cast<gazebo::physics::OpptODEPhysics*>(world_->GetPhysicsEngine().get())->getCumulativeAngles();
    initialWorldState_ = makeWorldState();
    lastConstructedWorldState_ = initialWorldState_;

    collisionFunction_ = [this](void * const data) {
        world_->GetPhysicsEngine()->UpdateCollision();
        bool collided = false;
        if (world_->GetPhysicsEngine()->GetContactManager()->GetContactCount() > 0) {
            for (auto & contact : world_->GetPhysicsEngine()->GetContactManager()->GetContacts()) {
                std::string collisionLink1 = contact->collision1->GetLink()->GetName();
                std::string collisionLink2 = contact->collision2->GetLink()->GetName();
                std::unique_ptr<ContactPoint> contactPoint(new ContactPoint());
                contactPoint->contactBody1 = collisionLink1;
                contactPoint->contactBody2 = collisionLink2;
                GZVector3 position = contact->positions[0];

#ifdef GZ_GT_7
                contactPoint->contactPointWorldPosition = VectorFloat( {position.X(), position.Y(), position.Z()});
#else
                contactPoint->contactPointWorldPosition = VectorFloat( {position.x, position.y, position.z});
#endif
                static_cast<ContactInformation* const>(data)->contactPoints.push_back(std::move(contactPoint));
                if (contains(collisionCheckedLinks_, collisionLink1) || contains(collisionCheckedLinks_, collisionLink2)) {
                    collided = true;
                }
            }
        }

        oppt::CollisionReportSharedPtr collisionReport(new CollisionReport());
        collisionReport->collides = collided;

        return collisionReport;
    };

    lastSimTime_ = world_->GetSimTime();
    if (prefix_ == "exec") {
        gazeboSubscriber_ = std::unique_ptr<GazeboSubscriber> (new GazeboSubscriber(this, world_->GetName()));
    }

    makeInitialWorldModelMap();
    initRootJoints();
}

GazeboInterface::~GazeboInterface()
{
    gazeboSubscriber_.reset();
    finishProcessingBodies_ = true;
    if (processBodiesEventsThread_) {
        processBodiesEventsThread_->join();
        delete processBodiesEventsThread_;
    }
    /**#ifdef GZ_GT_7
        if (world_->GetGazeboWorld()->Running()) {
    #else
        if (world_->GetGazeboWorld()->GetRunning()) {
    #endif
            world_->GetGazeboWorld()->Stop();
        }*/

    //world_->GetGazeboWorld()->Fini();

    //delete g_master;

    gazebo::transport::stop();
    gazebo::transport::fini();
    serverInterface_.stop();
}

void GazeboInterface::setSnapVelocities(const bool &snap) {
    snapVelocities_ = snap;
}

void GazeboInterface::initRootJoints()
{
    auto robotLinks = worldRobotLinkMap_.at(world_->GetName());
    std::vector<LinkPtr> leafLinks;
    for (auto &linkEntry : robotLinks) {
        if (linkEntry.second->GetChildJoints().empty()) {
            leafLinks.push_back(linkEntry.second);
        }
    }

    for (auto &link : leafLinks) {
        LinkPtr currentLink = link;
        while (currentLink->GetParentJoints().empty() == false) {
            if (currentLink->GetParentJointsLinks().empty())
                break;

            currentLink = currentLink->GetParentJointsLinks()[0];
        }

        auto chJoints = currentLink->GetChildJoints();
        for (auto &joint : chJoints) {
            rootJoints_[joint->GetName()] = joint;
        }
    }
}

std::string GazeboInterface::getWorldName(std::string& gazeboWorldFile,
        const unsigned int& threadId)
{
    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readFile(gazeboWorldFile, sdfModel);
    sdf::ElementPtr rootElement = sdfModel->Root();
    sdf::ElementPtr worldElement = rootElement->GetElement("world");
    std::string worldName = worldElement->Get<std::string> ("name");

    worldName += "_" + std::to_string(threadId);
    return worldName;
}

void GazeboInterface::initWorldFromFile(const std::string& gazeboWorldFile,
                                        const unsigned int& threadId)
{
    try {
        unsigned int inc = 0;
        if (!dummyInt) {
            dummyInt = std::make_unique<int>(32);
            bool loaded = false;
            LOGGING("Initializing Gazebo");
            while (!loaded) {
                try {
                    serverInterface_.setupServer(0, NULL, inc);
                    //server->PreLoad();
                    loaded = true;
                } catch (gazebo::common::Exception& e) {
                    // If we can't setup a server with the given port number,
                    // we increase the port number and try again
                    cout << "Gazebo exception: " << e << endl;
                    inc++;
                    if (inc == 1000) {
                        ERROR("Couldn't initilize gzserver after trying 1000 different ports");
                    }
                }
            }
        }

        WorldPtr world = loadWorld(gazeboWorldFile);
        world_ = std::make_unique<World>(world);
        static_cast<gazebo::physics::OpptODEPhysics*>(world_->GetPhysicsEngine().get())->makeJointNameMap();
        std::string wn = world_->GetName();
        gazebo::sensors::run_once(true);
    } catch (gazebo::common::Exception& e) {
        std::string msg = e.GetErrorStr();
        ERROR(msg);
    }
}

WorldPtr GazeboInterface::loadWorld(const std::string& gazeboWorldFile)
{

    // Load the world file
    sdf::SDFPtr sdf(new sdf::SDF);
    if (!sdf::init(sdf)) {
        gzerr << "Unable to initialize sdf\n";
        ERROR("GazeboInterface: loadWorld: Unable to initialize sdf\n");
    }

    sdf::readFile(gazeboWorldFile, sdf);
    gazebo::physics::WorldPtr world = gazebo::physics::create_world();
    sdf::ElementPtr worldElement = sdf->Root()->GetElement("world");

    // Make sure that only ODE is allowed as physics engine
    // Change it to ode2
    sdf::ElementPtr physicsElement = worldElement->GetElement("physics");
    if (!physicsElement) {
        ERROR("SDF has no physics element?!");
    }
    sdf::ParamPtr physicsTypeParam = physicsElement->GetAttribute("type");
    if (!physicsTypeParam) {
        ERROR("No physics type param?!");
    }
    std::string physicsTypeStr = physicsTypeParam->GetAsString();
    if (physicsTypeStr != "ode" && physicsTypeStr != "ode2") {
        ERROR("You have selected an unsupported physics engine in your SDF file. Currently only ODE is supported");
    }
    physicsTypeParam->Set<std::string> ("ode2");
    gazebo::physics::load_world(world, worldElement);
    gazebo::physics::init_world(world);
    world->SetPaused(true);
#ifdef GZ_GT_7
    world->Physics()->UpdateCollision();
#else
    world->GetPhysicsEngine()->UpdateCollision();
#endif

    initialSDFWorldState_ = GazeboWorldStatePtr(new GazeboWorldState(world));
    return world;
}


std::vector<gazebo::physics::Link *> GazeboInterface::getLinks() const
{
    auto models = world_->GetModels();
    std::vector<gazebo::physics::Link *> links;
    for (auto model : models) {
        auto modelLinks = model->GetLinks();
        for (auto &link : modelLinks) {
            links.push_back(link.get());
        }
        //links.insert(links.end(), modelLinks.begin(), modelLinks.end());
    }

    return links;
    /**std::string worldName = world_->GetName();
    std::vector<LinkPtr> links(worldLinkMap_.at(worldName).size());
    size_t c = 0;
    for (auto & linkEntry : worldLinkMap_.at(worldName)) {
        links[c] = linkEntry.second;
        c++;
    }

    return links;*/
}

std::vector<gazebo::physics::Joint *> GazeboInterface::getJoints() const
{
    std::vector<gazebo::physics::Joint *> joints;
    auto models = world_->GetModels();
    for (auto & model : models) {
        auto modelJoints = model->GetJoints();
        for (auto & modelJoint : modelJoints) {
            joints.push_back(modelJoint.get());
        }
    }

    return joints;
}

void GazeboInterface::initRobotModel(const std::string& worldName,
                                     ModelPtr& model,
                                     WorldLinkMap& worldRobotLinkMap)
{
    //dBodySetMovedCallback(0, 0);
    sensorInterface_->init(worldName);

    //worldJointMap[worldName] = std::unordered_map<std::string, gazebo::physics::JointPtr>();
    worldRobotLinkMap[worldName] = std::unordered_map<std::string, gazebo::physics::LinkPtr>();

    if (model) {
        /**std::vector<gazebo::physics::JointPtr> jointsTemp = model->GetJoints();
        for (size_t i = 0; i < jointsTemp.size(); i++) {
            std::string scopedName = jointsTemp[i]->GetScopedName();
            worldJointMap[worldName][scopedName] = jointsTemp[i];
        }*/

        for (auto & link : model->GetLinks()) {
            std::string scopedName = link->GetScopedName();
            worldRobotLinkMap[worldName][scopedName] = link;
        }
    }
}

void GazeboInterface::initWorldJointMap(WorldJointMap &worldJointMap) {
    worldJointMap[world_->GetName()] = std::unordered_map<std::string, gazebo::physics::JointPtr>();
    auto models = world_->GetModels();
    for (auto &model : models) {
        std::vector<gazebo::physics::JointPtr> modelJoints = model->GetJoints();
        for (auto &joint : modelJoints) {
            std::string scopedName = joint->GetScopedName();
            worldJointMap[world_->GetName()][scopedName] = joint;
        }
    }

}

void GazeboInterface::initWorldLinkMap(WorldLinkMap& worldLinkMap)
{
    worldLinkMap[world_->GetName()] = std::unordered_map<std::string, gazebo::physics::LinkPtr>();
    auto models = world_->GetModels();
    for (auto & model : models) {
        auto links = model->GetLinks();
        for (auto & link : links) {
            std::string scopedName = link->GetScopedName();
            /**std::string unscopedName = scopedName;
            if (scopedName.find("::") != std::string::npos) {
                VectorString nameElems;
                split(scopedName, "::", nameElems);
                unscopedName = nameElems[nameElems.size() - 1];
            }*/

            worldLinkMap[world_->GetName()][scopedName] = link;
        }
    }
}

void GazeboInterface::checkSDFValidity(const std::string& worldFile, const std::string& robotName)
{
    sdf::SDFPtr sdfWorldModel(new sdf::SDF());
    sdf::init(sdfWorldModel);
    sdf::readFile(worldFile, sdfWorldModel);
    sdf::ElementPtr rootElement = sdfWorldModel->Root();
    sdf::ElementPtr worldElement = rootElement->GetElement("world");
    sdf::ElementPtr modelElement = worldElement->GetElement("model");
    bool robotInWorld = false;
    while (modelElement) {
        if (modelElement->HasAttribute("name")) {
            auto modelName = modelElement->Get<std::string> ("name");
            if (modelName == robotName) {
                robotInWorld = true;
            }
        }

        modelElement = modelElement->GetNextElement();
    }

    if (!robotInWorld) {
        ERROR("The robot '" + robotName + "' specified in your configuration file couldn't be found in the environment file");
    }
}

void GazeboInterface::setInteractive(const bool& interactive)
{
    if (prefix_ == "exec" && !worldRunning_) {
        //if (interactive && !worldRunning_ && prefix_ == "exec") {
        world_->GetGazeboWorld()->SetPaused(true);
        world_->GetGazeboWorld()->Run();
        /**processBodiesEventsThread_ = new boost::thread(&GazeboInterface::processBodyEvents,
                this,
                bodyEventQueue_);
        std::string poseTopicStr = "/gazebo/" + world_->GetName() + "/pose/info";
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();
        worldRunning_ = true;*/
    } else {
        processBodiesEventsThread_ = NULL;
    }
}

void GazeboInterface::setDirtyCollisionFunction(DirtyCollisionFunction dirtyCollisionFunction)
{
    collisionFunction_ = dirtyCollisionFunction;
}

void GazeboInterface::removeBodiesFromWorld()
{
    ERROR("Remove bodies from world");
    std::string modelName;
    size_t currentModelSize = world_->GetModels().size();
    size_t numModelsToDelete = 0;
    for (auto & model : world_->GetModels()) {
        modelName = model->GetName();
        if (modelName != mainRobotName_) {
            numModelsToDelete += 1;
            gazebo::transport::requestNoReply(world_->GetName(), "entity_delete", modelName);
        }
    }
    while (world_->GetModels().size() > (currentModelSize - numModelsToDelete)) {
        usleep(0.001 * 1e6);
    }
}

LinkPtr GazeboInterface::findRootLink(const ModelPtr& model)
{
    std::vector<JointPtr> joints = model->GetJoints();
    VectorString parents;
    VectorString children;
    for (auto & joint : joints) {
        parents.push_back(joint->GetParent()->GetName());
        children.push_back(joint->GetChild()->GetName());
    }

    for (auto & parent : parents) {
        bool found = false;
        for (auto & child : children) {
            if (parent == child) {
                found = true;
                break;
            }
        }

        if (!found) {
            return model->GetLink(parent);
        }
    }

    return nullptr;
}

void GazeboInterface::processBodyEvents(std::shared_ptr<BodyEventQueue>& bodyEventQueue)
{
    while (true) {
        usleep(0.005 * 1e6);
        boost::mutex::scoped_lock scoped_lock(mtx_);
        if (bodyEventQueue->size() > 0) {
            auto bodyEvent = bodyEventQueue->front();
            std::string bodyScopedName = bodyEvent.second;
            VectorString bodyNameElems;
            split(bodyScopedName, "::", bodyNameElems);
            std::string worldName = bodyNameElems[0];
            std::string bodyName = bodyNameElems[bodyNameElems.size() - 1];
            if (bodyEvent.first == "add") {
                // The name of the world in which the change occured
                std::vector<ModelPtr> models = world_->GetModels();
                for (auto & model : models) {
                    if (model->GetName() == bodyName) {
                        // For now we set the model to static by removing the current
                        // model instance and replacing it with a static version
                        std::string modelName = model->GetName();
                        std::vector<ModelPtr> currentModelsT = world_->GetModels();
                        sdf::ElementPtr modelElem = model->GetSDF();
                        std::string currentName = modelElem->Get<std::string> ("name");
                        sdf::ElementPtr staticElement = modelElem->GetElement("static");
                        staticElement->Set(true);
                        std::string sdfString = modelElem->ToString("");
                        gazeboSubscriber_->suppressProcessing(true);

                        // Temporarily disconnect event
#ifdef GZ_GT_7
                        //addEntityConnection_ = nullptr;
#else
                        //gazebo::event::Events::DisconnectAddEntity(addEntityConnection_);
#endif
                        addBodyFromSDFString(sdfString);
                        /**addEntityConnection_ =
                            gazebo::event::Events::ConnectAddEntity(std::bind(&GazeboInterface::onAddBody,
                                    this,
                                    std::placeholders::_1));*/
                        addBodyCallback_(sdfString);
                        gazeboSubscriber_->suppressProcessing(false);

                    }
                }
            }

            bodyEventQueue->pop();
        }
        if (finishProcessingBodies_) {
            break;
        }
    }

}

void GazeboInterface::onAddBody(const std::string& str)
{
    bool contains = false;
    auto models = world_->GetModels();
    for (auto & model : models) {
        if (model->GetScopedName() == str) {
            contains = true;
            break;
        }
    }

    if (!contains && prefix_ == "exec") {
        LOGGING("ADD TO QUEUE");
        boost::mutex::scoped_lock scoped_lock(mtx_);
        std::string nonConstStr(str);
        auto pair = std::make_pair("add", nonConstStr);
        bodyEventQueue_->push(pair);
    }
}

void GazeboInterface::onRemoveBody(const std::string& str)
{
    LOGGING("ON REMOVE BODY " + str + " " + prefix_);
    bool contains = false;
    auto models = world_->GetModels();
    for (auto & model : models) {
        if (model->GetScopedName() == str) {
            contains = true;
            break;
        }
    }
    if (world_->GetName().find("exec") != std::string::npos) {
        boost::mutex::scoped_lock scoped_lock(mtx_);
        std::string nonConstStr(str);
        auto pair = std::make_pair("remove", nonConstStr);
        bodyEventQueue_->push(pair);
    }
}

void GazeboInterface::registerOnAddBodyCallback(std::function<void (const std::string&) >& callback)
{
    addBodyCallback_ = callback;
    /**addEntityConnection_ =
        gazebo::event::Events::ConnectAddEntity(std::bind(&GazeboInterface::onAddBody,
                this,
                std::placeholders::_1));*/
}

void GazeboInterface::registerOnRemoveBodyCallback(std::function<void (const std::string&) >& callback)
{
    gazeboSubscriber_->setRemoveBodyCallback(callback);
}

void GazeboInterface::registerOnPoseChangedCallback(std::function<void (const std::string&, const geometric::Pose&) >& callback)
{
    gazeboSubscriber_->setBodyPoseChangedCallback(callback);
}

ModelPtr GazeboInterface::findRobotModel(std::string& robotName)
{
    std::vector<ModelPtr> models = world_->GetModels();
    ModelPtr model;
    for (auto & m : models) {
        model = findRobotModelImpl(m, robotName);
        if (model) {
            return model;
        }
    }

    return model;
}

ModelPtr GazeboInterface::findRobotModelImpl(const ModelPtr& model, std::string& robotName)
{
    if (model->GetName() == robotName) {
        return model;
    }

    const std::vector<ModelPtr> nestedModels = model->NestedModels();
    ModelPtr mm;
    for (auto & nestedModel : nestedModels) {
        mm = findRobotModelImpl(nestedModel, robotName);
        if (mm) {
            return mm;
        }
    }

    return mm;

}

void GazeboInterface::ignoreEnvironmentChanges() {
    if (prefix_ == "exec") {
        gazeboSubscriber_->unsubscribe();
#ifdef GZ_GT_7
        //addEntityConnection_ = nullptr;
#else
        //gazebo::event::Events::DisconnectAddEntity(addEntityConnection_);
#endif
    }
}

void GazeboInterface::reset()
{
    // Temporarily disconnect event
    if (prefix_ == "exec") {
        gazeboSubscriber_->suppressProcessing(true);
#ifdef GZ_GT_7
        //addEntityConnection_ = nullptr;
#else
        //gazebo::event::Events::DisconnectAddEntity(addEntityConnection_);
#endif
    }

    // Remove models that have been added during runtime
    for (auto model : world_->GetModels()) {
        if (model->GetName() == mainRobotName_) {
            continue;
        }
        bool found = false;
        for (auto & entry : initialWorldModelMap_) {
            if (entry.first == model->GetName()) {
                found = true;
                break;
            }
        }

        if (!found) {
            removeBody(model->GetName());
        }

    }

    // Add models that have been removed during runtime
    for (auto & entry : initialWorldModelMap_) {
        bool found = false;
        for (auto model : world_->GetModels()) {
            if (model->GetName() == mainRobotName_) {
                continue;
            }
            if (entry.first == model->GetName()) {
                found = true;
                break;
            }
        }

        if (!found) {
            addBodyFromSDFString(entry.second);
        }
    }

    // Set the initial world state
    deferredPoseChanges_.clear();
    setWorldState(initialWorldState_.get(), true, true);
    if (prefix_ == "exec" and !ignoringEnvironmentChanges_) {
        /**addEntityConnection_ =
            gazebo::event::Events::ConnectAddEntity(std::bind(&GazeboInterface::onAddBody,
                    this,
                    std::placeholders::_1));*/
        gazeboSubscriber_->suppressProcessing(false);
    }
}

void GazeboInterface::makeInitialWorldModelMap()
{
    world_->GetGazeboWorld()->Reset();
    world_->GetGazeboWorld()->ResetPhysicsStates();
    world_->GetGazeboWorld()->SetState(*(initialSDFWorldState_->getWorldState()));
    auto models = world_->GetModels();
    for (auto & model : models) {
        if (model->GetName() != mainRobotName_) {
            auto sdfElement = model->GetSDF();
            const std::string sdfString = sdfElement->ToString("");
            initialWorldModelMap_[model->GetName()] = sdfString;
        }
    }
}

void GazeboInterface::setObservationSpaceDimension(const unsigned int& dimension)
{
    observationSpaceDimension_ = dimension;
}

void GazeboInterface::makeInitialWorldState(const VectorFloat& initialStateVec, const bool& fullReset)
{
    if (!world_) {
        WARNING("world is null");
        return;
    }

    std::string worldName = world_->GetName();

    if (fullReset) {
        world_->GetGazeboWorld()->Reset();
        world_->GetGazeboWorld()->ResetPhysicsStates();
        world_->GetGazeboWorld()->ResetTime();
        world_->GetGazeboWorld()->SetState(*(initialSDFWorldState_->getWorldState()));

    }

    static_cast<gazebo::physics::OpptODEPhysics*>(world_->GetPhysicsEngine().get())->setCumulativeAngles(initialCumulativeAngles_);
    for (auto & setStateFunction : setStateFunctions_) {
        setStateFunction(initialStateVec);
    }

    initialWorldState_ = makeWorldState();
    lastConstructedWorldState_ = initialWorldState_;

}

const VectorString GazeboInterface::getJointNames() const
{
    // TODO: Return all the joint names, not just the robot joints
    VectorString jointNames;
    for (auto const & entry : worldJointMap_.at(world_->GetName())) {
        jointNames.push_back(entry.first);
    }

    return jointNames;
}

const VectorString GazeboInterface::getRobotJointNames() const
{
    VectorString jointNames;
    for (auto const & entry : worldJointMap_.at(world_->GetName())) {
        jointNames.push_back(entry.first);
    }

    return jointNames;
}

const VectorString GazeboInterface::getRobotLinkNames() const
{
    VectorString linkNames;
    for (auto const & entry : worldRobotLinkMap_.at(world_->GetName())) {
        linkNames.push_back(entry.first);
    }

    return linkNames;
}

const VectorString GazeboInterface::getLinkNames() const
{
    VectorString linkNames;
    auto models = world_->GetModels();
    for (auto & model : models) {
        auto links = model->GetLinks();
        for (auto & link : links) {
            std::string scopedName = link->GetScopedName();
            /**std::string unscopedName = scopedName;
            if (scopedName.find("::") != std::string::npos) {
                VectorString nameElems;
                split(scopedName, "::", nameElems);
                unscopedName = nameElems[1];
            }*/
            linkNames.push_back(scopedName);
        }
    }

    return linkNames;
}

GazeboWorldStatePtr GazeboInterface::getInitialWorldState(const bool &setWorldToInitialWorldState)
{
    GazeboWorldStatePtr initialWorldState = initialWorldState_;
    if (!initialWorldState)
        ERROR("Initial world state requested, but no initial world state has been constructed");
    if (setWorldToInitialWorldState)
        setWorldState(initialWorldState.get(), true, true);
    return initialWorldState;
}

void GazeboInterface::addBodyFromSDFString(std::string& sdfString)
{
    if (sdfString.find("<sdf version") == std::string::npos)
        sdfString = "<sdf version='1.6'>" + sdfString + "</sdf>";
    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readString(sdfString, sdfModel);
    if (!sdfModel) {
        oppt::ERROR("Could not parse SDF file");
    }

    sdf::ElementPtr rootElement = sdfModel->Root();
    sdf::ElementPtr modelElement = rootElement->GetElement("model");
    if (modelElement) {
        std::string modelName = modelElement->Get<std::string> ("name");

        // Remove existing body with same name
        //removeBody(modelName);
        usleep(200000);
        unsigned int numModelsBeforeInsert = world_->GetModelCount();
        world_->GetGazeboWorld()->InsertModelString(sdfString);
        initWorldLinkMap(worldLinkMap_);
        initWorldJointMap(worldJointMap_);
        fakeStep();
        bool modelInserted = false;
        std::vector<ModelPtr> currentModels;
        unsigned int currentModelCount;
        while (!modelInserted) {
            currentModelCount = world_->GetModelCount();
            auto models = world_->GetModels();
            if (currentModelCount == numModelsBeforeInsert + 1) {
                modelInserted = true;
            }

            usleep(1e5);
        }

        //world_->GetGazeboWorld()->Stop();

    } else {
        ERROR("String has no modelElement!");
    }

    initWorldLinkMap(worldLinkMap_);
    initWorldJointMap(worldJointMap_);
    static_cast<gazebo::physics::OpptODEPhysics*>(world_->GetPhysicsEngine().get())->makeJointNameMap();
    initialCumulativeAngles_ =
        static_cast<gazebo::physics::OpptODEPhysics*>(world_->GetPhysicsEngine().get())->getCumulativeAngles();
}

void GazeboInterface::fakeStep()
{
    if (prefix_.find("exec") == std::string::npos) {
        //world->ProcessMessages();
        bool physicsEnabled = world_->GetEnablePhysicsEngine();
        auto physicsEngine =
            static_cast<gazebo::physics::OpptODEPhysics*>(world_->GetPhysicsEngine().get());
        bool collisionsBlocked = physicsEngine->collionCheckBlocked();
        physicsEngine->blockCollisionCheck(true);
        //physicsEngine->enableJoints(false);
        world_->EnablePhysicsEngine(false);
        std::map<std::string, bool> implicitSpringDamperUsed;

        auto jointMap = worldJointMap_.at(world_->GetName());
        for (auto & nameJointPair : jointMap) {
            implicitSpringDamperUsed[nameJointPair.first] =
                static_cast<gazebo::physics::ODEJoint*>(nameJointPair.second.get())->UsesImplicitSpringDamper();
            static_cast<gazebo::physics::ODEJoint*>(nameJointPair.second.get())->UseImplicitSpringDamper(false);

        }

        world_->GetGazeboWorld()->Run(1);
        usleep(100000);

        for (auto & nameJointPair : jointMap) {
            static_cast<gazebo::physics::ODEJoint*>(nameJointPair.second.get())->UseImplicitSpringDamper(implicitSpringDamperUsed[nameJointPair.first]);
        }
        world_->EnablePhysicsEngine(physicsEnabled);
        //physicsEngine->enableJoints(true);
        physicsEngine->blockCollisionCheck(collisionsBlocked);
    }
}

void GazeboInterface::removeBody(std::string name)
{
    unsigned int currentModelSize = 0;
    std::vector<ModelPtr> currentModels = world_->GetModels();
    bool modelFound = false;
    for (auto & model : currentModels) {
        VectorString nameElems;
        split(name, "::", nameElems);
        if (model->GetScopedName() == nameElems[0]) {
            modelFound = true;
            currentModelSize = world_->GetModelCount();
            world_->GetGazeboWorld()->RemoveModel(model->GetName());
            initWorldLinkMap(worldLinkMap_);
            initWorldJointMap(worldJointMap_);
            fakeStep();
            break;
        }
    }

    //std::vector<ModelPtr> newModels = world->GetModels();
    if (modelFound) {
        while (currentModelSize == world_->GetModelCount()) {
            usleep(1e2);
        }
    }

    initWorldLinkMap(worldLinkMap_);
    initWorldJointMap(worldJointMap_);
    static_cast<gazebo::physics::OpptODEPhysics*>(world_->GetPhysicsEngine().get())->makeJointNameMap();
    initialCumulativeAngles_ =
        static_cast<gazebo::physics::OpptODEPhysics*>(world_->GetPhysicsEngine().get())->getCumulativeAngles();
}

void GazeboInterface::changeModelPose(const std::string& name, const geometric::Pose& pose)
{
    std::vector<ModelPtr> currentModels = world_->GetModels();
    for (auto & model : currentModels) {
        if (model->GetName() == name) {
            auto p = std::pair<const std::string, const geometric::Pose> {name, pose};
            deferredPoseChanges_.push_back(p);
            break;
        }
    }
}

void GazeboInterface::applyDeferredPoseChanges()
{
    if (deferredPoseChanges_.size() > 0) {
        std::vector<ModelPtr> currentModels = world_->GetModels();
        for (auto & poseChange : deferredPoseChanges_) {
            for (auto & model : currentModels) {
                if (model->GetName() == poseChange.first) {
                    GZVector3 newPosition(poseChange.second.position.x(), poseChange.second.position.y(), poseChange.second.position.z());
                    GZQuaternion newOrientation(poseChange.second.orientation.w(),
                                                poseChange.second.orientation.x(),
                                                poseChange.second.orientation.y(),
                                                poseChange.second.orientation.z());
                    GZPose newPose(newPosition, newOrientation);
                    model->SetWorldPose(newPose);
                    break;
                }
            }
        }

        deferredPoseChanges_.clear();
    }
}

void GazeboInterface::setSoftLimitThreshold(const FloatType& softLimitThreshold)
{
    auto jointMap = worldJointMap_.at(world_->GetName());
    for (auto & nameJointPair : jointMap) {
#ifdef GZ_GT_7
        FloatType lowerSoftLimitPosition =
            nameJointPair.second->LowerLimit(0) + std::fabs(nameJointPair.second->LowerLimit(0)) * softLimitThreshold;

        FloatType upperSoftLimitPosition =
            nameJointPair.second->UpperLimit(0) - std::fabs(nameJointPair.second->UpperLimit(0)) * softLimitThreshold;
#else
        FloatType lowerSoftLimitPosition =
            nameJointPair.second->GetLowerLimit(0).Radian() + std::fabs(nameJointPair.second->GetLowerLimit(0).Radian()) * softLimitThreshold;

        FloatType upperSoftLimitPosition =
            nameJointPair.second->GetUpperLimit(0).Radian() - std::fabs(nameJointPair.second->GetUpperLimit(0).Radian()) * softLimitThreshold;
#endif
        FloatType softLimitVelocity =
            nameJointPair.second->GetVelocityLimit(0) - std::fabs(nameJointPair.second->GetVelocityLimit(0)) * softLimitThreshold;

        nameJointPair.second->SetLowerLimit(0, lowerSoftLimitPosition);
        nameJointPair.second->SetUpperLimit(0, upperSoftLimitPosition);
        nameJointPair.second->SetVelocityLimit(0, softLimitVelocity);
    }
}

void GazeboInterface::getStateLimits(VectorFloat& lowerStateLimits, VectorFloat& upperStateLimits)
{
    lowerStateLimits.clear();
    upperStateLimits.clear();
    auto jointMap = worldJointMap_.at(world_->GetName());
    const SpaceVariables *orderedVariables = stateSpaceInformation_->orderedVariables.get();
    for (auto &spaceVariable : orderedVariables->spaceVariables) {
        switch (spaceVariable) {
        case SpaceVariable::JOINT_POSITIONS:
            for (auto & jointName : stateSpaceInformation_->jointPositions) {
                for (auto & nameJointPair : jointMap) {
                    if (nameJointPair.first.find(jointName) != std::string::npos) {
#ifdef GZ_GT_7
                        lowerStateLimits.push_back(nameJointPair.second->LowerLimit(0));
                        upperStateLimits.push_back(nameJointPair.second->UpperLimit(0));
#else
                        lowerStateLimits.push_back(nameJointPair.second->GetLowerLimit(0).Radian());
                        upperStateLimits.push_back(nameJointPair.second->GetUpperLimit(0).Radian());
#endif

                    }
                }
            }
            break;
        case SpaceVariable::JOINT_VELOCITIES:
            for (auto & jointName : stateSpaceInformation_->jointVelocities) {
                for (auto & nameJointPair : jointMap) {
                    if (nameJointPair.first.find(jointName) != std::string::npos) {
                        lowerStateLimits.push_back(-nameJointPair.second->GetVelocityLimit(0));
                        upperStateLimits.push_back(nameJointPair.second->GetVelocityLimit(0));
                    }
                }
            }
            break;
        case SpaceVariable::LINK_POSES:
            for (size_t i = 0; i < stateSpaceInformation_->containedLinkPoses.size(); i++) {
                for (size_t j = 0; j < 6; j++) {
                    lowerStateLimits.push_back(stateSpaceInformation_->containedLinkPosesLowerLimits[i][j]);
                    upperStateLimits.push_back(stateSpaceInformation_->containedLinkPosesUpperLimits[i][j]);
                }
            }
            break;
        case SpaceVariable::LINK_POSITIONS_X:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkPositionsX.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsXLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsXLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_POSITIONS_Y:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkPositionsY.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsYLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsYLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_POSITIONS_Z:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkPositionsZ.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsZLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsZLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_ORIENTATIONS_X:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkOrientationsX.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsXLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsXLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_ORIENTATIONS_Y:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkOrientationsY.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsYLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsYLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_ORIENTATIONS_Z:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkOrientationsZ.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsZLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsZLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_LINEAR:
            for (size_t i = 0; i < stateSpaceInformation_->containedLinkVelocitiesLinear.size(); i++) {
                for (size_t j = 0; j != 3; j++) {
                    lowerStateLimits.push_back(stateSpaceInformation_->containedLinkVelocitiesLinearLimits[i][0]);
                    upperStateLimits.push_back(stateSpaceInformation_->containedLinkVelocitiesLinearLimits[i][1]);
                }
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_ANGULAR:
            for (size_t i = 0; i < stateSpaceInformation_->containedLinkVelocitiesAngular.size(); i++) {
                for (size_t j = 0; j < 3; j++) {
                    lowerStateLimits.push_back(stateSpaceInformation_->containedLinkVelocitiesAngularLimits[i][0]);
                    upperStateLimits.push_back(stateSpaceInformation_->containedLinkVelocitiesAngularLimits[i][1]);
                }
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_LINEAR_X:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkLinearVelocitiesX.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesXLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesXLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_LINEAR_Y:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkLinearVelocitiesY.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesYLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesYLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_LINEAR_Z:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkLinearVelocitiesZ.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesZLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesZLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_ANGULAR_X:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkAngularVelocitiesX.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkAngularVelocitiesXLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkAngularVelocitiesXLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_ANGULAR_Y:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkAngularVelocitiesY.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkAngularVelocitiesYLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkAngularVelocitiesYLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_ANGULAR_Z:
            for (size_t i = 0; i != stateSpaceInformation_->containedLinkAngularVelocitiesZ.size(); ++i) {
                lowerStateLimits.push_back(stateSpaceInformation_->containedLinkAngularVelocitiesZLimits[i][0]);
                upperStateLimits.push_back(stateSpaceInformation_->containedLinkAngularVelocitiesZLimits[i][1]);
            }
            break;
        default:
            ERROR("Huh");
            break;
        }
    }
}

void GazeboInterface::getObservationLimits(VectorFloat& lowerObservationLimits, VectorFloat& upperObservationLimits, const bool &ignoreObservationLimits)
{
    lowerObservationLimits.clear();
    upperObservationLimits.clear();
    auto jointMap = worldJointMap_.at(world_->GetName());
    const SpaceVariables *orderedVariables = observationSpaceInformation_->orderedVariables.get();
    for (auto &spaceVariable : orderedVariables->spaceVariables) {
        switch (spaceVariable) {
        case SpaceVariable::JOINT_POSITIONS:
            for (auto & jointName : observationSpaceInformation_->jointPositions) {
                for (auto & nameJointPair : jointMap) {
                    if (nameJointPair.first.find(jointName) != std::string::npos) {
#ifdef GZ_GT_7
                        lowerObservationLimits.push_back(nameJointPair.second->LowerLimit(0));
                        upperObservationLimits.push_back(nameJointPair.second->UpperLimit(0));
#else
                        lowerObservationLimits.push_back(nameJointPair.second->GetLowerLimit(0).Radian());
                        upperObservationLimits.push_back(nameJointPair.second->GetUpperLimit(0).Radian());
#endif
                    }
                }
            }
            break;
        case SpaceVariable::JOINT_VELOCITIES:
            for (auto & jointName : observationSpaceInformation_->jointVelocities) {
                for (auto & nameJointPair : jointMap) {
                    if (nameJointPair.first.find(jointName) != std::string::npos) {
                        lowerObservationLimits.push_back(-nameJointPair.second->GetVelocityLimit(0));
                        upperObservationLimits.push_back(nameJointPair.second->GetVelocityLimit(0));
                    }
                }
            }
            break;
        case SpaceVariable::LINK_POSES:
            for (size_t i = 0; i < observationSpaceInformation_->containedLinkPoses.size(); i++) {
                for (size_t j = 0; j < 6; j++) {
                    lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkPosesLowerLimits[i][j]);
                    upperObservationLimits.push_back(observationSpaceInformation_->containedLinkPosesUpperLimits[i][j]);
                }
            }
            break;
        case SpaceVariable::LINK_POSITIONS_X:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkPositionsX.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsXLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsXLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_POSITIONS_Y:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkPositionsY.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsYLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsYLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_POSITIONS_Z:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkPositionsZ.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsZLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsZLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_ORIENTATIONS_X:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkOrientationsX.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsXLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsXLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_ORIENTATIONS_Y:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkOrientationsY.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsYLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsYLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_ORIENTATIONS_Z:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkOrientationsZ.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsZLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsZLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_LINEAR:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkVelocitiesLinear.size(); ++i) {
                for (size_t j = 0; j != 3; ++j) {
                    //observationSpaceInformation_->containedLinkVelocitiesLinearLimits[i][j]
                    lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkVelocitiesLinearLimits[i][0]);
                    upperObservationLimits.push_back(observationSpaceInformation_->containedLinkVelocitiesLinearLimits[i][1]);
                }
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_ANGULAR:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkVelocitiesAngular.size(); ++i) {
                for (size_t j = 0; j != 3; ++j) {
                    lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkVelocitiesAngularLimits[i][0]);
                    upperObservationLimits.push_back(observationSpaceInformation_->containedLinkVelocitiesAngularLimits[i][1]);
                }
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_LINEAR_X:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkLinearVelocitiesX.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesXLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesXLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_LINEAR_Y:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkLinearVelocitiesY.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesYLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesYLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_LINEAR_Z:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkLinearVelocitiesZ.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesZLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesZLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_ANGULAR_X:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkAngularVelocitiesX.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkAngularVelocitiesXLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkAngularVelocitiesXLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_ANGULAR_Y:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkAngularVelocitiesY.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkAngularVelocitiesYLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkAngularVelocitiesYLimits[i][1]);
            }
            break;
        case SpaceVariable::LINK_VELOCITIES_ANGULAR_Z:
            for (size_t i = 0; i != observationSpaceInformation_->containedLinkAngularVelocitiesZ.size(); ++i) {
                lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkAngularVelocitiesZLimits[i][0]);
                upperObservationLimits.push_back(observationSpaceInformation_->containedLinkAngularVelocitiesZLimits[i][1]);
            }
            break;
        default:
            ERROR("Observation variable not recognized");
            break;
        }
    }

    if (!ignoreObservationLimits) {
        VectorFloat lowerLimitsSensors, upperLimitsSensors;
        sensorInterface_->getObservationLimits(lowerLimitsSensors, upperLimitsSensors);
        lowerObservationLimits.insert(lowerObservationLimits.end(), lowerLimitsSensors.begin(), lowerLimitsSensors.end());
        upperObservationLimits.insert(upperObservationLimits.end(), upperLimitsSensors.begin(), upperLimitsSensors.end());
    }
}

void GazeboInterface::getActionLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits)
{
    lowerLimits.clear();
    upperLimits.clear();
    gazebo::physics::JointPtr joint;
    const SpaceVariables *orderedVariables = actionSpaceInformation_->orderedVariables.get();
    for (auto &spaceVariable : orderedVariables->spaceVariables) {
        switch (spaceVariable) {
        case SpaceVariable::JOINT_TORQUES:
            for (auto & torqueControlledJoint : actionSpaceInformation_->torqueControlledJoints) {
                joint = worldJointMap_[world_->GetName()][torqueControlledJoint];
                lowerLimits.push_back(-joint->GetEffortLimit(0));
                upperLimits.push_back(joint->GetEffortLimit(0));
            }
            break;
        case SpaceVariable::JOINT_VELOCITIES:
            for (auto & velocityControlledJoint : actionSpaceInformation_->velocityControlledJoints) {
                joint = worldJointMap_[world_->GetName()][velocityControlledJoint];
                lowerLimits.push_back(-joint->GetVelocityLimit(0));
                upperLimits.push_back(joint->GetVelocityLimit(0));
            }
            break;
        case SpaceVariable::JOINT_POSITIONS:
            for (auto & positionControlledJoint : actionSpaceInformation_->positionControlledJoints) {
                joint = worldJointMap_[world_->GetName()][positionControlledJoint];
#ifdef GZ_GT_7
                lowerLimits.push_back(joint->LowerLimit(0));
                upperLimits.push_back(joint->UpperLimit(0));
#else
                lowerLimits.push_back(joint->GetLowerLimit(0).Radian());
                upperLimits.push_back(joint->GetUpperLimit(0).Radian());
#endif
            }
            break;
        case SpaceVariable::JOINT_POSITIONS_INCREMENT:
            break;
        default:
            ERROR("Action variable not recognized");
            break;
        }
    }
}

void GazeboInterface::getRobotBoundingBoxesDirty(BoundingBoxes* boundingBoxes)
{
    if (cachedCollisionLinks_.size() == 0) {
        LinkMap lm = worldRobotLinkMap_[world_->GetName()];
        for (size_t i = 0; i < boundingBoxes->linkNames.size(); i++) {
            LinkPtr linkPtr = lm[boundingBoxes->linkNames[i]];
            cachedCollisionLinks_.push_back(linkPtr);
        }
    }

    GZBox collisionBoundingBox;
    GZVector3 center;
    FloatType lenX, lenY, lenZ;
    if (boundingBoxes->linkNames.size() == 0) {
        return;
    }

    boundingBoxes->boxes = std::vector<VectorFloat> (boundingBoxes->linkNames.size());
    boundingBoxes->worldPoses = std::vector<geometric::Pose> (boundingBoxes->linkNames.size());
    unsigned int idx = 0;
    for (size_t i = 0; i != cachedCollisionLinks_.size(); i++) {
        for (auto & collision : cachedCollisionLinks_[i]->GetCollisions()) {
            collision->Update();
        }

        //VectorFloat linkBoundingBox(6);
        geometric::Pose linkWorldPoseVec;
#ifdef GZ_GT_7
        collisionBoundingBox = cachedCollisionLinks_[i]->CollisionBoundingBox();
        center = collisionBoundingBox.Center();
        lenX = collisionBoundingBox.XLength();
        lenY = collisionBoundingBox.YLength();
        lenZ = collisionBoundingBox.ZLength();
#else
        collisionBoundingBox = cachedCollisionLinks_[i]->GetCollisionBoundingBox();
        center = collisionBoundingBox.GetCenter();
        lenX = collisionBoundingBox.GetXLength();
        lenY = collisionBoundingBox.GetYLength();
        lenZ = collisionBoundingBox.GetZLength();
#endif

        if (lenX > 0 || lenY > 0 || lenZ > 0) {
            // Filter out dimensionless links
#ifdef GZ_GT_7
            GZPose linkCollisionWorldPose =
                cachedCollisionLinks_[i]->GetCollisions() [0]->InitialRelativePose() * cachedCollisionLinks_[i]->WorldPose();
            linkWorldPoseVec.position[0] = linkCollisionWorldPose.Pos().X();
            linkWorldPoseVec.position[0] = linkCollisionWorldPose.Pos().Y();
            linkWorldPoseVec.position[0] = linkCollisionWorldPose.Pos().Z();
            linkWorldPoseVec.orientation.w() = linkCollisionWorldPose.Rot().W();
            linkWorldPoseVec.orientation.x() = linkCollisionWorldPose.Rot().X();
            linkWorldPoseVec.orientation.y() = linkCollisionWorldPose.Rot().Y();
            linkWorldPoseVec.orientation.z() = linkCollisionWorldPose.Rot().Z();
#else
            GZPose linkCollisionWorldPose =
                cachedCollisionLinks_[i]->GetCollisions() [0]->GetInitialRelativePose() * cachedCollisionLinks_[i]->GetWorldPose();
            linkWorldPoseVec.position[0] = linkCollisionWorldPose.pos.x;
            linkWorldPoseVec.position[1] = linkCollisionWorldPose.pos.y;
            linkWorldPoseVec.position[2] = linkCollisionWorldPose.pos.z;
            linkWorldPoseVec.orientation.w() = linkCollisionWorldPose.rot.w;
            linkWorldPoseVec.orientation.x() = linkCollisionWorldPose.rot.x;
            linkWorldPoseVec.orientation.y() = linkCollisionWorldPose.rot.y;
            linkWorldPoseVec.orientation.z() = linkCollisionWorldPose.rot.z;

#endif


            boundingBoxes->worldPoses[i] = linkWorldPoseVec;
            idx++;
        } else {
            auto s = getState_();
            printVector(s, "s");
            LOGGING("dim 0 " + cachedCollisionLinks_[i]->GetName());
            getchar();
        }
    }

    boundingBoxes->linkNames.resize(idx);
    boundingBoxes->worldPoses.resize(idx);
    boundingBoxes->boxes.resize(idx);
}

void GazeboInterface::getRobotBoundingBoxes(const VectorFloat& stateVec, GazeboWorldStatePtr& worldState, BoundingBoxes* boundingBoxes)
{
    std::string worldName = world_->GetName();
    if (worldState) {
        if (* (lastConstructedWorldState_.get()) != * (worldState.get()))
            setWorldState(worldState.get(), false, false);
    }

    setStateManuallyNew(stateVec);
    getRobotBoundingBoxesDirty(boundingBoxes);
    auto vecAfterSet = getState_();
}

std::pair<VectorString, std::vector<geometric::Pose>> GazeboInterface::getLinkPosesDirty() const {
    auto linkMap = worldLinkMap_.at(world_->GetName());
    VectorString linkNames(linkMap.size(), "");
    std::vector<geometric::Pose> linkPoses;
    size_t c = 0;
    for (auto &worldLinkMapEntry : linkMap) {
        linkNames[c] = worldLinkMapEntry.first;
#ifdef GZ_GT_7
        linkPoses.push_back(geometric::Pose(worldLinkMapEntry.second->WorldPose()));
#else
        linkPoses.push_back(geometric::Pose(worldLinkMapEntry.second->GetWorldPose()));
#endif
        c++;
    }

    return std::make_pair(linkNames, linkPoses);
}

VectorFloat GazeboInterface::getLinkPosition(const VectorFloat& robotState,
        const GazeboWorldStatePtr& worldState,
        const std::string& linkName)
{
    std::string worldName = world_->GetName();
    if (worldState) {
        if (* (lastConstructedWorldState_.get()) != * (worldState.get()))
            setWorldState(worldState.get(), false, false);
    }

    setStateManuallyNew(robotState);
#ifdef GZ_GT_7
    const GZPose linkPose =
        worldRobotLinkMap_[worldName][linkName]->WorldPose();
#else
    const GZPose linkPose =
        worldRobotLinkMap_[worldName][linkName]->GetWorldPose();
#endif

    VectorFloat position(3);
#ifdef GZ_GT_7
    position[0] = linkPose.Pos().X();
    position[1] = linkPose.Pos().Y();
    position[2] = linkPose.Pos().Z();
#else
    position[0] = linkPose.pos[0];
    position[1] = linkPose.pos[1];
    position[2] = linkPose.pos[2];
#endif
    return position;
}

geometric::Pose GazeboInterface::getLinkPose(const VectorFloat& robotState,
        const GazeboWorldStatePtr& worldState,
        const std::string& linkName)
{
    std::string worldName = world_->GetName();
    if (worldState) {
        if (* (lastConstructedWorldState_.get()) != * (worldState.get()))
            setWorldState(worldState.get(), false, false);
    }

    setStateManuallyNew(robotState);
    geometric::Pose pose;
#ifdef GZ_GT_7
    const GZPose linkPose =
        worldRobotLinkMap_[worldName][linkName]->WorldPose();
    pose.position[0] = linkPose.Pos().X();
    pose.position[1] = linkPose.Pos().Y();
    pose.position[2] = linkPose.Pos().Z();
    pose.orientation.w() = linkPose.Rot().W();
    pose.orientation.x() = linkPose.Rot().X();
    pose.orientation.y() = linkPose.Rot().Y();
    pose.orientation.z() = linkPose.Rot().Z();
#else
    const GZPose linkPose =
        worldRobotLinkMap_[worldName][linkName]->GetWorldPose();
    pose.position[0] = linkPose.pos.x;
    pose.position[1] = linkPose.pos.y;
    pose.position[2] = linkPose.pos.z;
    pose.orientation.w() = linkPose.rot.w;
    pose.orientation.x() = linkPose.rot.x;
    pose.orientation.y() = linkPose.rot.y;
    pose.orientation.z() = linkPose.rot.z;
#endif
    return pose;
}

std::vector<geometric::Pose> GazeboInterface::getLinksCOGPose(const VectorFloat& robotState,
        const GazeboWorldStatePtr& worldState,
        const VectorString& linkNames)
{
    std::string worldName = world_->GetName();
    if (worldState) {
        if (* (lastConstructedWorldState_.get()) != * (worldState.get()))
            setWorldState(worldState.get(), false, false);
    }

    setStateManuallyNew(robotState);
    return getLinksCOGPoseDirty(linkNames);
}

std::vector<geometric::Pose> GazeboInterface::getLinksCOGPoseDirty(const VectorString& linkNames)
{
    std::vector<geometric::Pose> poses(linkNames.size());
    for (size_t i = 0; i != linkNames.size(); ++i) {
#ifdef GZ_GT_7
        const GZPose linkPose =
            worldRobotLinkMap_.at(world_->GetName()).at(linkNames[i])->WorldCoGPose();

        poses[i].position[0] = linkPose.Pos().X();
        poses[i].position[1] = linkPose.Pos().Y();
        poses[i].position[2] = linkPose.Pos().Z();
        poses[i].orientation.w() = linkPose.Rot().W();
        poses[i].orientation.x() = linkPose.Rot().X();
        poses[i].orientation.y() = linkPose.Rot().Y();
        poses[i].orientation.z() = linkPose.Rot().Z();
#else
        const GZPose linkPose =
            worldRobotLinkMap_.at(world_->GetName()).at(linkNames[i])->GetWorldCoGPose();

        poses[i].position[0] = linkPose.pos.x;
        poses[i].position[1] = linkPose.pos.y;
        poses[i].position[2] = linkPose.pos.z;
        poses[i].orientation.w() = linkPose.rot.w;
        poses[i].orientation.x() = linkPose.rot.x;
        poses[i].orientation.y() = linkPose.rot.y;
        poses[i].orientation.z() = linkPose.rot.z;
#endif
    }

    return poses;
}

std::vector<geometric::Pose> GazeboInterface::getLinksCollisionWorldPoses(const RobotStateSharedPtr& robotState,
        const VectorString& linkNames)
{
    if (!robotState)
        ERROR("Robot state is nullptr");
    std::string worldName = world_->GetName();
    auto worldState = robotState->getGazeboWorldState();
    if (!worldState) {
        ERROR("State has no GazeboWorldState");
    }

    if ((lastConstructedWorldState_.get()) != (worldState.get())) {
        setWorldState(worldState.get(), false, false);
    }

    setStateManuallyNew(robotState->as<VectorState>()->asVector());
    return getLinksCollisionWorldPosesDirty(linkNames);

}

std::vector<geometric::Pose> GazeboInterface::getLinksCollisionWorldPosesDirty(const VectorString& linkNames)
{
    std::vector<geometric::Pose> poses;
    for (size_t i = 0; i != linkNames.size(); ++i) {
        auto link = worldRobotLinkMap_.at(world_->GetName()).at(linkNames[i]);
#ifdef GZ_GT_7
        auto linkPose = link->WorldPose();
#else
        auto linkPose = link->GetWorldPose();
#endif

        auto collisions = link->GetCollisions();
        for (auto & collision : collisions) {
            geometric::Pose pose;
#ifdef GZ_GT_7
            auto collisionPose = collision->InitialRelativePose() + linkPose;
            pose.position[0] = collisionPose.Pos().X();
            pose.position[1] = collisionPose.Pos().Y();
            pose.position[2] = collisionPose.Pos().Z();
            pose.orientation.w() = collisionPose.Rot().W();
            pose.orientation.x() = collisionPose.Rot().X();
            pose.orientation.y() = collisionPose.Rot().Y();
            pose.orientation.z() = collisionPose.Rot().Z();
#else
            auto collisionPose = collision->GetInitialRelativePose() + linkPose;
            pose.position[0] = collisionPose.pos.x;
            pose.position[1] = collisionPose.pos.y;
            pose.position[2] = collisionPose.pos.z;
            pose.orientation.w() = collisionPose.rot.w;
            pose.orientation.x() = collisionPose.rot.x;
            pose.orientation.y() = collisionPose.rot.y;
            pose.orientation.z() = collisionPose.rot.z;
#endif

            poses.push_back(pose);
        }
    }

    return poses;
}

std::string GazeboInterface::getRobotName() const
{
    return mainRobotName_;
}

void GazeboInterface::getRobotVisualPoses(VectorFloat& state,
        const VectorString& visualNames,
        std::vector<geometric::Pose>& visualPoses,
        const GazeboWorldStatePtr &worldState)
{
    std::string worldName = world_->GetName();
    if (worldState)
        setWorldState(worldState.get(), false, false);
    setStateManuallyNew(state);
    visualPoses = std::vector<geometric::Pose> (visualNames.size());
    for (size_t i = 0; i != visualNames.size(); ++i) {
        uint32_t visualId;
        for (auto & linkEntry : worldLinkMap_[worldName]) {
            if (linkEntry.second->VisualId(visualNames[i], visualId)) {
                ignition::math::Pose3d visualPose;
                linkEntry.second->VisualPose(visualId, visualPose);
#ifdef GZ_GT_7
                GZPose linkPose = linkEntry.second->WorldPose();
                ignition::math::Pose3d linkPoseIgn(linkPose.Pos().X(),
                                                   linkPose.Pos().Y(),
                                                   linkPose.Pos().Z(),
                                                   linkPose.Rot().W(),
                                                   linkPose.Rot().X(),
                                                   linkPose.Rot().Y(),
                                                   linkPose.Rot().Z());
#else
                GZPose linkPose = linkEntry.second->GetWorldPose();
                ignition::math::Pose3d linkPoseIgn(linkPose.pos.x,
                                                   linkPose.pos.y,
                                                   linkPose.pos.z,
                                                   linkPose.rot.w,
                                                   linkPose.rot.x,
                                                   linkPose.rot.y,
                                                   linkPose.rot.z);
#endif

                ignition::math::Pose3d resPose = visualPose * linkPoseIgn;
                geometric::Pose pose;
                pose.position = Vector3f(resPose.Pos().X(), resPose.Pos().Y(), resPose.Pos().Z());
                pose.orientation.w() = resPose.Rot().W();
                pose.orientation.x() = resPose.Rot().X();
                pose.orientation.y() = resPose.Rot().Y();
                pose.orientation.z() = resPose.Rot().Z();
                visualPoses[i] = pose;
                break;
            }
        }
    }
}

VectorFloat GazeboInterface::getState_()
{
    VectorFloat stateVec(stateSpaceDimension_);
    for (auto & getStateFunction : getStateFunctions_) {
        getStateFunction(stateVec);
    }

    return stateVec;
}

std::unordered_map<std::string, VectorString> GazeboInterface::generateRedundancyMap(const VectorString& joints,
        const std::vector<VectorString>& redundantJoints) const
{
    std::unordered_map<std::string, VectorString> redundancyMap;
    for (size_t i = 0; i < joints.size(); i++) {
        for (size_t j = 0; j < redundantJoints.size(); j++) {
            if (contains(redundantJoints[j], joints[i])) {
                VectorString red;
                red.assign(redundantJoints[j].begin() + 1, redundantJoints[j].end());
                redundancyMap[joints[i]] = red;
            }
        }
    }
    return redundancyMap;
}

void GazeboInterface::setActionSpaceInformation(const ActionSpaceInformationPtr& actionSpaceInformation)
{
    actionSpaceInformation_ = actionSpaceInformation;
    requiresPhysics_ = actionSpaceInformation_->requiresPhysics;
    VectorString torqueControlledJoints = actionSpaceInformation_->torqueControlledJoints;
    VectorString velocityControlledJoints = actionSpaceInformation_->velocityControlledJoints;
    VectorString positionControlledJoints = actionSpaceInformation_->positionControlledJoints;
    VectorString positionIncrementControlledJoints = actionSpaceInformation_->positionIncrementControlledJoints;
    std::vector<VectorString> redundantJoints = actionSpaceInformation_->redundantJoints;
    unsigned int startIndex = 0;
    std::unordered_map<std::string, VectorString> redundancyMapTorque = generateRedundancyMap(torqueControlledJoints, redundantJoints);
    std::unordered_map<std::string, VectorString> redundancyMapVelocity = generateRedundancyMap(velocityControlledJoints, redundantJoints);
    std::unordered_map<std::string, VectorString> redundancyMapPosition = generateRedundancyMap(positionControlledJoints, redundantJoints);
    std::unordered_map<std::string, VectorString> redundancyMapPositionIncrement = generateRedundancyMap(positionIncrementControlledJoints, redundantJoints);
    const SpaceVariables *orderedVariables = actionSpaceInformation_->orderedVariables.get();

    std::vector<JointPtr> actionTorqueJoints;
    std::vector<std::vector<JointPtr>> redundantActionTorqueJoints;
    makeJointVectorHelper_(torqueControlledJoints, redundancyMapTorque, actionTorqueJoints, redundantActionTorqueJoints);

    std::vector<JointPtr> actionVelocityJoints;
    std::vector<std::vector<JointPtr>> redundantActionVelocityJoints;
    makeJointVectorHelper_(velocityControlledJoints, redundancyMapVelocity, actionVelocityJoints, redundantActionVelocityJoints);

    std::vector<JointPtr> actionPositionJoints;
    std::vector<std::vector<JointPtr>> redundantActionPositionJoints;
    makeJointVectorHelper_(positionControlledJoints, redundancyMapPosition, actionPositionJoints, redundantActionPositionJoints);

    std::vector<JointPtr> actionPositionIncrementJoints;
    std::vector<std::vector<JointPtr>> redundantActionPositionIncrementJoints;
    makeJointVectorHelper_(positionIncrementControlledJoints,
                           redundancyMapPositionIncrement,
                           actionPositionIncrementJoints,
                           redundantActionPositionIncrementJoints);

    for (auto &spaceVariable : orderedVariables->spaceVariables) {
        switch (spaceVariable) {
        case SpaceVariable::JOINT_TORQUES:
            if (torqueControlledJoints.size() > 0) {
                ApplyActionFunction applyTorque = [this,
                                                   actionTorqueJoints,
                                                   redundantActionTorqueJoints,
                startIndex](const VectorFloat & actionVec) {
                    FloatType jointVel;
                    FloatType jointVelLimit;
                    FloatType appliedTorque;
                    for (size_t i = 0; i != actionTorqueJoints.size(); ++i) {
                        appliedTorque = actionVec[startIndex + i];
                        jointVel = actionTorqueJoints[i]->GetVelocity(0);
                        jointVelLimit = actionTorqueJoints[i]->GetVelocityLimit(0);

                        // Set the applied torque to zero if the joint velocity limit has been reached
                        // and the torque is applied in the same direction
                        if (jointVel > jointVelLimit - std::numeric_limits<FloatType>::epsilon()) {
                            appliedTorque = appliedTorque > 0 ? 0 : appliedTorque;
                        } else if (jointVel < -jointVelLimit + std::numeric_limits<FloatType>::epsilon()) {
                            appliedTorque = appliedTorque < 0 ? 0 : appliedTorque;
                        }

                        actionTorqueJoints[i]->SetForce(0, appliedTorque);
                        for (size_t j = 0; j != redundantActionTorqueJoints[i].size(); ++j) {
                            redundantActionTorqueJoints[i][j]->SetForce(0, appliedTorque);
                        }
                    }
                };

                applyActionFunctions_.push_back(applyTorque);
                startIndex += torqueControlledJoints.size();
            }
            break;
        case SpaceVariable::JOINT_VELOCITIES:
            if (velocityControlledJoints.size() > 0) {
                ApplyActionFunction applyVelocity = [this,
                                                     actionVelocityJoints,
                                                     redundantActionVelocityJoints,
                startIndex](const VectorFloat & actionVec) {
                    VectorString redJoints;
                    std::string redundantJoint;
                    for (size_t i = 0; i != actionVelocityJoints.size(); ++i) {
                        setJointVelocityRecursive(actionVelocityJoints[i].get(), actionVec[startIndex + i]);
                        for (size_t j = 0; j != redundantActionVelocityJoints[i].size(); ++j) {
                            setJointVelocityRecursive(redundantActionVelocityJoints[i][j].get(), actionVec[startIndex + i]);
                        }
                    }
                };

                applyActionFunctions_.push_back(applyVelocity);
                startIndex += velocityControlledJoints.size();
            }
            break;
        case SpaceVariable::JOINT_POSITIONS:
            if (positionControlledJoints.size() > 0) {
                ApplyActionFunction setPosition = [this,
                                                   actionPositionJoints,
                                                   redundantActionPositionJoints,
                startIndex](const VectorFloat & actionVec) {
                    VectorString redJoints;
                    std::string redundantJoint;
                    size_t i = 0;
                    for (size_t i = 0; i != actionPositionJoints.size(); ++i) {
                        actionPositionJoints[i]->SetPosition(0, actionVec[startIndex + i]);
                        for (size_t j = 0; j != redundantActionPositionJoints[i].size(); ++j) {
                            redundantActionPositionJoints[i][j]->SetPosition(0, actionVec[startIndex + i]);
                        }
                    }
                };

                applyActionFunctions_.push_back(setPosition);
                startIndex += positionControlledJoints.size();
            }
            break;
        case SpaceVariable::JOINT_POSITIONS_INCREMENT:
            if (positionIncrementControlledJoints.size() > 0) {
                ApplyActionFunction setPositionIncrement = [this,
                                    actionPositionIncrementJoints,
                                    redundantActionPositionIncrementJoints,
                startIndex](const VectorFloat & actionVec) {
                    VectorString redJoints;
                    std::string redundantJoint;
                    FloatType currentPos = 0.0;
                    FloatType increment = 0.0;
                    FloatType newPos = 0.0;
                    for (size_t i = 0; i != actionPositionIncrementJoints.size(); ++i) {
#ifdef GZ_GT_7
                        currentPos = actionPositionIncrementJoints[i]->Position(0);
#else
                        currentPos = actionPositionIncrementJoints[i]->GetAngle(0).Radian();
#endif
                        increment = actionVec[i + startIndex];
                        newPos = currentPos + increment;
                        // Cap at joint limit
                        if (increment < 0.0) {
#ifdef GZ_GT_7
                            if (newPos <= actionPositionIncrementJoints[i]->LowerLimit(0))
                                newPos = actionPositionIncrementJoints[i]->LowerLimit(0);
#else
                            if (newPos <= actionPositionIncrementJoints[i]->GetLowerLimit(0).Radian())
                                newPos = actionPositionIncrementJoints[i]->GetLowerLimit(0).Radian();
#endif
                        } else {
#ifdef GZ_GT_7
                            if (newPos >= actionPositionIncrementJoints[i]->UpperLimit(0))
                                newPos = actionPositionIncrementJoints[i]->UpperLimit(0);
#else
                            if (newPos >= actionPositionIncrementJoints[i]->GetUpperLimit(0).Radian())
                                newPos = actionPositionIncrementJoints[i]->GetUpperLimit(0).Radian();
#endif
                        }

                        actionPositionIncrementJoints[i]->SetPosition(0, newPos);
                        for (size_t j = 0; j != redundantActionPositionIncrementJoints[i].size(); ++j) {
#ifdef GZ_GT_7
                            currentPos = redundantActionPositionIncrementJoints[i][j]->Position(0);
#else
                            currentPos = redundantActionPositionIncrementJoints[i][j]->GetAngle(0).Radian();
#endif
                            increment = actionVec[i + startIndex];
                            newPos = currentPos + increment;
                            if (increment < 0.0) {
#ifdef GZ_GT_7
                                if (newPos <= actionPositionIncrementJoints[i]->LowerLimit(0))
                                    newPos = actionPositionIncrementJoints[i]->LowerLimit(0);
#else
                                if (newPos <= actionPositionIncrementJoints[i]->GetLowerLimit(0).Radian())
                                    newPos = actionPositionIncrementJoints[i]->GetLowerLimit(0).Radian();
#endif
                            } else {
#ifdef GZ_GT_7
                                if (newPos >= actionPositionIncrementJoints[i]->UpperLimit(0))
                                    newPos = actionPositionIncrementJoints[i]->UpperLimit(0);
#else
                                if (newPos >= actionPositionIncrementJoints[i]->GetUpperLimit(0).Radian())
                                    newPos = actionPositionIncrementJoints[i]->GetUpperLimit(0).Radian();
#endif
                            }

                            redundantActionPositionIncrementJoints[i][j]->SetPosition(0,
                                    newPos);
                        }
                    }
                };

                applyActionFunctions_.push_back(setPositionIncrement);
                startIndex += positionIncrementControlledJoints.size();
            }
            break;
        default:
            ERROR("Action variable not recognized");
            break;
        }
    }
}

void GazeboInterface::setJointVelocityRecursive(gazebo::physics::Joint *joint, const FloatType & velocity) const
{
    joint->SetVelocity(0, velocity);
    gazebo::physics::LinkPtr childLink = joint->GetChild();
    for (auto & childJoint : childLink->GetChildJoints()) {
        FloatType childJointVelocity = childJoint->GetVelocity(0);
        setJointVelocityRecursive(childJoint.get(), childJointVelocity);
    }

}

void GazeboInterface::setLinkPoseRecursive(LinkPtr & link, GZPose & pose, GZPose & childLinkPose)
{
#ifdef GZ_GT_7
    auto p = (link->WorldPose() - childLinkPose) + pose;
#else
    auto p = (link->GetWorldPose() - childLinkPose) + pose;
#endif
    link->SetWorldPose(p);
    for (auto & childLink : link->GetChildJointsLinks()) {
        setLinkPoseRecursive(childLink, pose, childLinkPose);
    }
}

void GazeboInterface::snapVelocityRecursive(const bool & verbose)
{
    for (auto & jointEntry : rootJoints_) {
        snapVelocityRecursiveHelper(jointEntry.second);
    }
}

void GazeboInterface::snapVelocityRecursiveHelper(gazebo::physics::JointPtr & joint)
{
    FloatType velLimit = joint->GetVelocityLimit(0);
    if (velLimit >= 0) {
        FloatType currentVelocity = joint->GetVelocity(0);
        if (currentVelocity > velLimit - 1e-7) {
            setJointVelocityRecursive(joint.get(), velLimit - 1e-7);
        } else if (currentVelocity < -velLimit + 1e-7) {
            setJointVelocityRecursive(joint.get(), -velLimit + 1e-7);
        }
    }

    gazebo::physics::LinkPtr childLink = joint->GetChild();
    for (auto & childJoint : childLink->GetChildJoints()) {
        snapVelocityRecursiveHelper(childJoint);
    }
}

void GazeboInterface::setUpdateSceneFn(std::function<void()> updateSceneFn) {
    updateSceneFn_ = updateSceneFn;
}

GazeboObservationResultUniquePtr GazeboInterface::makeObservationReport(const GazeboObservationRequest * const observationRequest)
{
    boost::mutex::scoped_lock lock(odeMtx);
    GazeboWorldStatePtr currentWorldState = observationRequest->currentWorldState;
    if (!world_) {
        ERROR("World not found");
    }

    world_->GetGazeboWorld()->Reset();
    if (!currentWorldState) {
        WARNING("No current world state");
    } else if (*(lastConstructedWorldState_.get()) != * (currentWorldState.get())) {
        setWorldState(currentWorldState.get(), false, false);
    }

    setStateManuallyNew(observationRequest->currentStateVec);
    GazeboObservationResultUniquePtr observationResult(new GazeboObservationResult());
    observationResult->observationVec = std::vector<FloatType> (observationSpaceDimension_, 0);
    for (auto & observationFunction : getObservationFunctions_) {
        observationFunction(observationResult->observationVec);
    }

    return std::move(observationResult);
}

void GazeboInterface::setCollisionCheckedLinks(const VectorString & collisionCheckedLinks)
{
    collisionCheckedLinks_ = collisionCheckedLinks;
}

std::string GazeboInterface::getUnscopedName(const std::string & name) const
{
    std::string unscopedName = name;
    if (name.find("::") != std::string::npos) {
        VectorString nameElems;
        split(name, "::", nameElems);
        unscopedName = nameElems[nameElems.size() - 1];
    }

    return unscopedName;
}

void GazeboInterface::printWorldState()
{
    auto worldState = lastConstructedWorldState_;
    sdf::ElementPtr sdfElem(new sdf::Element);
    sdf::initFile("world.sdf", sdfElem);
    sdf::ElementPtr stateElem = sdfElem->GetElement("state");
    worldState->getWorldState()->FillSDF(stateElem);
    const std::string pref = "";
    std::string sdfString = stateElem->ToString(pref);
    PRINT(sdfString);

}

void GazeboInterface::printWorldState(GazeboWorldStatePtr & worldState)
{
    sdf::ElementPtr sdfElem(new sdf::Element);
    sdf::initFile("world.sdf", sdfElem);
    sdf::ElementPtr stateElem = sdfElem->GetElement("state");
    worldState->getWorldState()->FillSDF(stateElem);
    const std::string pref = "";
    std::string sdfString = stateElem->ToString(pref);
    PRINT(sdfString);
}

WorldPtr GazeboInterface::getWorld() const
{
    return world_->GetGazeboWorld();
}

void GazeboInterface::setWorldState(const GazeboWorldState *currentWorldState,
                                    const bool & applyDefPoseChanges,
                                    const bool& updateScene)
{
    if (currentWorldState) {
        world_->GetGazeboWorld()->SetState(* (currentWorldState->getWorldState()));
        static_cast<gazebo::physics::OpptODEPhysics*>(world_->GetPhysicsEngine().get())->setCumulativeAngles(
            currentWorldState->getCumulativeAngles());
    }
    if (applyDefPoseChanges) {
        applyDeferredPoseChanges();
    }
    if (updateScene && updateSceneFn_)
        updateSceneFn_();
}

void GazeboInterface::setStateVector(const VectorFloat & stateVector)
{
    setStateManuallyNew(stateVector);
}

void GazeboInterface::setMaxStepSize(const FloatType &stepSize) {
    world_->GetPhysicsEngine()->SetMaxStepSize(stepSize);
}

const GazeboWorldStatePtr GazeboInterface::getWorldState(const bool & generateNewWorldState)
{
    if (generateNewWorldState)
        makeWorldState();
    return lastConstructedWorldState_;
}

GazeboWorldStatePtr GazeboInterface::makeWorldState()
{
    GazeboWorldStatePtr worldState = std::make_shared<GazeboWorldState>(world_->GetGazeboWorld());
    worldState->setCumulativeAngles(
        static_cast<gazebo::physics::OpptODEPhysics*>(world_->GetPhysicsEngine().get())->getCumulativeAngles());
    lastConstructedWorldState_ = worldState;
    return worldState;
}

void GazeboInterface::setBeforePhysicsUpdateFn(std::function<void()> beforePhysicsUpdateFn)
{
    beforePhysicsUpdateFn_ = beforePhysicsUpdateFn;
}


void GazeboInterface::setAfterPhysicsUpdateFn(std::function<void()> afterPhysicsUpdateFn)
{
    afterPhysicsUpdateFn_ = afterPhysicsUpdateFn;
}

GazeboPropagationResultUniquePtr GazeboInterface::doPropagation(const GazeboPropagationRequest * const propagationRequest)
{
    GazeboPropagationResultUniquePtr propagationResult(new GazeboPropagationResult());
    std::unique_ptr<ContactInformation> contactInformation(new ContactInformation());
    if (!world_) {
        ERROR("World is null");
    }

    std::string worldName = world_->GetName();

    world_->GetPhysicsEngine()->Reset();
    size_t numIterations = propagationRequest->duration / world_->GetPhysicsEngine()->GetMaxStepSize();
    if (!propagationRequest->currentWorldState) {
        WARNING("State has no world state");
    } else {
        setWorldState(propagationRequest->currentWorldState.get(), true, true);
    }

    setStateVector(propagationRequest->currentStateVec);
    //world_->GetGazeboWorld()->SetSimTime(lastSimTime_);

    auto jointEntries = worldJointMap_[worldName];
    auto linkEntries = worldRobotLinkMap_[worldName];

    bool breaking = false;
    gazebo::common::Time startTime = world_->GetSimTime();
    FloatType physicsUpdateTime = 0.0;
    FloatType startTimeP = 0.0;

    if (!requiresPhysics_) {
        for (auto & actionFunction : applyActionFunctions_) {
            actionFunction(propagationRequest->actionVec);
        }

        if (propagationRequest->enableCollision) {
            //CollisionReportSharedPtr collisionReport =
            propagationResult->collisionReport = collisionFunction_((void * const)(contactInformation.get()));;
        }
    } else {
        for (size_t i = 0; i != numIterations; ++i) {
            FloatType elapsed = 0.0;
            for (auto & actionFunction : applyActionFunctions_) {
                actionFunction(propagationRequest->actionVec);
            }

            for (auto & jointEntry : jointEntries) {
                jointEntry.second->ApplyStiffnessDamping();
            }

            if (propagationRequest->enableCollision) {
                CollisionReportSharedPtr collisionReport = collisionFunction_((void * const)(contactInformation.get()));
                if (propagationRequest->allowCollisions) {
                    if (!(propagationResult->collisionReport) || !(propagationResult->collisionReport->collides)) {
                        propagationResult->collisionReport = collisionReport;
                    }
                } else {
                    propagationResult->collisionReport = collisionReport;
                    if (propagationResult->collisionReport->collides) {
                        breaking = true;
                        break;
                    }
                }
            }

            if (beforePhysicsUpdateFn_) {
                beforePhysicsUpdateFn_();
            }

            startTimeP = oppt::clock_ms();
            world_->GetPhysicsEngine()->UpdatePhysics();
            physicsUpdateTime += oppt::clock_ms() - startTimeP;
            for (auto & linkEntry : linkEntries) {
#ifdef GZ_GT_7
                linkEntry.second->SetWorldPose(linkEntry.second->DirtyPose(), false);
#else
                linkEntry.second->SetWorldPose(linkEntry.second->GetDirtyPose(), false);
#endif
            }

            if (snapVelocities_)
                snapVelocityRecursive(false);

            if (afterPhysicsUpdateFn_) {
                afterPhysicsUpdateFn_();
            }

            startTime += world_->GetPhysicsEngine()->GetMaxStepSize();
            world_->GetGazeboWorld()->SetSimTime(startTime);
            //cout << "elapsedPhysics: " << elapsed / 1000.0 << endl;


            /**if (saveUserData_ && prefix_ == "exec") {
                auto nextStateVec = getState_(mainWorldName_);
                propagationResult->subStates.push_back(nextStateVec);
            }*/
        }
    }

    //cout << "physicsUpdateTime: " << physicsUpdateTime / 1000.0 << endl;

    lastSimTime_ = world_->GetSimTime();
    propagationResult->nextStateVec = getState_();

    FloatType startMakeWorld = oppt::clock_ms();
    propagationResult->nextWorldState = makeWorldState();
    lastConstructedWorldState_ = propagationResult->nextWorldState;
    propagationResult->contactInformation = std::move(contactInformation);
    return std::move(propagationResult);
}

void GazeboInterface::setStateManuallyNew(const VectorFloat & currentStateVec)
{
    for (size_t i = 0; i < setStateFunctions_.size(); i++) {
        setStateFunctions_[i](currentStateVec);
    }
}

}
