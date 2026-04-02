/**
 * @file ManusGlove.cpp
 * @authors Ehsan Ranjbari <ehsan.ranjbari@iit.it>
 * @copyright 2024 AMI Lab - Istituto Italiano di Tecnologia
 *            Released under the terms of the BSD-3-Clause
 * @date 2024
 */

#include <ManusGlove.h>
#include <ManusSDK.h>
#include <ManusGloveHelper.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/IFrameTransform.h>

#include <assert.h>
#include <cmath>
#include <map>
#include <mutex>
#include <stdio.h>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <sstream>

#include <Eigen/Dense>
#include <Eigen/Core>

const std::string DeviceName = "ManusGlove";
const std::string LogPrefix = DeviceName + wearable::Separator;

using namespace wearable;
using namespace wearable::devices;

class ManusGlove::ManusGloveImpl
{
public:
    mutable std::mutex mutex;

    WearableName wearableName;

    TimeStamp timeStamp;

    double period;

    bool handSide;
    std::string handSideLogPrefix;

    bool hostType;
    bool useManusCore;

    std::vector<double> humanJointState, humanJointStateDeg;

    //Pointer to the glove object.
    std::unique_ptr<manusGlove::ManusGloveHelper> pGlove; /**< Pointer to the glove object. */

    // Vector of the human joint names
    std::vector<std::string> humanJointNameList;

    // Vector of the human finger names
    std::vector<std::string> humanFingerNames;

    // The name of the human hand link name
    std::string humanHandLinkName;

    std::vector<std::pair<double, double>> jointLimits;
    Eigen::MatrixXd couplingMatrix;
    Eigen::VectorXd offsetVector;

    // Link Sensor
    std::string linkSensorPrefix;
    class ManusGloveVirtualLinkKinSensor;
    std::vector<SensorPtr<ManusGloveVirtualLinkKinSensor>> manusGloveVirtualLinkKinSensor;

    // Joint Sensor
    std::string jointSensorPrefix;
    class ManusGloveVirtualJointKinSensor;
    std::vector<SensorPtr<ManusGloveVirtualJointKinSensor>> manusGloveJointSensorVector;

    yarp::dev::IFrameTransform* frameTransformInterface = nullptr;
    std::string palmFrameName;
    Eigen::Matrix4f palmTransform;
    std::vector<std::pair<std::string, Eigen::Matrix4f>> humanFingerTransforms;
    yarp::sig::Matrix transformBuffer;

    bool publishRawData = false;

    // Constructor
    ManusGloveImpl();

    bool update();

    bool open(yarp::os::Searchable& config);

    bool close();

    // Utilities Methods


};

ManusGlove::ManusGloveImpl::ManusGloveImpl() {}

bool ManusGlove::ManusGloveImpl::open(yarp::os::Searchable& config)
{
    this->pGlove = std::make_unique<manusGlove::ManusGloveHelper>();
    // Parameters from xml file
    period = config.check("period", yarp::os::Value(0.01)).asFloat64();

    // Assigning the correct handside
    if (!config.check("is_right_hand"))
    {
        yError()<<"Cannot find the is_right_hand parameter";
        return false;
    }

    bool isRightHand = config.find("is_right_hand").asBool();
    handSide = isRightHand;
    if (handSide)
    {
        handSideLogPrefix = "Right";
    }
    else
    {
        handSideLogPrefix = "Left";
    }


    // Choose the host type: Remote/Local
    if (!config.check("is_local_host"))
    {
        yError()<<"Cannot find the is_local_host parameter";
        return false;
    }

    bool isLocalHost = config.find("is_local_host").asBool();
    hostType = isLocalHost;

    useManusCore = config.check("use_manus_core", yarp::os::Value(true)).asBool();
    yInfo() << LogPrefix << "use_manus_core: " << (useManusCore ? "true" : "false");

    // Get Human Joint Names
    yarp::os::Bottle* jointListYarp;
    if (!(config.check("human_joint_list") && config.find("human_joint_list").isList())) {
        yError() << LogPrefix << "Unable to find human_joint_list in the config file.";
        return false;
    }
    jointListYarp = config.find("human_joint_list").asList();

    for (size_t i = 0; i < jointListYarp->size(); i++) {
        humanJointNameList.push_back(jointListYarp->get(i).asString());
    }
    yInfo() << LogPrefix << "human joint names: " << humanJointNameList;

    // Get human hand link name
    if (!(config.check("hand_link") && config.find("hand_link").isString())) {
        yError() << LogPrefix << "Unable to find hand_link in the config file.";
        return false;
    }
    humanHandLinkName = config.find("hand_link").asString();
    yInfo() << LogPrefix << "human hand link name: " << humanHandLinkName;

    // Get human hand finger names
    yarp::os::Bottle* fingerListYarp;
    if (!(config.check("human_finger_list") && config.find("human_finger_list").isList()))
    {
        yError() << LogPrefix << "Unable to find human_finger_list in the config file.";
        return false;
    }
    fingerListYarp = config.find("human_finger_list").asList();

    for (size_t i = 0; i < fingerListYarp->size(); i++)
    {
        humanFingerNames.push_back(fingerListYarp->get(i).asString());
    }
    yInfo() << LogPrefix << "human finger names: " << humanFingerNames;


    // Get Robot Joint limit
    // MIN
    yarp::os::Bottle* robotJointLimitMinYarp;
    if (!(config.check("min_joint_limit_deg")) || !config.find("min_joint_limit_deg").isList())
    {
        yError()<<LogPrefix<<"couldn't find min_joint_limit_deg.";
        return false;
    }
    robotJointLimitMinYarp = config.find("min_joint_limit_deg").asList();

    if (robotJointLimitMinYarp->size() != humanJointNameList.size())
    {
        yError() << LogPrefix << "min_joint_limit_deg size is not equal to human_joint_list size.";
        return false;
    }

    // MAX
    yarp::os::Bottle* robotJointLimitMaxYarp;
    if (!(config.check("max_joint_limit_deg")) || !config.find("max_joint_limit_deg").isList())
    {
        yError()<<LogPrefix<<"couldn't find max_joint_limit_deg.";
        return false;
    }
    robotJointLimitMaxYarp = config.find("max_joint_limit_deg").asList();

    if (robotJointLimitMaxYarp->size() != humanJointNameList.size())
    {
        yError() << LogPrefix << "max_joint_limit_deg size is not equal to human_joint_list size.";
        return false;
    }

    for (size_t i = 0; i < humanJointNameList.size(); i++)
    {
        jointLimits.push_back(std::make_pair(robotJointLimitMinYarp->get(i).asFloat64(), robotJointLimitMaxYarp->get(i).asFloat64()));
    }

    //Parse the coupling matrix
    yarp::os::Bottle* couplingMatrixYarp;
    if (!(config.check("coupling_matrix")) || !config.find("coupling_matrix").isList())
    {
        yError() << LogPrefix << "couldn't find coupling_matrix.";
        return false;
    }
    couplingMatrixYarp = config.find("coupling_matrix").asList();
    if (couplingMatrixYarp->size() != humanJointNameList.size() * humanJointNameList.size())
    {
        yError() << LogPrefix << "coupling_matrix size is supposed to be a square matrix with number of rows/cols equal to the number of joints.";
        return false;
    }
    couplingMatrix.resize(humanJointNameList.size(), humanJointNameList.size());
    for (size_t i = 0; i < humanJointNameList.size(); i++)
    {
        for (size_t j = 0; j < humanJointNameList.size(); j++)
        {
            couplingMatrix(i, j) = couplingMatrixYarp->get(i * humanJointNameList.size() + j).asFloat64();
        }
    }
    std::stringstream ss_matrix;
    ss_matrix << std::endl << couplingMatrix;
    yInfo() << LogPrefix << "coupling matrix: " << ss_matrix.str();

    //Parse the offset vector
    yarp::os::Bottle* offsetVectorYarp;
    if (!(config.check("offset_vector_deg")) || !config.find("offset_vector_deg").isList())
    {
        yError() << LogPrefix << "couldn't find offset_vector_deg.";
        return false;
    }
    offsetVectorYarp = config.find("offset_vector_deg").asList();
    if (offsetVectorYarp->size() != humanJointNameList.size())
    {
        yError() << LogPrefix << "offset_vector_deg size is not equal to the number of joints.";
        return false;
    }
    offsetVector.resize(humanJointNameList.size());
    for (size_t i = 0; i < humanJointNameList.size(); i++)
    {
        offsetVector(i) = offsetVectorYarp->get(i).asFloat64();
    }
    std::stringstream ss_vector;
    ss_vector << std::endl << offsetVector;
    yInfo() << LogPrefix << "offset vector: " << ss_vector.str();

    publishRawData = config.check("publish_raw_data", yarp::os::Value(false)).asBool();
    yInfo() << LogPrefix << "publish_raw_data: " << (publishRawData ? "true" : "false");

    if (!pGlove->Initialize(hostType, useManusCore))
    {
        yError() << LogPrefix << "Failed to initialize the ManusGloveHelper on the" << handSideLogPrefix << "hand.";
        return false;
    }
    yInfo() << LogPrefix << "ManusGloveHelper initialized successfully on the" << handSideLogPrefix << "hand.";

    if (!pGlove->SetHandJoints(humanJointNameList, handSide))
    {
        yError() << LogPrefix << "Failed to set the hand joints.";
        return false;
    }

    size_t stateJoints = humanJointNameList.size();
    if (publishRawData)
    {
        stateJoints *= 2; // For raw joint values
    }
    humanJointState.resize(stateJoints, 0);
    humanJointStateDeg.resize(humanJointNameList.size(), 0);

    palmFrameName = config.check("palm_frame_name", yarp::os::Value("")).asString();

    const std::string palmTransformName = "palm_transform";
    palmTransform.setIdentity();
    transformBuffer.resize(4, 4);
    transformBuffer.zero();
    transformBuffer(3, 3) = 1.0; // Set the last element to 1.0 for homogeneous coordinates
    if (config.check(palmTransformName))
    {
        if (!config.find(palmTransformName).isList())
        {
            yError() << LogPrefix << "The palm_transform matrix should be a list!";
            return false;
        }
        yarp::os::Bottle* palmTransformBottle = config.find(palmTransformName).asList();
        if (palmTransformBottle->size() != 16)
        {
            yError() << LogPrefix << "The size of the palm_transform matrix is suposed to be 16! (size:"
                << palmTransformBottle->size() << ")";
            return false;
        }
        for (size_t i = 0; i < 4; i++)
        {
            for (size_t j = 0; j < 4; j++)
            {
                palmTransform(i, j) = palmTransformBottle->get(i * 4 + j).asFloat32();
            }
        }
    }

    return true;
}

bool ManusGlove::ManusGloveImpl::update()
{
    std::lock_guard<std::mutex> lock(mutex);


    if (!pGlove->getHandJointPosition(humanJointStateDeg, handSide))
    {
        yWarningThrottle(5) << LogPrefix << "Failed to get hand joint position. Skipping iteration";
		return true; // Return true to keep the thread running, we can try to get the data in the next iteration.
    }

    //The first half of the humanJointState vector is for the modified joint values after applying the coupling matrix
    //The second part is for the raw joint values as received from the glove (in degrees)
    size_t numJoints = humanJointNameList.size();
    Eigen::Map<Eigen::VectorXd> humanJointStateEigen(humanJointState.data(), numJoints);
    Eigen::Map<Eigen::VectorXd> humanJointStateDegEigen(humanJointStateDeg.data(), humanJointStateDeg.size());

    // Apply the coupling matrix and offset vector
    humanJointStateEigen = couplingMatrix * humanJointStateDegEigen + offsetVector;

    if (publishRawData)
    {
        // The second half of the humanJointState vector is for the raw joint values
        Eigen::Map<Eigen::VectorXd> humanJointStateRawEigen(humanJointState.data() + numJoints, numJoints);
        humanJointStateRawEigen = humanJointStateDegEigen;
    }

    for (size_t i = 0; i < numJoints; i++)
    {
        humanJointState[i] = std::clamp(humanJointState[i], jointLimits[i].first, jointLimits[i].second) * EIGEN_PI / 180;
    }

    bool use_tf = frameTransformInterface != nullptr && !palmFrameName.empty();
    if (use_tf)
    {
        pGlove->getHandRawSkeleton(humanFingerTransforms, handSide);
        for (size_t i = 0; i < humanFingerTransforms.size(); i++)
        {
            Eigen::Matrix4f transform = palmTransform * humanFingerTransforms[i].second;

            for (size_t i = 0; i < 3; ++i)
            {
                for (size_t j = 0; j < 4; ++j)
                {
                    transformBuffer(i, j) = transform(i, j);
                }
            }

            bool ok = frameTransformInterface->setTransform(humanFingerTransforms[i].first, palmFrameName, transformBuffer);
            if (!ok)
            {
                yWarning() << LogPrefix << "Failed to set transform for finger: " << humanFingerTransforms[i].first;
            }
        }
    }

    return true;
}

bool ManusGlove::ManusGloveImpl::close()
{
    pGlove->ShutDown();
    yInfo() << LogPrefix << "Core closed successfully.";
    return true;
}

// Constructor
ManusGlove::ManusGlove()
    : PeriodicThread(0.01)
    , pImpl{std::make_unique<ManusGloveImpl>()}
{}

// Destructor
ManusGlove::~ManusGlove() = default;

bool ManusGlove::open(yarp::os::Searchable& config)
{

    // Check the device name
    if (!(config.check("wearableName") && config.find("wearableName").isString())) {
        yInfo() << LogPrefix << "Using default wearable name ManusGlove";
        pImpl->wearableName = DeviceName;
    }
    else {
        pImpl->wearableName = config.find("wearableName").asString();
        yInfo() << LogPrefix << "Using the wearable name " << pImpl->wearableName;
    }

    // Configure the implementation class
    if (!pImpl->open(config)) {
        yError() << LogPrefix << "Cannot configure the implementation class";
        return false;
    }

    // Create Virtual Joint Sensors
    pImpl->jointSensorPrefix = getWearableName() + sensor::IVirtualJointKinSensor::getPrefix();

    for (int i = 0; i < pImpl->humanJointNameList.size(); i++)
    {
        pImpl->manusGloveJointSensorVector.push_back(std::make_shared<ManusGloveImpl::ManusGloveVirtualJointKinSensor>(pImpl.get(), i, pImpl->jointSensorPrefix + pImpl->humanJointNameList[i]));
    }


    if (pImpl->publishRawData)
    {
        //Add the virtual joint sensors for the raw joint values
        std::string rawJointSensorPrefix = pImpl->jointSensorPrefix + "raw_deg" + wearable::Separator;
        for (int i = 0; i < pImpl->humanJointNameList.size(); i++)
        {
            pImpl->manusGloveJointSensorVector.push_back(std::make_shared<ManusGloveImpl::ManusGloveVirtualJointKinSensor>(pImpl.get(), i + pImpl->humanJointNameList.size(), rawJointSensorPrefix + pImpl->humanJointNameList[i]));
        }
    }

    // Create Virtual Link Sensors
    pImpl->linkSensorPrefix = getWearableName() + sensor::IVirtualLinkKinSensor::getPrefix();
    pImpl->manusGloveVirtualLinkKinSensor.push_back(
        std::make_shared<ManusGloveImpl::ManusGloveVirtualLinkKinSensor>(
            pImpl.get(), pImpl->linkSensorPrefix + pImpl->humanHandLinkName));

    for (size_t i = 0; i < pImpl->humanFingerNames.size(); i++) {
        pImpl->manusGloveVirtualLinkKinSensor.push_back(
            std::make_shared<ManusGloveImpl::ManusGloveVirtualLinkKinSensor>(
                pImpl.get(),
                pImpl->linkSensorPrefix + pImpl->humanFingerNames[i]
                    + "::fingertip"));
    }

    setPeriod(pImpl->period);

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the period thread.";
        return false;
    }

    return true;
}

// =================================================
// ManusGlove implementation of VirtualLinkKinSensor
// =================================================

class ManusGlove::ManusGloveImpl::ManusGloveVirtualLinkKinSensor
    : public sensor::IVirtualLinkKinSensor
{
public:
    ManusGloveVirtualLinkKinSensor(ManusGlove::ManusGloveImpl* gloveImplPtr,
                                   const sensor::SensorName& name = {},
                                   const sensor::SensorStatus& status = sensor::SensorStatus::Ok)
        : IVirtualLinkKinSensor(name, status)
        , gloveImpl(gloveImplPtr)
        , sensorName(name)
    {
        assert(gloveImpl != nullptr);
    }

    ~ManusGloveVirtualLinkKinSensor() override = default;

    bool getLinkAcceleration(Vector3& linear, Vector3& angular) const override
    {

        // we do not handle linear and angular accelerations in the current
        // implementation

        linear.fill(0.0);

        angular.fill(0.0);

        return true;
    }

    bool getLinkPose(Vector3& position, Quaternion& orientation) const override
    {

        // we do not handle position in the current implementation
        position = {0, 0, 0};

        orientation = {0, 0, 0, 0};

        return true;
    }

    bool getLinkVelocity(Vector3& linear, Vector3& angular) const override
    {

        // we do not handle linear and angular velocity

        angular.fill(0.0);

        linear.fill(0.0);

        return true;
    }

    inline void setStatus(const sensor::SensorStatus& status) { m_status = status; }

private:
    ManusGlove::ManusGloveImpl* gloveImpl{nullptr};
    std::string sensorName;
};

// ==================================================
// ManusGlove implementation of VirtualJointKinSensor
// ==================================================

class ManusGlove::ManusGloveImpl::ManusGloveVirtualJointKinSensor
    : public sensor::IVirtualJointKinSensor
{
public:
    ManusGloveVirtualJointKinSensor(ManusGlove::ManusGloveImpl* gloveImplPtr,
                                    const int humanJointIndex,
                                    const sensor::SensorName& name = {},
                                    const sensor::SensorStatus& status = sensor::SensorStatus::Ok)
        : IVirtualJointKinSensor(name, status)
        , gloveImpl(gloveImplPtr)
        , humanJointIndex(humanJointIndex)
    {
        assert(gloveImpl != nullptr);
    }

    ~ManusGloveVirtualJointKinSensor() override = default;

    bool getJointPosition(double& position) const override
    {

        std::lock_guard<std::mutex> lock(gloveImpl->mutex);

        position = gloveImpl->humanJointState[humanJointIndex];

        return true;
    }

    bool getJointVelocity(double& velocity) const override
    {

        std::lock_guard<std::mutex> lock(gloveImpl->mutex);

        velocity = 0.0;

        return true;
    }

    bool getJointAcceleration(double& acceleration) const override
    {

        std::lock_guard<std::mutex> lock(gloveImpl->mutex);

        acceleration = 0.0;

        return true;
    }

    inline void setStatus(const sensor::SensorStatus& status) { m_status = status; }

private:
    ManusGlove::ManusGloveImpl* gloveImpl{nullptr};
    const int humanJointIndex;
};

// ==================================================

void ManusGlove::run()
{
    // Get timestamp
    pImpl->timeStamp.time = yarp::os::Time::now();

    // to implement
    pImpl->update();
}

bool wearable::devices::ManusGlove::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (!(poly->view(pImpl->frameTransformInterface) && pImpl->frameTransformInterface)) {
        yError() << LogPrefix << "Failed to view the IFrameTransform interface from the PolyDriver";
        return false;
    }

    return true;
}

bool wearable::devices::ManusGlove::detach()
{
    pImpl->frameTransformInterface = nullptr;
    return true;
}

bool wearable::devices::ManusGlove::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool wearable::devices::ManusGlove::detachAll()
{
    return detach();
}

bool ManusGlove::close()
{
    this->askToStop();
    while(this->isRunning())
    {
        yDebug()<<"Waiting for the thread to stop ";
        yarp::os::Time::delay(1);

    }

    yDebug()<<"Thread stopped ";

    if (!pImpl->close()) {
        yError() << LogPrefix << "Cannot close correctly the manus glove implementation.";
        return false;
    }

    yDebug()<<"end of closing ";

    return true;
}

// =========================
// IPreciselyTimed interface
// =========================

yarp::os::Stamp ManusGlove::getLastInputStamp()
{
    // Stamp count should be always zero
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return yarp::os::Stamp(0, pImpl->timeStamp.time);
}

// ---------------------------------------
// Implement Sensors and Actuators Methods
// ---------------------------------------

wearable::WearableName ManusGlove::getWearableName() const
{
    return pImpl->wearableName + wearable::Separator;
}

wearable::WearStatus ManusGlove::getStatus() const
{
    wearable::WearStatus status = wearable::WearStatus::Ok;

    for (const auto& s : getAllSensors()) {
        if (s->getSensorStatus() != sensor::SensorStatus::Ok) {
            status = wearable::WearStatus::Error;
            // TO CHECK
            break;
        }
    }
    // Default return status is Ok
    return status;
}

wearable::TimeStamp ManusGlove::getTimeStamp() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return {pImpl->timeStamp.time, 0};
}

wearable::SensorPtr<const wearable::sensor::ISensor>
ManusGlove::getSensor(const wearable::sensor::SensorName name) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> sensors = getAllSensors();
    for (const auto& s : sensors) {
        if (s->getSensorName() == name) {
            return s;
        }
    }
    yWarning() << LogPrefix << "User specified name <" << name << "> not found";
    return nullptr;
}

wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
ManusGlove::getSensors(const wearable::sensor::SensorType aType) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> outVec;
    switch (aType)
    {
        case sensor::SensorType::VirtualLinkKinSensor: {
            outVec.reserve(pImpl->manusGloveVirtualLinkKinSensor.size());
            for (const auto& manusgloveLinkSensor : pImpl->manusGloveVirtualLinkKinSensor)
                outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(manusgloveLinkSensor));
            break;
        }
        case sensor::SensorType::VirtualJointKinSensor: {
            outVec.reserve(pImpl->manusGloveJointSensorVector.size());
            for (const auto& element : pImpl->manusGloveJointSensorVector)
                outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(element));
            break;
        }
        default: {
            return {};
        }
    }

    return outVec;
}

wearable::ElementPtr<const wearable::actuator::IActuator>
ManusGlove::getActuator(const wearable::actuator::ActuatorName name) const
{
    return nullptr;
}

wearable::VectorOfElementPtr<const wearable::actuator::IActuator>
ManusGlove::getActuators(const wearable::actuator::ActuatorType aType) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
ManusGlove::getVirtualLinkKinSensor(const wearable::sensor::SensorName name) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>
ManusGlove::getVirtualJointKinSensor(const wearable::sensor::SensorName name) const
{
    for (SensorPtr<ManusGloveImpl::ManusGloveVirtualJointKinSensor> &element : pImpl->manusGloveJointSensorVector)
    {
        if (name == element->getSensorName())
        {
            return element;
        }
    }

    return nullptr;
}

wearable::ElementPtr<const wearable::actuator::IHaptic>
ManusGlove::getHapticActuator(const actuator::ActuatorName name) const
{
    return nullptr;
}
// =========================
// Defintion of Utilities
// =========================
