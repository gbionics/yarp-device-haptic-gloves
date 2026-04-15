/**
 * @file ManusGloveHelper.h
 * @authors Ehsan Ranjbari <ehsan.ranjbari@iit.it>
 * @copyright 2024 AMI Lab - Istituto Italiano di Tecnologia
 *            Released under the terms of the BSD-3-Clause
 * @date 2024
 */

#ifndef MANUS_GLOVE_HELPER_HPP
#define MANUS_GLOVE_HELPER_HPP

#include <Eigen/Dense>

// std
#include <array>
#include <iostream>
#include <memory>
#include <stdint.h>
#include <mutex>
#include <vector>
#include <chrono> // Used for haptic commands.
#include <cmath> // Used for rounding float values.
#include <iomanip> // Used for printing glove data, and converting glove IDs to strings.

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

// Manus Glove
#include <ManusSDK.h>

/**
 * GloveControlHelper is an helper class for controlling the glove.
 */
namespace manusGlove
{
    enum class ClientReturnCode : int;
    enum class ClientState : int;
    class ManusGloveHelper;
    class RawClientSkeleton;
    class RawClientSkeletonCollection;
    const std::string LogPrefix = "manusGlove::ManusGloveHelper::";
}

// The current state of the client.
enum class manusGlove::ClientState : int
{
    ClientState_PickingConnectionType = 0,
    ClientState_LookingForHosts,
    ClientState_NoHostsFound,
    ClientState_PickingHost,
    ClientState_ConnectingToCore,
    ClientState_DisplayingData,
    ClientState_Disconnected,

    ClientState_MAX_CLIENT_STATE_SIZE
};

// Values that can be returned by this application.
enum class manusGlove::ClientReturnCode : int
{
    ClientReturnCode_Success = 0,
    ClientReturnCode_FailedPlatformSpecificInitialization,
    ClientReturnCode_FailedToResizeWindow,
    ClientReturnCode_FailedToInitialize,
    ClientReturnCode_FailedToFindHosts,
    ClientReturnCode_FailedToConnect,
    ClientReturnCode_UnrecognizedStateEncountered,
    ClientReturnCode_FailedToShutDownSDK,
    ClientReturnCode_FailedPlatformSpecificShutdown,
    ClientReturnCode_FailedToRestart,
    ClientReturnCode_FailedWrongTimeToGetData,

    ClientReturnCode_MAX_CLIENT_RETURN_CODE_SIZE
};

// Used to store the information about the final animated skeletons.
class manusGlove::RawClientSkeleton
{
public:
    RawSkeletonInfo info;
    std::vector<SkeletonNode> nodes;
    std::vector<NodeInfo> nodeInfos;
};

// Used to store all the final animated skeletons received from Core.
class manusGlove::RawClientSkeletonCollection
{
public:
    std::vector<RawClientSkeleton> skeletons;
};

class manusGlove::ManusGloveHelper
{

public:
    /**
      Constructor
    **/

    ManusGloveHelper();
    /**
      Destructor
    **/
    ~ManusGloveHelper();

//===================================================================================================================================================================================
    /**
     * @brief Initialize the SDK, register the callbacks and set the coordinate system.
     * This needs to be done before any of the other SDK functions can be used.
     * @param p_hostType if true it looks for hosts only locally, if false it looks for hosts anywhere in the network-name description
     * @param p_useManusCore if true uses CoreSdk_InitializeCore(), if false uses CoreSdk_InitializeIntegrated()
     */
    bool Initialize(bool p_hostType, bool p_useManusCore = false);

    /**
     * When you are done with the SDK, don't forget to nicely shut it down
     * this will close all connections to the host, close any threads and clean up after itself
     * after this is called it is expected to exit the client program. If not it needs to call initialize again.
     */
    bool ShutDown();
//===================================================================================================================================================================================

    //Callbacks
    /**
     * @brief Gets called when the client is connects to manus core
     * This callback is optional and here it changes the client's state.
     */
    static void OnConnectedCallback(const ManusHost *const p_Host);

    /**
     * @brief called when the client disconnects from manus core.
     * This callback is optional and here it changes the client's state.
     */
    static void OnDisconnectedCallback(const ManusHost* const p_Host);

    /**
     * @brief This gets called when receiving landscape information from core.
     * @param p_Landscape contains the new landscape from core.
     */
    static void OnLandscapeCallback(const Landscape* const p_Landscape);

    /** @brief This gets called when receiving a system message from Core.
     * @param p_SystemMessage contains the system message received from core.
     */
    static void OnSystemCallback(const SystemMessage* const p_SystemMessage);

    /**
     * @brief This gets called when receiving ergonomics data from Manus Core.
     * Ergonomics data gets generated and sent when glove data changes, this means that the stream
     * does not always contain ALL of the devices, because some may not have had new data since
     * the last time the ergonomics data was sent.
     * @param p_Ergo contains the ergonomics data for each glove connected to Core.
     */
    static void OnErgonomicsCallback(const ErgonomicsStream* const p_Ergo);

    /**
     * @brief This gets called when receiving skeleton stream data from Manus Core.
     * The skeleton stream contains the animated skeletons that are being generated by Manus Core.
     * @param p_SkeletonInfo contains the skeletons that have been animated by Manus Core.
     */
    static void OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const p_RawSkeletonStreamInfo);

//===================================================================================================================================================================================

    // Utilities
    /**
     * @brief @brief Round the given float value so that it has no more than the given number of decimals.
     * @param p_Value given float value
     * @param p_NumDecimalsToKeep number of decimals to keep
     * @return float
     */
    float RoundFloatValue(float p_Value, int p_NumDecimalsToKeep);

    /**
     * @brief Convert Device Class Type To String
     * @param p_Type
     * @return std::string
     */
    std::string ConvertDeviceClassTypeToString(DeviceClassType p_Type);

    /**
     * @brief Convert Device Family Type To String
     * @param p_Type
     * @return std::string
     */
    std::string ConvertDeviceFamilyTypeToString(DeviceFamilyType p_Type);

    /**
     * @brief Convert Chain Type To String
     * @param p_Type
     * @return std::string
     */
    std::string ConvertChainTypeToString(ChainType p_Type);

    /**
     * @brief Convert Side To String
     * @param p_Side
     * @return std::string
     */
    std::string ConvertSideToString(Side p_Side);

    /**
     * @brief Convert Finger Joint Type To String
     * @param p_Type
     * @return std::string
     */
    std::string ConvertFingerJointTypeToString(FingerJointType p_Type);

    /**
     * @brief Copy the given string into the given target.
     *
     * @param p_Target Given Target
     * @param p_MaxLengthThatWillFitInTarget
     * @param p_Source Given String
     * @return true true if it is successful
     */
    bool CopyString(char *const p_Target, const size_t p_MaxLengthThatWillFitInTarget, const std::string &p_Source);

    /**
     * @brief Gets the hand joint angles (in degrees) from the Manus gloves.
     *
     * @param jointAngleList
     * @param p_isRightHand - true: Right hand, false: Left hand
     * @return true if it is successful
     */
    bool getHandJointPosition(std::vector<double>& jointAngleList, bool p_isRightHand);


    /**
     * Specify the hand joints that you want to get the position of.
     * @param p_isRightHand - true: Right hand, false: Left hand
     */
    bool SetHandJoints(const std::vector<std::string>& p_JointNames, bool p_isRightHand);

    /**
    * Converts a ManusTransform to an Eigen::Matrix4f.
    */
    Eigen::Matrix4f ManusTransformToEigen(const ManusTransform& p_Transform);

    /**
    * Get the raw skeleton data of a hand.
    */
    bool getHandRawSkeleton(std::vector<std::pair<std::string, Eigen::Matrix4f>>& p_SkeletonData, bool p_isRightHand);

    //===================================================================================================================================================================================

protected:

    /**
     * Used to register the callbacks between sdk and core.
     * Callbacks that are registered functions that get called when a certain 'event' happens, such as data coming in from Manus Core.
     * All of these are optional, but depending on what data you require you may or may not need all of them.
     */
    virtual bool RegisterAllCallbacks();

    /**
     * Used to Initialize the Coordinate system.(VUH or Direction).
     */
    virtual bool InitializeCoordinateSystem();
//TODO
    /** @brief This is to determine how the client is going to connect to manus core.
     * For now, we want to Find a host running Core anywhere on the network.
     * @param p_hostType if true it looks for hosts only locally, if false it looks for hosts anywhere in the network
     */
    virtual bool LookingForHosts(bool p_hostType);

    /** @brief Prints the available hosts and connecting to manus core via the SDK.
     */
    virtual bool ConnectingToCore();

    /** @brief Some things happen before every display update, no matter what state.
     * They happen here, such as the updating of the landscape and the generated tracker
     */
    virtual bool UpdateBeforeDisplayingData();

    /** @brief Once the connections are made we loop this function
     * it calls all the input handlers for different aspects of the SDK
     * and then prints any relevant data of it.
     */
    virtual bool DisplayingData();

    /** @brief Displays the ergonomics data of the gloves.
     */
    virtual bool DisplayingDataGlove();

    /**
     * @brief Prints the ergonomics data of a hand.
     * @param p_ErgoData
     * @param p_Left
     */
    void PrintHandErgoData(ErgonomicsData &p_ErgoData, bool p_Left);

    /**
     * @brief Print the ergonomics data received from Core.
     */
    void PrintErgonomicsData();

    /**
     * @brief Prints the dongle data received from Core.
     */
    void PrintDongleData();

    /**
     * @brief Prints the last received system messages received from Core.
     */
    void PrintSystemMessage();

    /**
     * This support function sets up the nodes for the skeleton hand
     * In order to have any 3d positional/rotational information from the gloves or body,
     * one needs to setup the skeleton on which this data should be applied.
     * In the case of this sample we create a Hand skeleton for which we want to get the calculated result.
     * The ID's for the nodes set here are the same IDs which are used in the OnSkeletonStreamCallback,
     * this allows us to create the link between Manus Core's data and the data we enter here.
     */
    bool SetupHandNodes(uint32_t p_SklIndex);

    /**
     * This function sets up some basic hand chains.
     * Chains are required for a Skeleton to be able to be animated, it basically tells Manus Core
     * which nodes belong to which body part and what data needs to be applied to which node.
     * @param p_SklIndex The index of the temporary skeleton on which the chains will be added.
     * @return Returns true if everything went fine, otherwise returns false.
     */
    bool SetupHandChains(uint32_t p_SklIndex);

    /**
     * This function sets up a very minimalistic hand skeleton.
     * In order to have any 3d positional/rotational information from the gloves or body,
     * one needs to setup a skeleton on which this data can be applied.
     * In the case of this sample we create a Hand skeleton in order to get skeleton information
     * in the OnSkeletonStreamCallback function. This sample does not contain any 3D rendering, so
     * we will not be applying the returned data on anything.
     */
    void LoadSkeleton();

    /**
     * Skeletons are pretty extensive in their data setup
     * so we have several support functions so we can correctly receive and parse the data,
     * this function helps setup the data.
     * @param p_Id the id of the created node setup
     * @param p_ParentId the id of the node parent
     * @param p_PosX X position of the node, this is defined with respect to the global coordinate system or the local one depending on
     * the parameter p_UseWorldCoordinates set when initializing the sdk,
     * @param p_PosY Y position of the node this is defined with respect to the global coordinate system or the local one depending on
     * the parameter p_UseWorldCoordinates set when initializing the sdk,
     * @param p_PosZ Z position of the node this is defined with respect to the global coordinate system or the local one depending on
     * the parameter p_UseWorldCoordinates set when initializing the sdk,
     * @param p_Name the name of the node setup
     * @return the generated node setup
     */
    NodeSetup CreateNodeSetup(uint32_t p_Id, uint32_t p_ParentId, float p_PosX, float p_PosY, float p_PosZ, std::string p_Name);

    /**
     *
     */
    static ManusVec3 CreateManusVec3(float p_X, float p_Y, float p_Z);

protected:
    static ManusGloveHelper* s_Instance;
    static std::mutex s_InstanceMutex;
    bool m_Running = true;
    std::string ManusGlove_LogPrefix = "ManusGloveHelper::";

    std::function<ClientReturnCode()> m_CurrentInteraction = nullptr;

    // Networking
    bool m_ShouldConnectLocally = true;

    uint32_t m_HostToConnectTo = 0;
    uint32_t m_NumberOfHostsFound = 0;
    uint32_t m_SecondsToFindHosts = 2;

    int32_t m_SecondsToAttemptReconnecting = 60;
    int32_t m_MaxReconnectionAttempts = 10;
    uint32_t m_SleepBetweenReconnectingAttemptsInMs = 100;

    std::unique_ptr<ManusHost[]> m_AvailableHosts = nullptr;
    std::unique_ptr<ManusHost> m_Host;

    //Data
    uint32_t m_SessionId = 0;

    std::vector<uint32_t> m_LoadedSkeletons;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_TimeSinceLastDisconnect;

    std::mutex m_LandscapeMutex;
    Landscape *m_NewLandscape = nullptr;
    Landscape *m_Landscape = nullptr;
    std::vector<GestureLandscapeData> m_NewGestureLandscapeData;
    std::vector<GestureLandscapeData> m_GestureLandscapeData;

    ManusTimestampInfo m_ErgoTimestampInfo;
    std::mutex m_LeftGloveErgoDataMutex;
    ErgonomicsData m_LeftGloveErgoData;
    std::mutex m_RightGloveErgoDataMutex;
    ErgonomicsData m_RightGloveErgoData;

    RawClientSkeleton m_LeftGloveRawSkeleton;
    RawClientSkeleton m_RightGloveRawSkeleton;
    RawClientSkeletonCollection m_RawSkeletons;
    std::mutex m_LeftGloveRawSkeletonMutex;
    std::mutex m_RightGloveRawSkeletonMutex;
    std::mutex m_RawSkeletonsMutex;


    uint32_t m_FirstLeftGloveID = 0;
    uint32_t m_FirstRightGloveID = 0;

    std::mutex m_SystemMessageMutex;
    std::string m_SystemMessage = "";
    SystemMessageType m_SystemMessageCode = SystemMessageType::SystemMessageType_Unknown;
    uint32_t m_ModifiedSkeletonIndex = UINT_MAX;

    uint32_t m_FrameCounter = 0;

    std::vector<ErgonomicsDataType> m_Joints;

    double m_startupTime = 0.0;
};
#endif
