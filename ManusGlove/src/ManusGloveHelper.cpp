/**
 * @file ManusGloveHelper.cpp
 * @authors Ehsan Ranjbari <ehsan.ranjbari@iit.it>
 * @copyright 2024 AMI Lab - Istituto Italiano di Tecnologia
 *            Released under the terms of the BSD-3-Clause
 * @date 2024
 */

#include <ManusGloveHelper.h>
#include <unordered_map>
#include <thread>
#include <cstring>

using namespace manusGlove;
using namespace std;

ManusGloveHelper* ManusGloveHelper::s_Instance = nullptr;
std::mutex manusGlove::ManusGloveHelper::s_InstanceMutex;

ManusGloveHelper::ManusGloveHelper()
{
    std::lock_guard<std::mutex> lock(s_InstanceMutex);
    if (s_Instance != nullptr)
    {
        yInfo() << ManusGlove_LogPrefix << "Found an already existing ManusGloveHelper instance.";
        return;
    }
    s_Instance = this;
}

ManusGloveHelper::~ManusGloveHelper()
{
    std::lock_guard<std::mutex> lock(s_InstanceMutex);
    if (s_Instance != this)
    {
        return;
    }
    s_Instance = nullptr;
}

bool ManusGloveHelper::Initialize(bool p_hostType, bool p_useManusCore)
{
    if (s_Instance != this)
    {
        yInfo() << ManusGlove_LogPrefix << "There is already an instance of ManusGloveHelper. Avoiding initializing.";
        return true;
    }
    m_ShouldConnectLocally = p_hostType;
    // before we can use the SDK, some internal SDK bits need to be initialized.
    // however after initializing, the SDK is not yet connected to a host or doing anything network related just yet.
    // Handle the platform specific initialization
    auto errorCode = p_useManusCore ? CoreSdk_InitializeCore() : CoreSdk_InitializeIntegrated();
    if (errorCode != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "SDK::Failed to initilize Core SDK. Error:" << errorCode;
        return false;
    }

    if (CoreSdk_SetSessionType(SessionType::SessionType_CoreSDK) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "SDK::Failed to set the session type.";
        return false;
    }

    if (!RegisterAllCallbacks())
    {
        yError() << ManusGlove_LogPrefix << "SDK::Failed to register all the callbacks.";
        return false;
    }

    if (!InitializeCoordinateSystem())
    {
        yError() << ManusGlove_LogPrefix << "SDK::Failed to initialize coordinate system.";
        return false;
    }

    // To look and get the number of available hosts.
    if(!LookingForHosts(m_ShouldConnectLocally))
    {
        yError() << ManusGlove_LogPrefix << "Failed to initilize. Check the hosts";
        return false;
    }

    if (!ConnectingToCore())
    {
        yError() << ManusGlove_LogPrefix << "Failed to connect to the core.";
        return false;
    }

    if (CoreSdk_SetRawSkeletonHandMotion(HandMotion::HandMotion_None) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "SDK::Failed to set the raw skeleton hand motion.";
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    PrintDongleData();
    m_startupTime = yarp::os::Time::now();

    return true;
}

bool ManusGloveHelper::ShutDown()
{
    if (s_Instance != this)
    {
        yInfo() << ManusGlove_LogPrefix << "There is another instance of ManusGloveHelper. Avoiding shutting down.";
        return true;
    }

    if (CoreSdk_ShutDown() != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to shut down the SDK wrapper.";
        return false;
    }
    yInfo() << ManusGlove_LogPrefix << "Shutting down the core SDK.";

    return true;
}

bool ManusGloveHelper::RegisterAllCallbacks()
{
    // Regtister the callback for when manus core is connected to he SDK
    // it is optional, but helps trigger your client nicely if needed.
    // see the function OnConnectedCallback for more details
    if (CoreSdk_RegisterCallbackForOnConnect(*OnConnectedCallback) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to register callback function for after connecting to Manus Core. SDK is not available.";
        return false;
    }

    // Register the callback for when manus core is disconnected to the SDK
    // it is optional, but helps trigger your client nicely if needed.
    // see OnDisconnectedCallback for more details.
    if (CoreSdk_RegisterCallbackForOnDisconnect(*OnDisconnectedCallback) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to register callback function for after disconnecting from Manus Core. SDK is not available.";
        return false;
    }

    // Register the callback for when manus core is sending landscape data
    // it is optional, but this allows for a reactive adjustment of device information.
    // see OnLandscapeCallback for more details.
    if (CoreSdk_RegisterCallbackForLandscapeStream(*OnLandscapeCallback) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to register callback for landscape from Manus Core. SDK is not available.";
        return false;
    }

    // Register the callback for when manus core is sending System messages
    // This is usually not used by client applications unless they want to show errors/events from core.
    // see OnSystemCallback for more details.
    if (CoreSdk_RegisterCallbackForSystemStream(*OnSystemCallback) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to register callback function for system feedback from Manus Core. SDK is not available.";
        return false;
    }

    // Register the callback for when manus core is sending Ergonomics data
    // it is optional, but helps trigger your client nicely if needed.
    // see OnErgonomicsCallback for more details.
    if (CoreSdk_RegisterCallbackForErgonomicsStream(*OnErgonomicsCallback) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() <<ManusGlove_LogPrefix << "Failed to register callback function for ergonomics data from Manus Core. SDK is not available.";
        return false;
    }

    // Register the callback for the raw skeleton data
    if (CoreSdk_RegisterCallbackForRawSkeletonStream(*OnRawSkeletonStreamCallback) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to register callback function for raw skeleton stream from Manus Core. SDK is not available.";
        return false;
    }

    return true;
}

bool ManusGloveHelper::InitializeCoordinateSystem()
{
    // after everything is registered and initialized.
    // we must also set the coordinate system being used for the data in this client.
    // (each client can have their own settings. unreal and unity for instance use different coordinate systems)
    // if this is not set, the SDK will not connect to any Manus core host.
    // The coordinate system used for this client is z-up, x-positive, right-handed and in meter scale.
    CoordinateSystemVUH t_VUH;
    CoordinateSystemVUH_Init(&t_VUH);
    t_VUH.handedness = Side::Side_Right;
    t_VUH.up = AxisPolarity::AxisPolarity_PositiveZ;
    t_VUH.view = AxisView::AxisView_XFromViewer;
    t_VUH.unitScale = 1.0f; //1.0 is meters, 0.01 is cm, 0.001 is mm.
    bool p_UseWorldCoordinates = true;


    // The above specified coordinate system is used to initialize and the coordinate space is specified (world/local).
    const SDKReturnCode t_CoordinateResult = CoreSdk_InitializeCoordinateSystemWithVUH(t_VUH, p_UseWorldCoordinates);

    if (t_CoordinateResult != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to Initialize the coordinate system.";
        return false;
    }
    return true;
}

void ManusGloveHelper::LoadSkeleton()
{
    uint32_t t_SklIndex = 0;

    SkeletonSetupInfo t_SKL;
    SkeletonSetupInfo_Init(&t_SKL);
    t_SKL.type = SkeletonType::SkeletonType_Hand;
    t_SKL.settings.scaleToTarget = true;
    t_SKL.settings.useEndPointApproximations = true;
    t_SKL.settings.targetType = SkeletonTargetType::SkeletonTargetType_UserIndexData;
    // If the user does not exist then the added skeleton will not be animated.
    // Same goes for any other skeleton made for invalid users/gloves.
    t_SKL.settings.skeletonTargetUserIndexData.userIndex = 0;

    CopyString(t_SKL.name, sizeof(t_SKL.name), std::string("LeftHand"));//TODO: RightHand? (create a handside string)

    SDKReturnCode t_Res = CoreSdk_CreateSkeletonSetup(t_SKL, &t_SklIndex);
    if (t_Res != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to Create Skeleton Setup.";
        return;
    }

    // setup nodes and chains for the skeleton hand
    if (!SetupHandNodes(t_SklIndex))
        return;
    if (!SetupHandChains(t_SklIndex))
        return;

    // load skeleton
    uint32_t t_ID = 0;
    t_Res = CoreSdk_LoadSkeleton(t_SklIndex, &t_ID);
    if (t_Res != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to load skeleton.";
        return;
    }

    if (t_ID == 0)
    {
        yError() << ManusGlove_LogPrefix << "Failed to give skeleton an ID.";
    }
    m_LoadedSkeletons.push_back(t_ID);
}

NodeSetup ManusGloveHelper::CreateNodeSetup(uint32_t p_Id, uint32_t p_ParentId, float p_PosX, float p_PosY, float p_PosZ, std::string p_Name)
{
    NodeSetup t_Node;
    NodeSetup_Init(&t_Node);
    t_Node.id = p_Id; // Every ID needs to be unique per node in a skeleton.
    CopyString(t_Node.name, sizeof(t_Node.name), p_Name);
    t_Node.type = NodeType::NodeType_Joint;
    // Every node should have a parent unless it is the Root node.
    t_Node.parentID = p_ParentId; // Setting the node ID to its own ID ensures it has no parent.
    t_Node.settings.usedSettings = NodeSettingsFlag::NodeSettingsFlag_None;

    t_Node.transform.position.x = p_PosX;
    t_Node.transform.position.y = p_PosY;
    t_Node.transform.position.z = p_PosZ;
    return t_Node;
}

ManusVec3 ManusGloveHelper::CreateManusVec3(float p_X, float p_Y, float p_Z)
{
    ManusVec3 t_Vec;
    t_Vec.x = p_X;
    t_Vec.y = p_Y;
    t_Vec.z = p_Z;
    return t_Vec;
}

bool manusGlove::ManusGloveHelper::SetHandJoints(const std::vector<std::string>& p_JointNames, bool p_isRightHand)
{
    std::vector<std::string> failures;

    std::unordered_map<std::string, ErgonomicsDataType> leftFingersMap = {
        {"l_thumb_oppose",    ErgonomicsDataType_LeftFingerThumbMCPSpread},
        {"l_thumb_proximal",  ErgonomicsDataType_LeftFingerThumbMCPStretch},
        {"l_thumb_middle",    ErgonomicsDataType_LeftFingerThumbPIPStretch},
        {"l_thumb_distal",    ErgonomicsDataType_LeftFingerThumbDIPStretch},
        {"l_index_abduction", ErgonomicsDataType_LeftFingerIndexMCPSpread},
        {"l_index_proximal",  ErgonomicsDataType_LeftFingerIndexMCPStretch},
        {"l_index_middle",    ErgonomicsDataType_LeftFingerIndexPIPStretch},
        {"l_index_distal",    ErgonomicsDataType_LeftFingerIndexDIPStretch},
        {"l_middle_abduction",ErgonomicsDataType_LeftFingerMiddleMCPSpread},
        {"l_middle_proximal", ErgonomicsDataType_LeftFingerMiddleMCPStretch},
        {"l_middle_middle",   ErgonomicsDataType_LeftFingerMiddlePIPStretch},
        {"l_middle_distal",   ErgonomicsDataType_LeftFingerMiddleDIPStretch},
        {"l_ring_abduction",  ErgonomicsDataType_LeftFingerRingMCPSpread},
        {"l_ring_proximal",   ErgonomicsDataType_LeftFingerRingMCPStretch},
        {"l_ring_middle",     ErgonomicsDataType_LeftFingerRingPIPStretch},
        {"l_ring_distal",     ErgonomicsDataType_LeftFingerRingDIPStretch},
        {"l_pinky_abduction", ErgonomicsDataType_LeftFingerPinkyMCPSpread},
        {"l_pinky_proximal",  ErgonomicsDataType_LeftFingerPinkyMCPStretch},
        {"l_pinky_middle",    ErgonomicsDataType_LeftFingerPinkyPIPStretch},
        {"l_pinky_distal",    ErgonomicsDataType_LeftFingerPinkyDIPStretch}};

    std::unordered_map<std::string, ErgonomicsDataType> rightFingersMap = {
        {"r_thumb_oppose",    ErgonomicsDataType_RightFingerThumbMCPSpread},
        {"r_thumb_proximal",  ErgonomicsDataType_RightFingerThumbMCPStretch},
        {"r_thumb_middle",    ErgonomicsDataType_RightFingerThumbPIPStretch},
        {"r_thumb_distal",    ErgonomicsDataType_RightFingerThumbDIPStretch},
        {"r_index_abduction", ErgonomicsDataType_RightFingerIndexMCPSpread},
        {"r_index_proximal",  ErgonomicsDataType_RightFingerIndexMCPStretch},
        {"r_index_middle",    ErgonomicsDataType_RightFingerIndexPIPStretch},
        {"r_index_distal",    ErgonomicsDataType_RightFingerIndexDIPStretch},
        {"r_middle_abduction",ErgonomicsDataType_RightFingerMiddleMCPSpread},
        {"r_middle_proximal", ErgonomicsDataType_RightFingerMiddleMCPStretch},
        {"r_middle_middle",   ErgonomicsDataType_RightFingerMiddlePIPStretch},
        {"r_middle_distal",   ErgonomicsDataType_RightFingerMiddleDIPStretch},
        {"r_ring_abduction",  ErgonomicsDataType_RightFingerRingMCPSpread},
        {"r_ring_proximal",   ErgonomicsDataType_RightFingerRingMCPStretch},
        {"r_ring_middle",     ErgonomicsDataType_RightFingerRingPIPStretch},
        {"r_ring_distal",     ErgonomicsDataType_RightFingerRingDIPStretch},
        {"r_pinky_abduction", ErgonomicsDataType_RightFingerPinkyMCPSpread},
        {"r_pinky_proximal",  ErgonomicsDataType_RightFingerPinkyMCPStretch},
        {"r_pinky_middle",    ErgonomicsDataType_RightFingerPinkyPIPStretch},
        {"r_pinky_distal",    ErgonomicsDataType_RightFingerPinkyDIPStretch}};

    for (const auto& jointName : p_JointNames)
    {
        if (p_isRightHand)
        {
            auto it = rightFingersMap.find(jointName);
            if (it != rightFingersMap.end())
            {
                m_Joints.push_back(it->second);
            }
            else
            {
                failures.push_back(jointName);
            }
        }
        else
        {
            auto it = leftFingersMap.find(jointName);
            if (it != leftFingersMap.end())
            {
                m_Joints.push_back(it->second);
            }
            else
            {
                failures.push_back(jointName);
            }
        }
    }

    if (!failures.empty())
    {
        std::string hand_name = p_isRightHand ? "right" : "left";
        yError() << ManusGlove_LogPrefix << "Failed to find the following joints in the" << hand_name <<"hand: ";
        for (const auto& failure : failures)
        {
            yError() << ManusGlove_LogPrefix << failure;
        }
        return false;
    }
    return true;
}

inline Eigen::Matrix4f manusGlove::ManusGloveHelper::ManusTransformToEigen(const ManusTransform& p_Transform)
{
    Eigen::Matrix4f t_Matrix;
    t_Matrix.setIdentity();
    t_Matrix.col(3) = Eigen::Vector4f(p_Transform.position.x, p_Transform.position.y, p_Transform.position.z, 1.0f);
    Eigen::Quaternionf t_Quat(p_Transform.rotation.w, p_Transform.rotation.x, p_Transform.rotation.y, p_Transform.rotation.z);
    t_Matrix.block<3, 3>(0, 0) = t_Quat.toRotationMatrix();
    return t_Matrix;
}

bool manusGlove::ManusGloveHelper::getHandRawSkeleton(std::vector<std::pair<std::string, Eigen::Matrix4f>>& p_SkeletonData, bool p_isRightHand)
{
    const RawClientSkeleton& t_RawSkeleton = p_isRightHand ? s_Instance->m_RightGloveRawSkeleton : s_Instance->m_LeftGloveRawSkeleton;
    std::lock_guard<std::mutex> lock(p_isRightHand ? s_Instance->m_RightGloveRawSkeletonMutex : s_Instance->m_LeftGloveRawSkeletonMutex);
    p_SkeletonData.resize(t_RawSkeleton.nodes.size());
    for (size_t i = 0; i < t_RawSkeleton.nodeInfos.size(); ++i)
    {
        const auto& nodeInfo = t_RawSkeleton.nodeInfos[i];
        const auto& node = t_RawSkeleton.nodes[i];
        std::string side = ConvertSideToString(nodeInfo.side);
        std::string chainType = ConvertChainTypeToString(nodeInfo.chainType);
        std::string jointType = ConvertFingerJointTypeToString(nodeInfo.fingerJointType);
        if (nodeInfo.chainType == ChainType_Hand && nodeInfo.fingerJointType == FingerJointType_Invalid)
        {
            // Avoid using "Invalid" in the joint name
            jointType = "Manus";
        }
        p_SkeletonData[i].first = side + chainType + jointType;
        p_SkeletonData[i].second = ManusTransformToEigen(node.transform);
    }
    return true;
}

bool ManusGloveHelper::SetupHandNodes(uint32_t p_SklIndex)
{
    // Define number of fingers per hand and number of joints per finger
    const uint32_t t_NumFingers = 5;
    const uint32_t t_NumJoints = 4;

    // Create an array with the initial position of each hand node.
    // Note, these values are just an example of node positions and refer to the hand laying on a flat surface.
    ManusVec3 t_Fingers[t_NumFingers * t_NumJoints] = {
        CreateManusVec3(0.024950f, 0.000000f, 0.025320f), // Thumb CMC joint
        CreateManusVec3(0.000000f, 0.000000f, 0.032742f), // Thumb MCP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.028739f), // Thumb IP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.028739f), // Thumb Tip joint

        // CreateManusVec3(0.011181f, 0.031696f, 0.000000f), //Index CMC joint // Note: we are not adding the matacarpal bones in this example, if you want to animate the metacarpals add each of them to the corresponding finger chain.
        CreateManusVec3(0.011181f, 0.000000f, 0.052904f), // Index MCP joint, if metacarpal is present: CreateManusVec3(0.000000f, 0.000000f, 0.052904f)
        CreateManusVec3(0.000000f, 0.000000f, 0.038257f), // Index PIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.020884f), // Index DIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.018759f), // Index Tip joint

        // CreateManusVec3(0.000000f, 0.033452f, 0.000000f), //Middle CMC joint
        CreateManusVec3(0.000000f, 0.000000f, 0.051287f), // Middle MCP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.041861f), // Middle PIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.024766f), // Middle DIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.019683f), // Middle Tip joint

        // CreateManusVec3(-0.011274f, 0.031696f, 0.000000f), //Ring CMC joint
        CreateManusVec3(-0.011274f, 0.000000f, 0.049802f), // Ring MCP joint, if metacarpal is present: CreateManusVec3(0.000000f, 0.000000f, 0.049802f),
        CreateManusVec3(0.000000f, 0.000000f, 0.039736f),  // Ring PIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.023564f),  // Ring DIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.019868f),  // Ring Tip joint

        // CreateManusVec3(-0.020145f, 0.027538f, 0.000000f), //Pinky CMC joint
        CreateManusVec3(-0.020145f, 0.000000f, 0.047309f), // Pinky MCP joint, if metacarpal is present: CreateManusVec3(0.000000f, 0.000000f, 0.047309f),
        CreateManusVec3(0.000000f, 0.000000f, 0.033175f),  // Pinky PIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.018020f),  // Pinky DIP joint
        CreateManusVec3(0.000000f, 0.000000f, 0.019129f),  // Pinky Tip joint
    };

    // skeleton entry is already done. just the nodes now.
    // setup a very simple node hierarchy for fingers
    // first setup the root node
    //
    // root, This node has ID 0 and parent ID 0, to indicate it has no parent.
    SDKReturnCode t_Res = CoreSdk_AddNodeToSkeletonSetup(p_SklIndex, CreateNodeSetup(0, 0, 0, 0, 0, "Hand"));
    if (t_Res != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << "Failed to Add Node To Skeleton Setup. The error given was: " << t_Res;
        return false;
    }

    // then loop for 5 fingers
    int t_FingerId = 0;
    for (uint32_t i = 0; i < t_NumFingers; i++)
    {
        uint32_t t_ParentID = 0;
        // then the digits of the finger that are linked to the root of the finger.
        for (uint32_t j = 0; j < t_NumJoints; j++)
        {
            t_Res = CoreSdk_AddNodeToSkeletonSetup(p_SklIndex, CreateNodeSetup(1 + t_FingerId + j, t_ParentID, t_Fingers[i * 4 + j].x, t_Fingers[i * 4 + j].y, t_Fingers[i * 4 + j].z, "fingerdigit"));
            if (t_Res != SDKReturnCode::SDKReturnCode_Success)
            {
                yError() << ManusGlove_LogPrefix << "Failed to Add Node To Skeleton Setup.";
                return false;
            }
            t_ParentID = 1 + t_FingerId + j;
        }
        t_FingerId += t_NumJoints;
    }
    return true;
}

bool ManusGloveHelper::SetupHandChains(uint32_t p_SklIndex)
{
    // Add the Hand chain, this identifies the wrist of the hand
    {
        ChainSettings t_ChainSettings;
        ChainSettings_Init(&t_ChainSettings);
        t_ChainSettings.usedSettings = ChainType::ChainType_Hand;
        t_ChainSettings.hand.handMotion = HandMotion::HandMotion_IMU;
        t_ChainSettings.hand.fingerChainIdsUsed = 5; // we will have 5 fingers
        t_ChainSettings.hand.fingerChainIds[0] = 1;  // links to the other chains we will define further down
        t_ChainSettings.hand.fingerChainIds[1] = 2;
        t_ChainSettings.hand.fingerChainIds[2] = 3;
        t_ChainSettings.hand.fingerChainIds[3] = 4;
        t_ChainSettings.hand.fingerChainIds[4] = 5;

        ChainSetup t_Chain;
        ChainSetup_Init(&t_Chain);
        t_Chain.id = 0; // Every ID needs to be unique per chain in a skeleton.
        t_Chain.type = ChainType::ChainType_Hand;
        t_Chain.dataType = ChainType::ChainType_Hand;
        t_Chain.side = Side::Side_Left;
        t_Chain.dataIndex = 0;
        t_Chain.nodeIdCount = 1;
        t_Chain.nodeIds[0] = 0; // this links to the hand node created in the SetupHandNodes
        t_Chain.settings = t_ChainSettings;

        SDKReturnCode t_Res = CoreSdk_AddChainToSkeletonSetup(p_SklIndex, t_Chain);
        if (t_Res != SDKReturnCode::SDKReturnCode_Success)
        {
            yError() << ManusGlove_LogPrefix << "Failed to Add Chain To Skeleton Setup.";
            return false;
        }
    }

    // Add the 5 finger chains
    const ChainType t_FingerTypes[5] = {ChainType::ChainType_FingerThumb,
                                        ChainType::ChainType_FingerIndex,
                                        ChainType::ChainType_FingerMiddle,
                                        ChainType::ChainType_FingerRing,
                                        ChainType::ChainType_FingerPinky};
    for (int i = 0; i < 5; i++)
    {
        ChainSettings t_ChainSettings;
        ChainSettings_Init(&t_ChainSettings);
        t_ChainSettings.usedSettings = t_FingerTypes[i];
        t_ChainSettings.finger.handChainId = 0; // This links to the wrist chain above.
        // This identifies the metacarpal bone, if none exists, or the chain is a thumb it should be set to -1.
        // The metacarpal bone should not be part of the finger chain, unless you are defining a thumb which does need it.
        t_ChainSettings.finger.metacarpalBoneId = -1;
        t_ChainSettings.finger.useLeafAtEnd = false; // this is set to true if there is a leaf bone to the tip of the finger.
        ChainSetup t_Chain;
        ChainSetup_Init(&t_Chain);
        t_Chain.id = i + 1; // Every ID needs to be unique per chain in a skeleton.
        t_Chain.type = t_FingerTypes[i];
        t_Chain.dataType = t_FingerTypes[i];
        t_Chain.side = Side::Side_Left;
        t_Chain.dataIndex = 0;
        if (i == 0) // Thumb
        {
            t_Chain.nodeIdCount = 4; // The amount of node id's used in the array
            t_Chain.nodeIds[0] = 1;  // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[1] = 2;  // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[2] = 3;  // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[3] = 4;  // this links to the hand node created in the SetupHandNodes
        }
        else // All other fingers
        {
            t_Chain.nodeIdCount = 4;          // The amount of node id's used in the array
            t_Chain.nodeIds[0] = (i * 4) + 1; // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[1] = (i * 4) + 2; // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[2] = (i * 4) + 3; // this links to the hand node created in the SetupHandNodes
            t_Chain.nodeIds[3] = (i * 4) + 4; // this links to the hand node created in the SetupHandNodes
        }
        t_Chain.settings = t_ChainSettings;

        SDKReturnCode t_Res = CoreSdk_AddChainToSkeletonSetup(p_SklIndex, t_Chain);
        if (t_Res != SDKReturnCode::SDKReturnCode_Success)
        {
            return false;
        }
    }
    return true;
}

bool ManusGloveHelper::LookingForHosts(bool p_hostType)
{
    if(!p_hostType)
    {
    yInfo() << ManusGlove_LogPrefix << "Finding a host running Core anywhere on the network...";
    }
    else
    {
    yInfo() << ManusGlove_LogPrefix << "Finding a host running Core locally on the network...";
    }

    // Underlying function will sleep for m_SecondsToFindHosts to allow servers to reply.
    if (CoreSdk_LookForHosts(m_SecondsToFindHosts, p_hostType) == SDKReturnCode::SDKReturnCode_SdkNotAvailable)
    {
        yError() << ManusGlove_LogPrefix << "Failed to look for hosts. Sdk Not Available";
        return false;
    }

    m_NumberOfHostsFound = 0;
    if (CoreSdk_GetNumberOfAvailableHostsFound(&m_NumberOfHostsFound) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to get the number of available hosts.";
        return false;
    }

    if (m_NumberOfHostsFound == 0)
    {
        yWarning() << ManusGlove_LogPrefix << "No hosts found.";
    }
    else
    {
        yInfo() << ManusGlove_LogPrefix << "Found " << m_NumberOfHostsFound << " available hosts.";
    }

    m_AvailableHosts.reset(new ManusHost[m_NumberOfHostsFound]);
    if (CoreSdk_GetAvailableHostsFound(m_AvailableHosts.get(), m_NumberOfHostsFound) != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to get the available hosts.";
        return false;
    }

    return true;
}

bool ManusGloveHelper::ConnectingToCore()
{
    yInfo() << ManusGlove_LogPrefix << "Found the following hosts: ";

    // Note: only 10 hosts are shown, to match the number of number keys, for easy selection.
    for (unsigned int t_HostNumber = 0; t_HostNumber < 10 && t_HostNumber < m_NumberOfHostsFound; t_HostNumber++)
    {
        yInfo() << ManusGlove_LogPrefix << "[ " << t_HostNumber << "] "<<
                " hostname: " << m_AvailableHosts[t_HostNumber].hostName <<
                " IP address: " << m_AvailableHosts[t_HostNumber].ipAddress <<
                " Version " << m_AvailableHosts[t_HostNumber].manusCoreVersion.major << "." <<
                m_AvailableHosts[t_HostNumber].manusCoreVersion.minor << "." <<
                m_AvailableHosts[t_HostNumber].manusCoreVersion.patch;

        m_HostToConnectTo = t_HostNumber;
    }

    SDKReturnCode t_ConnectResult = SDKReturnCode::SDKReturnCode_Error;

    t_ConnectResult = CoreSdk_ConnectToHost(m_AvailableHosts[m_HostToConnectTo]);

    if (t_ConnectResult == SDKReturnCode::SDKReturnCode_NotConnected)
    {
        yWarning() << ManusGlove_LogPrefix << "The host and SDK are not connected.";
        return true; // Differentiating between error and no connect
    }
    if (t_ConnectResult != SDKReturnCode::SDKReturnCode_Success)
    {
        yError() << ManusGlove_LogPrefix << "Failed to connect the host and SDK.";
        return false;
    }

    // Note: a log message from somewhere in the SDK during the connection process can cause text
    // to permanently turn green after this step. Adding a sleep here of 2+ seconds "fixes" the
    // issue. It seems to be caused by a threading issue somewhere, resulting in a log call being
    // interrupted while it is printing the green [info] text. The log output then gets stuck in
    // green mode.

    return true;
}

bool ManusGloveHelper::UpdateBeforeDisplayingData() //TODO: FailedUpdate??
{
    m_LandscapeMutex.lock();
    if (m_NewLandscape != nullptr)
    {
        if (m_Landscape != nullptr)
        {
            delete m_Landscape;
        }
        m_Landscape = m_NewLandscape;
        m_NewLandscape = nullptr;
        m_GestureLandscapeData.swap(m_NewGestureLandscapeData);
    }
    m_LandscapeMutex.unlock();

    m_FirstLeftGloveID = 0;
    m_FirstRightGloveID = 0;
    if (m_Landscape == nullptr)
        return true;
    for (size_t i = 0; i < m_Landscape->gloveDevices.gloveCount; i++)
    {
        if (m_FirstLeftGloveID == 0 && m_Landscape->gloveDevices.gloves[i].side == Side::Side_Left)
        {
            m_FirstLeftGloveID = m_Landscape->gloveDevices.gloves[i].id;
            continue;
        }
        if (m_FirstRightGloveID == 0 && m_Landscape->gloveDevices.gloves[i].side == Side::Side_Right)
        {
            m_FirstRightGloveID = m_Landscape->gloveDevices.gloves[i].id;
            continue;
        }
    }

    return true;
}

bool ManusGloveHelper::DisplayingData()
{
    DisplayingDataGlove();
    PrintSystemMessage();

    return true;
}

bool ManusGloveHelper::DisplayingDataGlove()
{
    yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << "Glove Data: ";
    PrintErgonomicsData();

    return true;
}

void ManusGloveHelper::PrintSystemMessage()
{
    m_SystemMessageMutex.lock();
    yInfo() << ManusGlove_LogPrefix << "Received System data: " << m_SystemMessage << " code: " << m_SystemMessageCode;
    m_SystemMessageMutex.unlock();
}

void ManusGloveHelper::PrintHandErgoData(ErgonomicsData &p_ErgoData, bool p_Left)
{
    const std::string t_FingerNames[NUM_FINGERS_ON_HAND] = {"[thumb] ", "[index] ", "[middle]", "[ring]  ", "[pinky] "};
    const std::string t_FingerJointNames[NUM_FINGERS_ON_HAND] = {"mcp", "pip", "dip"};
    const std::string t_ThumbJointNames[NUM_FINGERS_ON_HAND] = {"cmc", "mcp", "ip "};

    int t_DataOffset = 0;
    if (!p_Left)
        t_DataOffset = 20;

    const std::string *t_JointNames = t_ThumbJointNames;
    for (unsigned int t_FingerNumber = 0; t_FingerNumber < NUM_FINGERS_ON_HAND; t_FingerNumber++)
    {
        yInfo() << ManusGlove_LogPrefix << t_FingerNames[t_FingerNumber] << " " << t_JointNames[0] << " spread: " <<
        RoundFloatValue(p_ErgoData.data[t_DataOffset], 2) << " " << t_JointNames[0] << " stretch: " <<
        RoundFloatValue(p_ErgoData.data[t_DataOffset + 1], 2) << " " << t_JointNames[1] << " stretch: " <<
        RoundFloatValue(p_ErgoData.data[t_DataOffset + 2], 2) << " " << t_JointNames[2] << " stretch: " <<
        RoundFloatValue(p_ErgoData.data[t_DataOffset + 3], 2);

        t_JointNames = t_FingerJointNames;
        t_DataOffset += 4;
    }
}

void ManusGloveHelper::PrintErgonomicsData()
{
    yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " -- Left Glove -- " << m_FirstLeftGloveID;
    if (m_LeftGloveErgoData.id == m_FirstLeftGloveID)
    {
        PrintHandErgoData(m_LeftGloveErgoData, true);
    }

    else
    {
        yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " ...No Data...";
    }

    yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " -- Right Glove -- " << m_FirstRightGloveID;
    if (m_RightGloveErgoData.id == m_FirstRightGloveID)
    {
        PrintHandErgoData(m_RightGloveErgoData, false);
    }
    else
    {
        yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " ...No Data...";
    }

}

void ManusGloveHelper::PrintDongleData()
{
    // get a dongle id
    uint32_t t_DongleCount = 0;
    if (CoreSdk_GetNumberOfDongles(&t_DongleCount) != SDKReturnCode::SDKReturnCode_Success)
        return;
    if (t_DongleCount == 0)
        return; // we got no gloves to work on anyway!

    uint32_t *t_DongleIds = new uint32_t[t_DongleCount]();
    if (CoreSdk_GetDongleIds(t_DongleIds, t_DongleCount) != SDKReturnCode::SDKReturnCode_Success)
        return;

    DongleLandscapeData t_DongleData;

    for (uint32_t i = 0; i < t_DongleCount; i++)
    {
        SDKReturnCode t_Result = CoreSdk_GetDataForDongle(t_DongleIds[i], &t_DongleData);
        yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " -- Dongle -- " << t_DongleData.id;

        if (t_Result == SDKReturnCode::SDKReturnCode_Success)
        {
            yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " Type: "<< ConvertDeviceClassTypeToString(t_DongleData.classType) << "-" << ConvertDeviceFamilyTypeToString(t_DongleData.familyType);

            yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " License: {}" << t_DongleData.licenseType;
        }
        else
        {
            yInfo() << ManusGlove_LogPrefix << ManusGlove_LogPrefix << " ...No Data...";
        }
    }
}

// ==============================================
//                  Callbacks
// ==============================================
void ManusGloveHelper::OnConnectedCallback(const ManusHost *const p_Host)
{
    yInfo() << s_Instance->ManusGlove_LogPrefix << "Connected to Manus Core.";

    // No need to initialize these as they get filled in the CoreSdk_GetVersionsAndCheckCompatibility
    ManusVersion t_SdkVersion;
    ManusVersion t_CoreVersion;
    bool t_IsCompatible;

    const SDKReturnCode t_Result = CoreSdk_GetVersionsAndCheckCompatibility(&t_SdkVersion, &t_CoreVersion, &t_IsCompatible);

    if (t_Result == SDKReturnCode::SDKReturnCode_Success)
    {
        const std::string t_Versions = "Sdk version : " + std::string(t_SdkVersion.versionInfo) + ", Core version : " + std::string(t_CoreVersion.versionInfo) + ".";

        if (t_IsCompatible)
        {
            yInfo() << s_Instance->ManusGlove_LogPrefix << "Versions are compatible.{}" << t_Versions;
        }
        else
        {
            yWarning() << s_Instance->ManusGlove_LogPrefix << "Versions are not compatible with each other.";
        }
    }
    else
    {
        yError() << s_Instance->ManusGlove_LogPrefix << "Failed to get the versions from the SDK.";
    }
    uint32_t t_SessionId;
    const SDKReturnCode t_SessionIdResult = CoreSdk_GetSessionId(&t_SessionId);
    if (t_SessionIdResult == SDKReturnCode::SDKReturnCode_Success && t_SessionId != 0)
    {
        yInfo() << s_Instance->ManusGlove_LogPrefix << "Session Id: {}" << t_SessionId;
        s_Instance->m_SessionId = t_SessionId;
    }
    else
    {
        yInfo() << s_Instance->ManusGlove_LogPrefix << "Failed to get the Session ID from Core. The value returned was{}." << t_SessionIdResult;
    }

    ManusHost t_Host(*p_Host);
    s_Instance->m_Host = std::make_unique<ManusHost>(t_Host);
}

void ManusGloveHelper::OnDisconnectedCallback(const ManusHost* const p_Host)
{
    yInfo() << s_Instance->ManusGlove_LogPrefix << "Disconnected from Manus Core.";

    s_Instance->m_TimeSinceLastDisconnect = std::chrono::high_resolution_clock::now();

    ManusHost t_Host(*p_Host);
    s_Instance->m_Host = std::make_unique<ManusHost>(t_Host);
}

void ManusGloveHelper::OnLandscapeCallback(const Landscape* const p_Landscape)
{
    if (s_Instance == nullptr)
        return;

    Landscape *t_Landscape = new Landscape(*p_Landscape);
    s_Instance->m_LandscapeMutex.lock();
    if (s_Instance->m_NewLandscape != nullptr)
        delete s_Instance->m_NewLandscape;
    s_Instance->m_NewLandscape = t_Landscape;
    s_Instance->m_NewGestureLandscapeData.resize(t_Landscape->gestureCount);
    CoreSdk_GetGestureLandscapeData(s_Instance->m_NewGestureLandscapeData.data(), (uint32_t)s_Instance->m_NewGestureLandscapeData.size());
    s_Instance->m_LandscapeMutex.unlock();
}

void ManusGloveHelper::OnSystemCallback(const SystemMessage* const p_SystemMessage)
{
    if (s_Instance)
    {
        s_Instance->m_SystemMessageMutex.lock();

        switch (p_SystemMessage->type)
        {
        case SystemMessageType::SystemMessageType_TemporarySkeletonModified:
            // if the message was triggered by a temporary skeleton being modified then save the skeleton index,
            // this information will be used to get and load the skeleton into core
            s_Instance->m_ModifiedSkeletonIndex = p_SystemMessage->infoUInt;
            break;
        default:
            s_Instance->m_SystemMessageCode = p_SystemMessage->type;
            s_Instance->m_SystemMessage = p_SystemMessage->infoString;
            break;
        }
        s_Instance->m_SystemMessageMutex.unlock();
    }
}

void ManusGloveHelper::OnErgonomicsCallback(const ErgonomicsStream* const p_Ergo)
{
    if (s_Instance)
    {
        for (uint32_t i = 0; i < p_Ergo->dataCount; i++)
        {
            if (p_Ergo->data[i].isUserID)
                continue;

            ErgonomicsData *t_Ergo = nullptr;
            std::mutex* t_Mutex = nullptr;
            if (p_Ergo->data[i].id == s_Instance->m_FirstLeftGloveID)
            {
                t_Ergo = &s_Instance->m_LeftGloveErgoData;
                t_Mutex = &s_Instance->m_LeftGloveErgoDataMutex;
            }
            if (p_Ergo->data[i].id == s_Instance->m_FirstRightGloveID)
            {
                t_Ergo = &s_Instance->m_RightGloveErgoData;
                t_Mutex = &s_Instance->m_RightGloveErgoDataMutex;
            }
            if (t_Ergo == nullptr)
                continue;
            CoreSdk_GetTimestampInfo(p_Ergo->publishTime, &s_Instance->m_ErgoTimestampInfo);
            t_Ergo->id = p_Ergo->data[i].id;
            t_Ergo->isUserID = p_Ergo->data[i].isUserID;
            for (int j = 0; j < ErgonomicsDataType::ErgonomicsDataType_MAX_SIZE; j++)
            {
                std::lock_guard<std::mutex> lock(*t_Mutex);
                t_Ergo->id = p_Ergo->data[i].id;
                t_Ergo->isUserID = p_Ergo->data[i].isUserID;
                for (int j = 0; j < ErgonomicsDataType::ErgonomicsDataType_MAX_SIZE; j++)
                {
                    t_Ergo->data[j] = p_Ergo->data[i].data[j];// TODO: HumanJointState
                }
            }
        }
    }
}

void ManusGloveHelper::OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const p_RawSkeletonStreamInfo)
{
    if (s_Instance)
    {
        int left_index = -1;
        int right_index = -1;
        {
            std::lock_guard<std::mutex> lock(s_Instance->m_RawSkeletonsMutex);
            s_Instance->m_RawSkeletons.skeletons.resize(p_RawSkeletonStreamInfo->skeletonsCount);

            for (uint32_t i = 0; i < p_RawSkeletonStreamInfo->skeletonsCount; i++)
            {
                CoreSdk_GetRawSkeletonInfo(i, &s_Instance->m_RawSkeletons.skeletons[i].info);
                s_Instance->m_RawSkeletons.skeletons[i].nodes.resize(s_Instance->m_RawSkeletons.skeletons[i].info.nodesCount);
                s_Instance->m_RawSkeletons.skeletons[i].info.publishTime = p_RawSkeletonStreamInfo->publishTime;
                if (s_Instance->m_RawSkeletons.skeletons[i].info.gloveId == s_Instance->m_FirstLeftGloveID)
                {
                    left_index = i;
                }
                else if (s_Instance->m_RawSkeletons.skeletons[i].info.gloveId == s_Instance->m_FirstRightGloveID)
                {
                    right_index = i;
                }
                auto ok = CoreSdk_GetRawSkeletonData(i, s_Instance->m_RawSkeletons.skeletons[i].nodes.data(), s_Instance->m_RawSkeletons.skeletons[i].nodes.size());
                if (ok != SDKReturnCode::SDKReturnCode_Success)
                {
                    yError() << "Failed to get raw skeleton data for skeleton index: " << i;
                }
            }
        }


        if (left_index >= 0)
        {
            std::lock_guard<std::mutex> lock(s_Instance->m_LeftGloveRawSkeletonMutex);
            s_Instance->m_LeftGloveRawSkeleton = s_Instance->m_RawSkeletons.skeletons[left_index];
            s_Instance->m_LeftGloveRawSkeleton.nodeInfos.resize(s_Instance->m_LeftGloveRawSkeleton.info.nodesCount);
            auto ok = CoreSdk_GetRawSkeletonNodeInfoArray(s_Instance->m_LeftGloveRawSkeleton.info.gloveId, s_Instance->m_LeftGloveRawSkeleton.nodeInfos.data(), s_Instance->m_LeftGloveRawSkeleton.nodeInfos.size());
            if (ok != SDKReturnCode::SDKReturnCode_Success)
            {
                yError() << "Failed to get raw skeleton node info for left glove with ID: " << s_Instance->m_LeftGloveRawSkeleton.info.gloveId;
            }

        }
        if (right_index >= 0)
        {
            std::lock_guard<std::mutex> lock(s_Instance->m_RightGloveRawSkeletonMutex);
            s_Instance->m_RightGloveRawSkeleton = s_Instance->m_RawSkeletons.skeletons[right_index];
            s_Instance->m_RightGloveRawSkeleton.nodeInfos.resize(s_Instance->m_RightGloveRawSkeleton.info.nodesCount);
            auto ok = CoreSdk_GetRawSkeletonNodeInfoArray(s_Instance->m_RightGloveRawSkeleton.info.gloveId, s_Instance->m_RightGloveRawSkeleton.nodeInfos.data(), s_Instance->m_RightGloveRawSkeleton.nodeInfos.size());
            if (ok != SDKReturnCode::SDKReturnCode_Success)
            {
                yError() << "Failed to get raw skeleton node info for right glove with ID: " << s_Instance->m_RightGloveRawSkeleton.info.gloveId;
            }
        }
    }
}

// ==============================================
//                  Utility
// ==============================================

float ManusGloveHelper::RoundFloatValue(float p_Value, int p_NumDecimalsToKeep)
{
    // Since C++11, powf is supposed to be declared in <cmath>.
    // Unfortunately, gcc decided to be non-compliant on this for no apparent
    // reason, so now we have to do this.
    // https://stackoverflow.com/questions/5483930/powf-is-not-a-member-of-std
    float t_Power = static_cast<float>(std::pow(
        10.0,
        static_cast<double>(p_NumDecimalsToKeep)));
    return std::round(p_Value * t_Power) / t_Power;
}

std::string ManusGloveHelper::ConvertDeviceClassTypeToString(DeviceClassType p_Type)
{
    switch (p_Type)
    {
    case DeviceClassType_Dongle:
        return "Dongle";
    case DeviceClassType_Glove:
        return "Glove";
    case DeviceClassType_Glongle:
        return "Glongle (Glove Dongle)";
    default:
        return "Unknown";
    }
}

std::string ManusGloveHelper::ConvertDeviceFamilyTypeToString(DeviceFamilyType p_Type)
{
    switch (p_Type)
    {
    case DeviceFamilyType_Prime1:
        return "Prime 1";
    case DeviceFamilyType_Prime2:
        return "Prime 2";
    case DeviceFamilyType_PrimeX:
        return "Prime X";
    case DeviceFamilyType_Metaglove:
        return "Metaglove";
    case DeviceFamilyType_MetaglovePro:
        return "Metaglove Pro";
    case DeviceFamilyType_MetagloveProPrecision:
        return "Metaglove Pro Precision";
    case DeviceFamilyType_Prime3:
        return "Prime 3";
    case DeviceFamilyType_Virtual:
        return "Virtual";
    default:
        return "Unknown";
    }
}

std::string manusGlove::ManusGloveHelper::ConvertChainTypeToString(ChainType p_Type)
{
    switch (p_Type)
    {
    case ChainType_FingerThumb:
        return "Thumb";
    case ChainType_FingerIndex:
        return "Index";
    case ChainType_FingerMiddle:
        return "Middle";
    case ChainType_FingerRing:
        return "Ring";
    case ChainType_FingerPinky:
        return "Pinky";
    case ChainType_Head:
        return "Head";
    case ChainType_Neck:
        return "Neck";
    case ChainType_Spine:
        return "Spine";
    case ChainType_Shoulder:
        return "Shoulder";
    case ChainType_Arm:
        return "Arm";
    case ChainType_Hand:
        return "Hand";
    case ChainType_Foot:
        return "Foot";
    case ChainType_Toe:
        return "Toe";
    case ChainType_Invalid:
        return "Invalid";
    case ChainType_Leg:
        return "Leg";
    default:
        break;
    }
    return "Unknown";
}

inline std::string manusGlove::ManusGloveHelper::ConvertSideToString(Side p_Side)
{
    switch (p_Side)
    {
    case Side_Invalid:
        return "Invalid";
    case Side_Left:
        return "Left";
    case Side_Right:
        return "Right";
    case Side_Center:
        return "Center";
    default:
        break;
    }
    return "Unknown";
}

std::string manusGlove::ManusGloveHelper::ConvertFingerJointTypeToString(FingerJointType p_Type)
{
    switch (p_Type)
    {
     case FingerJointType_Invalid:
         return "Invalid";
     case FingerJointType_Metacarpal:
         return "Metacarpal";
     case FingerJointType_Proximal:
         return "Proximal";
     case FingerJointType_Intermediate:
         return "Intermediate";
     case FingerJointType_Distal:
         return "Distal";
     case FingerJointType_Tip:
         return "Tip";
     default:
         break;
    }
    return "Unknown";
}

bool ManusGloveHelper::CopyString(char *const p_Target, const size_t p_MaxLengthThatWillFitInTarget, const std::string &p_Source)
{
    if (p_Source.size() >= p_MaxLengthThatWillFitInTarget)
    {
        yError() << ManusGlove_LogPrefix << "Copying the string " << p_Source.c_str() << " failed: source exceeds target buffer size";
        return false;
    }
    std::strncpy(p_Target, p_Source.c_str(), p_MaxLengthThatWillFitInTarget - 1);
    p_Target[p_MaxLengthThatWillFitInTarget - 1] = '\0';
    return true;
}

bool ManusGloveHelper::getHandJointPosition(std::vector<double>& jointAngleList, bool p_isRightHand)
{

    if (!UpdateBeforeDisplayingData())
    {
        yError() << ManusGlove_LogPrefix << "Failed to update the joint position.";
        return false;
    }

    jointAngleList.resize(m_Joints.size());

    //Provide 0 values for the first 3 seconds since the data coming from the SDK might not be reliable
    if (yarp::os::Time::now() - m_startupTime < 3)
    {
        std::fill(jointAngleList.begin(), jointAngleList.end(), 0);
        return true;
    }

    float* data = p_isRightHand ? s_Instance->m_RightGloveErgoData.data : s_Instance->m_LeftGloveErgoData.data;
    std::mutex* mutex = p_isRightHand ? &s_Instance->m_RightGloveErgoDataMutex : &s_Instance->m_LeftGloveErgoDataMutex;

    if (data == nullptr)
    {
        yError() << ManusGlove_LogPrefix << "Data is null for the hand.";
        return false;
    }

    std::lock_guard<std::mutex> lock(*mutex);
    {
        for (size_t i = 0; i < m_Joints.size(); i++)
        {
            jointAngleList[i] = RoundFloatValue(data[m_Joints[i]], 2);
        }
    }

    return true;
}
