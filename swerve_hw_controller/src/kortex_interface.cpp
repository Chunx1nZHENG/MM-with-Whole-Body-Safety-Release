#include "kortex_interface.hpp"
#include <google/protobuf/util/json_util.h>

KortexRobot::KortexRobot(const std::string &IP)
{
    m_sIP = IP;
    m_bIsConnected = false;
    m_bIsBusy = false;
    Init();
}

KortexRobot::~KortexRobot()
{
    std::cout<<"Kortex exit, set SINGLE_LEVEL_SERVOING"<<"\n";
    auto servoingMode = Kinova::Api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    m_pBase->SetServoingMode(servoingMode);
    Disconnect();
}

void KortexRobot::Disconnect()
{
    if (m_bIsConnected){
        m_pSessionManager->CloseSession();
        m_pSessionManagerRT->CloseSession();

        m_pRouterClient->SetActivationStatus(false);
        m_pRouterClientRT->SetActivationStatus(false);

        m_pTcpClient->disconnect();
        m_pUdpClient->disconnect();
    }

    m_bIsConnected = false;

    if (m_pControlConfigClient != nullptr)
    {
        delete m_pControlConfigClient;
        m_pControlConfigClient = nullptr;
    }

    if (m_pBase != nullptr)
    {
        delete m_pBase;
        m_pBase = nullptr;
    }
    if (m_pDeviceConfigClient != nullptr)
    {
        delete m_pDeviceConfigClient;
        m_pDeviceConfigClient = nullptr;
    }
    if (m_pSessionManager != nullptr)
    {
        delete m_pSessionManager;
        m_pSessionManager = nullptr;
    }
    if (m_pRouterClient != nullptr)
    {
        delete m_pRouterClient;
        m_pRouterClient = nullptr;
    }
    if (m_pTcpClient != nullptr)
    {
        delete m_pRouterClient;
        m_pTcpClient = nullptr;
    }

    if (m_pSessionManagerRT != nullptr)
    {
        delete m_pSessionManagerRT;
        m_pSessionManagerRT = nullptr;
    }
    if (m_pRouterClientRT != nullptr)
    {
        delete m_pRouterClientRT;
        m_pRouterClientRT = nullptr;
    }
    if (m_pUdpClient != nullptr)
    {
        delete m_pUdpClient;
        m_pUdpClient = nullptr;
    }
}

bool KortexRobot::Init()
{
    if (m_bIsConnected)
    {
        Disconnect();
    }
    auto error_callback = [] (Kinova::Api::KError err) { cout << " callback error" << err.toString();};
    std::cout << "debug 1" << std::endl;
    m_pTcpClient = new Kinova::Api::TransportClientTcp();
    std::cout << "debug 2" << std::endl;
    m_pRouterClient = new Kinova::Api::RouterClient(m_pTcpClient, error_callback);
    std::cout<<m_sIP<<std::endl;
    
    if(!m_pTcpClient->connect(m_sIP, 10000))
    {
        std::cout << "Failed to connect" << std::endl;
        return false;
    }
    auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout((60000));
    create_session_info.set_connection_inactivity_timeout((2000));
    std::cout << "debug3" << std::endl;
    try
    {
        m_pSessionManager = new Kinova::Api::SessionManager(m_pRouterClient);
        m_pSessionManager->CreateSession(create_session_info);

        //Access devices
        m_pDeviceConfigClient = new Kinova::Api::DeviceConfig::DeviceConfigClient(m_pRouterClient);
        m_pBase = new Kinova::Api::Base::BaseClient(m_pRouterClient);
        m_pControlConfigClient = new Kinova::Api::ControlConfig::ControlConfigClient(m_pRouterClient);

        m_nNbDOF = m_pBase->GetActuatorCount().count();
        auto servoingMode = Kinova::Api::Base::ServoingModeInformation();
        servoingMode.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        m_pBase->SetServoingMode(servoingMode);
        std::cout << "debug4" << std::endl;
    }
    catch(Kinova::Api::KDetailedException ex)
    {
        OnError(ex);
        Disconnect();
        return false;
    }

    //connecting the cyclic clients
    m_pUdpClient = new Kinova::Api::TransportClientUdp();
    m_pRouterClientRT = new Kinova::Api::RouterClient(m_pUdpClient, error_callback);
    std::cout << "debug5" << std::endl;
    if (!m_pUdpClient->connect(m_sIP, 10001))
    {
        std::cout << "Failed to connect cyclic" << std::endl;
        Disconnect();
        return false;
    }
    try
    {
        m_pSessionManagerRT = new Kinova::Api::SessionManager(m_pRouterClientRT);
        m_pSessionManagerRT->CreateSession(create_session_info);
        m_pBaseCyclic = new Kinova::Api::BaseCyclic::BaseCyclicClient(m_pRouterClientRT);
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
        Disconnect();
        return false;
    }
    
    m_Feedback =  m_pBaseCyclic->RefreshFeedback();
    for (int i=0; i<m_nNbDOF; ++i){
        commands.push_back(m_Feedback.actuators(i).position());
        base_command.add_actuators()->set_position(m_Feedback.actuators(i).position());
    }
    std::cout << "debug6" << std::endl;
    m_bIsConnected = true;
    return true;
}

bool KortexRobot::ReadAndSetJointSpeeds(std::vector<float> &jointSpeeds)
{
    m_Feedback =  m_pBaseCyclic->RefreshFeedback();
    if (!m_bIsConnected)
    {
        //Speed Commands have priority over actions
        return false;
    }

    if (jointSpeeds.size() != m_nNbDOF)
    {
        return false;
    }
    auto lambda_fct_callback = [](const Kinova::Api::Error &err, const Kinova::Api::BaseCyclic::Feedback data)
    {
        // We are printing the data of the moving actuator just for the example purpose,
        // avoid this in a real-time loop
        std::string serialized_data;
        google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
        // std::cout << serialized_data << std::endl << std::endl;
    };
    for (int i=0; i<m_nNbDOF; ++i){
        commands[i]+=0.002f * jointSpeeds[i];
        base_command.mutable_actuators(i)->set_position(fmod(commands[i],360.0f));
    }
    try
    {
        // RefreshCommand
  
        m_pBaseCyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
    }
    
    return true;
}
void KortexRobot::SubscribeToNotifications()
{
    if (!m_bIsConnected)
    {
        return;
    }
    Kinova::Api::Common::NotificationOptions options;
    options.set_type(Kinova::Api::Common::NOTIFICATION_TYPE_EVENT);

    using namespace std::placeholders;

    std::function<void(Kinova::Api::Base::ActionNotification)> actionCallback = std::bind(&KortexRobot::OnActionNotificationCallback, this, _1);
    auto handle = m_pBase->OnNotificationActionTopic(actionCallback, options);
    m_NotificationHandleList.push_back(handle);
}

void KortexRobot::UnsubscribeToNotifications()
{
    if (!m_bIsConnected)
    {
        return;
    }

    for (auto handle:m_NotificationHandleList)
    {
        m_pBase->Unsubscribe(handle);
    }
}

void KortexRobot::OnError(Kinova::Api::KDetailedException &ex)
{
    // You can print the error informations and error codes
    auto error_info = ex.getErrorInfo().getError();
    std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;
    
    std::cout << "KError error_code: " << error_info.error_code() << std::endl;
    std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
    std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

    // Error codes by themselves are not very verbose if you don't see their corresponding enum value
    // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
    std::cout << "Error code string equivalent: " << Kinova::Api::ErrorCodes_Name(Kinova::Api::ErrorCodes(error_info.error_code())) << std::endl;
    std::cout << "Error sub-code string equivalent: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
}

void KortexRobot::OnActionNotificationCallback(Kinova::Api::Base::ActionNotification notif)
{
    switch (notif.action_event())
    {
        case Kinova::Api::Base::ACTION_START:
        {
            m_bIsBusy = true;
            // std::cout << "Action ID " << notif.handle().identifier() << " has started" << std::endl;
            break;
        }
        case Kinova::Api::Base::ACTION_END:
        {
            m_bIsBusy = false;
            // std::cout << "Action ID " << notif.handle().identifier() << " has ended" << std::endl;
            break;
        }
        case Kinova::Api::Base::ACTION_ABORT:
        {
            m_bIsBusy = false;
            // std::cout << "Action ID " << notif.handle().identifier() << " has aborted" << std::endl;
            break;
        }
        case Kinova::Api::Base::ACTION_PAUSE:
        {
            m_bIsBusy = false;
            // std::cout << "Action ID " << notif.handle().identifier() << "has paused" << std::endl;
            break;
        }
    }
}

bool KortexRobot::ExcuteExistingAction(const std::string &actionName, Kinova::Api::Base::RequestedActionType &actiontpye)
{
    if (!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }

    try
    {
        auto action_list = m_pBase->ReadAllActions(actiontpye);
        auto action_handle = Kinova::Api::Base::ActionHandle();
        action_handle.set_identifier(0);
        for (auto action : action_list.action_list())
        {
            if (action.name() == actionName)
            {
                action_handle = action.handle();
            }
        }
        if (action_handle.identifier() == 0)
        {
            return false;
        }
        else
        {
            m_pBase->ExecuteActionFromReference(action_handle);
        }
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return true;
}

bool KortexRobot::MoveTo(const tCartesianVector position, const tCartesianVector orientation, const Kinova::Api::Base::CartesianTrajectoryConstraint &constraint)
{
    if (!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }

    m_Action.mutable_handle()->set_action_type(Kinova::Api::Base::REACH_POSE);
    m_Action.mutable_reach_pose()->mutable_target_pose()->set_x(position.x);
    m_Action.mutable_reach_pose()->mutable_target_pose()->set_y(position.y);
    m_Action.mutable_reach_pose()->mutable_target_pose()->set_z(position.z);
    m_Action.mutable_reach_pose()->mutable_target_pose()->set_theta_x(orientation.x);
    m_Action.mutable_reach_pose()->mutable_target_pose()->set_theta_y(orientation.y);
    m_Action.mutable_reach_pose()->mutable_target_pose()->set_theta_z(orientation.z);

    switch (constraint.type_case())
    {
        case Kinova::Api::Base::CartesianTrajectoryConstraint::TypeCase::kDuration:
        {
            m_Action.mutable_reach_pose()->mutable_constraint()->set_duration(constraint.duration());
        }
        case Kinova::Api::Base::CartesianTrajectoryConstraint::TypeCase::kSpeed:
        {
            m_Action.mutable_reach_pose()->mutable_constraint()->mutable_speed()->set_translation(constraint.speed().translation());
            m_Action.mutable_reach_pose()->mutable_constraint()->mutable_speed()->set_orientation(constraint.speed().orientation());
        }
        default:
        {

        }
    }
    
    try
    {
        m_pBase->ExecuteAction(m_Action);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return true;
}

bool KortexRobot::SetJointAngles(const std::vector<float> &angles, const Kinova::Api::Base::JointTrajectoryConstraint &constraints)
{
    if (!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }

    if (angles.size() != m_nNbDOF)
    {
        return false;
    }

    m_Action.mutable_handle()->set_action_type(Kinova::Api::Base::REACH_JOINT_ANGLES);

    Kinova::Api::Base::ConstrainedJointAngles* constraintedJoint = m_Action.mutable_reach_joint_angles();
    Kinova::Api::Base::JointAngles* jointAngles = constraintedJoint->mutable_joint_angles();
    jointAngles->clear_joint_angles();

    for (int a=0; a < m_nNbDOF; ++a)
    {
        Kinova::Api::Base::JointAngle* jointAngle = jointAngles->add_joint_angles();
        jointAngle->set_joint_identifier(a);
        jointAngle->set_value(angles[a]);
    }

    if (constraints.type() != Kinova::Api::Base::JointTrajectoryConstraintType::UNSPECIFIED_JOINT_CONSTRAINT)
    {
        constraintedJoint->mutable_constraint()->set_type(constraints.type());
        constraintedJoint->mutable_constraint()->set_value(constraints.value());
    }

    try
    {
        m_pBase->ExecuteAction(m_Action);
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return true;
}

bool KortexRobot::SendTwistCommand(const tCartesianVector &translation, const tCartesianVector &rotation)
{
    if (!m_bIsConnected)
    {
        // Twist command have priority over actions
        return false;
    }

    // Build twist command
    Kinova::Api::Base::TwistCommand cmd;
    cmd.mutable_twist()->set_linear_x(translation.x);
    cmd.mutable_twist()->set_linear_y(translation.y);
    cmd.mutable_twist()->set_linear_z(translation.z);
    cmd.mutable_twist()->set_angular_x(rotation.x);
    cmd.mutable_twist()->set_angular_y(rotation.y);
    cmd.mutable_twist()->set_angular_z(rotation.z);

    try
    {
        m_pBase->SendTwistCommand(cmd);
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
    }

    return true;
}

bool KortexRobot::SetTwistReferenceFrame(const Kinova::Api::Common::CartesianReferenceFrame &frame)
{
    if (!m_bIsConnected)
    {
        return false;
    }

    Kinova::Api::ControlConfig::CartesianReferenceFrameInfo frameRequest;
    frameRequest.set_reference_frame(frame);

    try
    {
        m_pControlConfigClient->SetCartesianReferenceFrame(frameRequest);
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
    }
    

    return true;
}

bool KortexRobot::SetJointSpeeds(std::vector<float> &jointSpeeds)
{
    if (!m_bIsConnected)
    {
        //Speed Commands have priority over actions
        return false;
    }

    if (jointSpeeds.size() != m_nNbDOF)
    {
        return false;
    }
    Kinova::Api::Base::JointSpeeds speeds;
    for (int a=0; a<m_nNbDOF; ++a)
    {
        Kinova::Api::Base::JointSpeed* speed = speeds.add_joint_speeds();
        speed->set_joint_identifier(a);
        speed->set_value(jointSpeeds[a]);
        std::cout<<jointSpeeds[a]<<" ";
        speed->set_duration(1);
    }
    try
    {
        m_pBase->SendJointSpeedsCommand(speeds);
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
    }
    
    return true;
}

bool KortexRobot::PlaySequence(const Kinova::Api::Base::SequenceHandle &sequenceHandle)
{
    if (!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }
    try
    {
        m_pBase->PlaySequence(sequenceHandle);
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
    }
    
    return true;
}

bool KortexRobot::ExecuteAction(const Kinova::Api::Base::Action &action)
{
    if (!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }

    try
    {
        m_pBase->ExecuteAction(action);
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
    }
    
    return true;
}

bool KortexRobot::WaitWhileRobotIsMoving(const int timeout)
{
    if (!m_bIsConnected)
    {
        return false;
    }

    int cycle = 0;
    
    while(m_bIsBusy && cycle < timeout)
    {
        cycle++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (cycle >= timeout)
    {
        return false;
    }

    return true;
}

bool KortexRobot::Stop()
{
    if (!m_bIsConnected)
    {
        return false;
    }
    try
    {
        m_pBase->Stop();
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
    }
    
    return true;
}

bool KortexRobot::RefreshFeedback()
{
    if (!m_bIsConnected)
    {
        return false;
    }
    try
    {
       m_Feedback =  m_pBaseCyclic->RefreshFeedback();
    }
    catch(Kinova::Api::KDetailedException& ex)
    {
        OnError(ex);
        return false;
    }
    
    return true;
}

// void KortexRobot::OnRefreshFeedbackCallback(const Kinova::Api::Error &err, const Kinova::Api::BaseCyclic::Feedback &feedback)
// {
//     m_Feedback = feedback;

//     // Insert collision detection example here
//     std::vector<float> torques = GetJointTorques();

//     if (!m_CustomData.empty())
//     {
//         for (int i = 0; i < m_nNbDOF; ++i)
//         {
//             if (fabs(torques[i] - m_CustomData[i]) > 0.5f)
//             {
//                 std::cout << "Collision detected!" << std::endl;
//                 break;
//             }
//         }
//     }
// }

Kinova::Api::BaseCyclic::ActuatorFeedback KortexRobot::GetActuatorFeedback(const int actIdx) const
{
    if (actIdx >= m_nNbDOF)
    {
        return Kinova::Api::BaseCyclic::ActuatorFeedback();
    }

    return m_Feedback.actuators(actIdx);
}

Kinova::Api::Base::Pose KortexRobot::GetCartesianPose() const
{
    Kinova::Api::Base::Pose pose;

    if (!m_bIsConnected)
    {
        return pose;
    }
    Kinova::Api::BaseCyclic::BaseFeedback bf = m_Feedback.base();

    pose.set_x(bf.tool_pose_x());
    pose.set_y(bf.tool_pose_y());
    pose.set_z(bf.tool_pose_z());
    pose.set_theta_x(bf.tool_pose_theta_x());
    pose.set_theta_y(bf.tool_pose_theta_y());
    pose.set_theta_z(bf.tool_pose_theta_z());

    return pose;
}

Kinova::Api::Base::Twist KortexRobot::GetCartesianTwist() const
{
    Kinova::Api::Base::Twist twist;

    if (!m_bIsConnected)
    {
        return twist;
    }
    Kinova::Api::BaseCyclic::BaseFeedback bf = m_Feedback.base();

    twist.set_linear_x(bf.tool_twist_linear_x());
    twist.set_linear_y(bf.tool_twist_linear_y());
    twist.set_linear_z(bf.tool_twist_linear_z());
    twist.set_angular_x(bf.tool_twist_angular_x());
    twist.set_angular_y(bf.tool_twist_angular_y());
    twist.set_angular_z(bf.tool_twist_angular_z());

    return twist;
}

Kinova::Api::Base::Wrench KortexRobot::GetCartesianWrench() const
{
        Kinova::Api::Base::Wrench wrench;

    if (!m_bIsConnected)
    {
        return wrench;
    }
    Kinova::Api::BaseCyclic::BaseFeedback bf = m_Feedback.base();

    wrench.set_force_x(bf.tool_external_wrench_force_x());
    wrench.set_force_y(bf.tool_external_wrench_force_y());
    wrench.set_force_z(bf.tool_external_wrench_force_z());
    wrench.set_torque_x(bf.tool_external_wrench_torque_x());
    wrench.set_torque_y(bf.tool_external_wrench_torque_y());
    wrench.set_torque_z(bf.tool_external_wrench_torque_z());

    return wrench;
}

std::vector<float> KortexRobot::GetJointPositions() const
{
    std::vector<float> positions;
    if (!m_bIsConnected)
    {
        return positions;
    }

    for (int idx = 0; idx < m_nNbDOF; ++idx)
    {
        auto position = m_Feedback.actuators(idx).position();
        if(position>180)
            position = position - 360;
        positions.push_back(position);
    }

    return positions;
}

std::vector<float> KortexRobot::GetJointVelocities() const
{
    std::vector<float> velocities;
    if (!m_bIsConnected)
    {
        return velocities;
    }

    for (int idx = 0; idx < m_nNbDOF; ++idx)
    {
        velocities.push_back(m_Feedback.actuators(idx).velocity());
    }

    return velocities;
}

std::vector<float> KortexRobot::GetJointTorques() const
{
    std::vector<float> torques;
    if (!m_bIsConnected)
    {
        return torques;
    }

    for (int idx = 0; idx < m_nNbDOF; idx++)
    {
        torques.push_back(m_Feedback.actuators(idx).torque());
    }

    return torques;
}

bool KortexRobot::SetCustomData(std::vector<float> data)
{
    if (data.size() == m_nNbDOF)
    {
        m_CustomData = data;
        return true;
    }
    return false;
}
