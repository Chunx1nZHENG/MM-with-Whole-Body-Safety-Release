#pragma once

#include "string.h"
#include "vector"
#include <iostream>
#include <functional>

#include <SessionManager.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <ControlConfigClientRpc.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>


struct tCartesianVector
{
    float x,y,z;

    tCartesianVector(const float& X, const float& Y, const float& Z): x(X), y(Y), z(Z){};
    tCartesianVector operator+(const tCartesianVector v) const {return tCartesianVector(x+v.x, y+v.y, z+v.y);}
    float Norm() const {return sqrtf(x*x + y*y + z*z);}
    tCartesianVector Normalized() const
    {
        float norm = Norm();
        if (norm > 0.0f)
            return tCartesianVector(x/norm, y/norm, z/norm);
        else
            return tCartesianVector(0.0f, 0.0f, 0.0f);
    }
};


class KortexRobot
{
public:
    KortexRobot(const std::string& IP);
    ~KortexRobot();

    bool Init();
    void Disconnect();
    bool IsConnected() {return m_bIsConnected;}

    int                                         GetNbDoF() const {return m_nNbDOF;}
    Kinova::Api::BaseCyclic::Feedback           GetFeedback() const {return m_Feedback;}
    Kinova::Api::BaseCyclic::ActuatorFeedback   GetActuatorFeedback(const int actIdx) const;
    Kinova::Api::Base::Pose                     GetCartesianPose() const;
    Kinova::Api::Base::Twist                    GetCartesianTwist() const;
    Kinova::Api::Base::Wrench                   GetCartesianWrench() const;
    std::vector<float>                          GetJointPositions() const;
    std::vector<float>                          GetJointVelocities() const;
    std::vector<float>                          GetJointTorques() const;

    bool RefreshFeedback();
    bool ReadAndSetJointSpeeds(std::vector<float> &jointSpeeds);
    bool SetCustomData(std::vector<float> data);

    bool ExcuteExistingAction(const std::string& actionName, Kinova::Api::Base::RequestedActionType& actiontpye);
    bool MoveTo(const tCartesianVector position, const tCartesianVector orientation, const Kinova::Api::Base::CartesianTrajectoryConstraint& constraint);
    bool SetJointAngles(const std::vector<float>& angles, const Kinova::Api::Base::JointTrajectoryConstraint& constraints);
    bool SendTwistCommand(const tCartesianVector& translation, const tCartesianVector& rotation);
    bool SetTwistReferenceFrame(const Kinova::Api::Common::CartesianReferenceFrame& frame);
    bool SetJointSpeeds(std::vector<float>& jointSpeeds);
    Kinova::Api::Base::SequenceHandle CreateSequence(const Kinova::Api::Base::Sequence& sequence) {return m_pBase->CreateSequence(sequence);}
    bool PlaySequence(const Kinova::Api::Base::SequenceHandle& sequenceHandle);
    bool ExecuteAction(const Kinova::Api::Base::Action& action);
    bool WaitWhileRobotIsMoving(const int timeout);
    bool Stop();

    void SubscribeToNotifications();
    void UnsubscribeToNotifications();

protected:
    void OnError(Kinova::Api::KDetailedException& ex);
    void OnActionNotificationCallback(Kinova::Api::Base::ActionNotification notif);

    void OnRefreshFeedbackCallback(const Kinova::Api::Error& err, const Kinova::Api::BaseCyclic::Feedback& feedback);

protected:
    int                                                     m_nNbDOF;
    bool                                                    m_bIsBusy;
    Kinova::Api::Base::Action                               m_Action;
    Kinova::Api::ControlConfig::ControlConfigClient*        m_pControlConfigClient;

    std::string                                             m_sIP;
    bool                                                    m_bIsConnected;
    std::vector<Kinova::Api::Common::NotificationHandle>    m_NotificationHandleList;

    Kinova::Api::TransportClientTcp*                        m_pTcpClient;
    Kinova::Api::RouterClient*                              m_pRouterClient;
    Kinova::Api::SessionManager*                            m_pSessionManager;
    Kinova::Api::DeviceConfig::DeviceConfigClient*          m_pDeviceConfigClient;
    Kinova::Api::Base::BaseClient*                          m_pBase;

    Kinova::Api::TransportClientUdp*                        m_pUdpClient;
    Kinova::Api::RouterClient*                              m_pRouterClientRT;
    Kinova::Api::SessionManager*                            m_pSessionManagerRT;
    Kinova::Api::BaseCyclic::BaseCyclicClient*              m_pBaseCyclic;
    Kinova::Api::BaseCyclic::Feedback                       m_Feedback;
    
    Kinova::Api::BaseCyclic::Command                        base_command;
    std::vector<float>                                      commands;
    std::vector<float>                                      m_CustomData; // For the purpose of the example
};