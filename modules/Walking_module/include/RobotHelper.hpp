/**
 * @file RobotHelper.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef ROBOT_HELPER_HPP
#define ROBOT_HELPER_HPP

// std
#include <memory>
#include <vector>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Timer.h>

#include <iCub/ctrl/filters.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>

#include <WalkingPIDHandler.hpp>

class RobotHelper
{
    double m_dT;
    yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */
    std::vector<std::string> m_axesList; /**< Vector containing the name of the controlled joints. */
    unsigned int m_actuatedDOFs; /**< Number of the actuated DoFs. */
    int m_controlMode{-1};

    // YARP Interfaces exposed by the remotecontrolboardremapper
    yarp::dev::IEncodersTimed *m_encodersInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IPositionDirect *m_positionDirectInterface{nullptr}; /**< Direct position control interface. */
    yarp::dev::IPositionControl *m_positionInterface{nullptr}; /**< Position control interface. */
    yarp::dev::IVelocityControl *m_velocityInterface{nullptr}; /**< Velocity control interface. */
    yarp::dev::ITorqueControl *m_torqueInterface{nullptr}; /**< Torque control interface. */
    yarp::dev::IControlMode *m_controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IControlLimits *m_limitsInterface{nullptr}; /**< Encorders interface. */

    std::unique_ptr<WalkingPIDHandler> m_PIDHandler; /**< Pointer to the PID handler object. */

    yarp::os::Bottle m_remoteControlBoards; /**< Contain all the name of the controlled joints. */

    double m_positioningTime;

    yarp::sig::Vector m_positionFeedbackDeg; /**< Current joint position [deg]. */
    yarp::sig::Vector m_velocityFeedbackDeg; /**< Current joint velocity [deg/s]. */
    iDynTree::VectorDynSize m_positionFeedbackRad; /**< Current joint position [rad]. */
    iDynTree::VectorDynSize m_positionFeedbackRadPrevious; /**< Previous joint position [rad]. */
    iDynTree::VectorDynSize m_velocityFeedbackRad; /**< Current joint velocity [rad/s]. */
    iDynTree::VectorDynSize m_torqueFeedback; /**< Current joint torque [Nm]. */

    iDynTree::VectorDynSize m_desiredJointPositionRad; /**< Desired Joint Position [rad]. */
    iDynTree::VectorDynSize m_desiredJointValueDeg; /**< Desired joint position or velocity [deg or deg/s]. */

    iDynTree::VectorDynSize m_jointsVelocityLimit; /**< JointVeloc [rad/s]. */
    iDynTree::VectorDynSize m_jointsAngleLowerLimit; /**< JointVeloc [rad/s]. */
    iDynTree::VectorDynSize m_jointsAngleUpperLimit; /**< JointVeloc [rad/s]. */

    // yarp::sig::Vector m_positionFeedbackDegFiltered;
    yarp::sig::Vector m_velocityFeedbackDegFiltered; /**< Vector containing the filtered joint velocity [deg/s]. */
    std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_positionFilter; /**< Joint position low pass filter .*/
    std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_velocityFilter; /**< Joint velocity low pass filter .*/
    bool m_useVelocityFilter; /**< True if the joint velocity filter is used. */
    bool m_manuallyEvaluateVelocity;

    yarp::os::BufferedPort<yarp::sig::Vector> m_leftWrenchPort; /**< Left foot wrench port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_rightWrenchPort; /**< Right foot wrench port. */
    yarp::sig::Vector m_leftWrenchInput; /**< YARP vector that contains left foot wrench. */
    yarp::sig::Vector m_rightWrenchInput; /**< YARP vector that contains right foot wrench. */
    yarp::sig::Vector m_leftWrenchInputFiltered; /**< YARP vector that contains left foot filtered wrench. */
    yarp::sig::Vector m_rightWrenchInputFiltered; /**< YARP vector that contains right foot filtered wrench. */
    iDynTree::Wrench m_leftWrench; /**< iDynTree vector that contains left foot wrench. */
    iDynTree::Wrench m_rightWrench; /**< iDynTree vector that contains right foot wrench. */
    std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_leftWrenchFilter; /**< Left wrench low pass filter.*/
    std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_rightWrenchFilter; /**< Right wrench low pass filter.*/
    bool m_useWrenchFilter; /**< True if the wrench filter is used. */

    double m_startingPositionControlTime;
    bool m_positionMoveSkipped;

    bool m_useExternalRobotBase;
    iDynTree::Transform m_robotBaseTransform;
    iDynTree::Twist m_robotBaseTwist;
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotBasePort; /**< Right foot wrench port. */
    double m_heightOffset;

    // contact
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotContactFootPort; /**< contact Foot port. */
    bool m_contactFoot; /**< booalean to identify which foot is in contact : 0 left foot - 1 right foot. */

    /**
     * Get the higher position error among all joints.
     * @param desiredJointPositionsRad desired joint position in radiants;
     * @param worstError is a pair containing the indices of the joint with the
     * worst error and its value.
     * @return true in case of success and false otherwise.
     */
    bool getWorstError(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                       std::pair<std::string, double>& worstError);
public:

    /**
     * Configure the Robot.
     * @param config is the reference to a resource finder object.
     * @param name robot name
     * @return true in case of success and false otherwise.
     */
    bool configureRobot(const yarp::os::Searchable& rf);

    /**
     * Configure the Force torque sensors. The FT ports are only opened please use yarpamanger
     * to connect them.
     * @param config is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configureForceTorqueSensors(const yarp::os::Searchable& config);

    bool configurePIDHandler(const yarp::os::Bottle& config);

    /**
     * Get all the feedback signal from the interfaces
     * @return true in case of success and false otherwise.
     */
    bool getFeedbacks(unsigned int maxAttempts = 1);

    bool getFeedbacksRaw( bool useBaseEst, unsigned int maxAttempts = 1);

    /**
     * Switch the control mode.
     * @param controlMode is the control mode.
     * @return true in case of success and false otherwise.
     */
    bool switchToControlMode(const int& controlMode);

    /**
     * Set the desired position reference. (The position will be sent using PositionControl mode)
     * @param jointPositionsRadians desired final joint position;
     * @param positioningTimeSec minimum jerk trajectory duration.
     * @return true in case of success and false otherwise.
     */
    bool setPositionReferences(const iDynTree::VectorDynSize& jointPositionsRadians,
                               const double& positioningTimeSec);

    bool checkMotionDone(bool& motionDone);
    /**
     * Set the desired position reference.
     * (The position will be sent using DirectPositionControl mode)
     * @param desiredPositionsRad desired final joint position;
     * @return true in case of success and false otherwise.
     */
    bool setDirectPositionReferences(const iDynTree::VectorDynSize&  desiredPositionsRad);

    /**
     * Set the desired velocity reference.
     * @param desiredVelocityRad desired joints velocity;
     * @return true in case of success and false otherwise.
     */
    bool setVelocityReferences(const iDynTree::VectorDynSize& desiredVelocityRad);

    /**
     * Set the desired torque reference.
     * @param desiredTorque desired joints torque;
     * @return true in case of success and false otherwise.
     */
    bool setTorqueReferences(const iDynTree::VectorDynSize& desiredTorque);

    void setHeightOffset(const double& offset);

    bool resetFilters();

    bool close();

    const iDynTree::VectorDynSize& getJointPosition() const;
    const iDynTree::VectorDynSize& getJointVelocity() const;
    const iDynTree::VectorDynSize& getJointTorque() const;

    const iDynTree::VectorDynSize& getVelocityLimits() const;

    const iDynTree::VectorDynSize& getJointAngleUpperLimits() const;
    const iDynTree::VectorDynSize& getJointAngleLowerLimits() const;

    const iDynTree::Wrench& getLeftWrench() const;
    const iDynTree::Wrench& getRightWrench() const;

    const std::vector<std::string>& getAxesList() const;

    int getActuatedDoFs();

    const iDynTree::Transform& getBaseTransform() const;

    const iDynTree::Twist&getBaseTwist() const;

    bool isExternalRobotBaseUsed();
    bool getContactFoot();

    WalkingPIDHandler& getPIDHandler();
};

#endif
