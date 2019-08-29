/**
 * @file WalkingImpedanceController.hpp
 * @authors Liana Bertoni <liana.bertoni@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef WALKING_IMPEDANCE_CONTROLLER_HPP
#define WALKING_IMPEDANCE_CONTROLLER_HPP

// YARP
#include <yarp/os/Searchable.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>

/**
 * WalkingImpedanceController class
 * implements the impedance controll to handle the contacts
 * as a spring damper system following this control law with gravity compensation:
 * tau = K * (qd - q) + D * (dotqd - dotq) + g,
 * where :
 * K and D are the stiffness and damper gain matrix
 * q are the joint values
 * qd are the joint desired values
 * dotq are the joint velocity values
 * dotqd are the joint desired velocity values
 */
class WalkingImpedanceController
{
    bool  m_isInitialized{false};
    bool  m_isSetFeedbackSignals{false};
    bool  m_isSetDesiredSignals{false};
    bool  m_isSetGravityCompensationTerm{false};

    iDynTree::VectorDynSize m_jointPositionFeedback; /**< Joint position Feedback. */
    iDynTree::VectorDynSize m_jointPositionDesired; /**< Desired joint position. */
    iDynTree::VectorDynSize m_jointVelocityFeedback; /**< Joint Velocity Feedback. */
    iDynTree::VectorDynSize m_jointVelocityDesired; /**< Desired joint velocity. */

    iDynTree::VectorDynSize m_kSTIFF; /**< Controller gain, stiffness vector. */
    iDynTree::VectorDynSize m_kDAMP; /**< Controller gain, damper vector. */

    iDynTree::JointDOFsDoubleArray m_jointGravityTorques; /**< Joint gravity torques vector. */

    iDynTree::VectorDynSize m_controllerOutput; /**< Controller output. */

public:

    /**
     * Initialize the method
     * @param config yarp searchable configuration variable.
     * @return true in case of success, failure otherwise.
     */
    bool initialize(const yarp::os::Searchable& config);

    /**
     * Set the feedback variables : joint position and velocity.
     * @param jointPositionFeedback are the joint positions of the robot;
     * @param jointVelocityFeedback are the joint velocities of the robot;
     */
    void setFeedback(const iDynTree::VectorDynSize& jointPositionFeedback, const iDynTree::VectorDynSize& jointVelocityFeedback);

    /**
     * Set the desired variable : joint position and velocity.
     * @param jointPositionDesired are the joint positions of the robot;
     * @param jointVelocityDesired are the joint velocities of the robot;
     */
    void setReferenceSignal(const iDynTree::VectorDynSize& jointPositionDesired, const iDynTree::VectorDynSize& jointVelocityDesired);

     /**
     * Set the joint gravity torques.
     * @param jointGravityTorques sets Set the gravity compensation term;
     */
    void setGravityCompensationTerm(const iDynTree::JointDOFsDoubleArray& jointGravityTorques);

    /**
     * Evaluate the control output.
     * @return true in case of success, failure otherwise.
     */
    bool evaluateControl();

    /**
     * Get the controller output.
     * @return the output of the controller.
     */
    iDynTree::VectorDynSize getControllerOutput();
};

#endif
