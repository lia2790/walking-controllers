/**
 * @file WalkingPDFeedforwardController.hpp
 * @authors Liana Bertoni <liana.bertoni@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef WALKING_PDFEEDFORWARD_CONTROLLER_HPP
#define WALKING_PDFEEDFORWARD_CONTROLLER_HPP

// YARP
#include <yarp/os/Searchable.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>

/**
 * WalkingPDFeedForwardController class
 * implements the classical PD-Feedforward control law as follows:
 * fu = Kff * xff + Kp * (xd - x) + Kd * (dotxd - dotx),
 * where :
 * xff is the feedforward term 
 * Kff is the feedforward gain
 * Kp is the proportional positive gain
 * Kd is the derivative positive gain
 * xd is the desired position vector
 * x  is the actual/measure/feedback position vector
 * dotxd is the desired velocity vector
 * dotx is the actual/measure/feedback velocity vector
 */
class WalkingPDFeedForwardController
{
    bool m_isInitialized{false};
    bool m_isSetFeedbackSignal{false};
    bool m_isSetDesiredSignal{false};
    bool m_isSetFeedForward{false};

    iDynTree::VectorDynSize m_positionFeedback; /**< position feedback vector. */
    iDynTree::VectorDynSize m_velocityFeedback; /**< velocity feedback vector. */
    iDynTree::VectorDynSize m_positionDesired; /**< position desired vector. */
    iDynTree::VectorDynSize m_velocityDesired; /**< velocoty desired vector. */

    iDynTree::VectorDynSize m_Kp; /**< proportional gain vector. */
    iDynTree::VectorDynSize m_Kd; /**< derivative gain vector. */
    iDynTree::VectorDynSize m_Kff; /**< feedforward gain vector. */

    iDynTree::VectorDynSize m_feedForward; /**< Feedforward term.*/

    iDynTree::VectorDynSize m_controllerOutput; /**< Controller output vector. */

public:

    /**
     * Initialize the method
     * @param config yarp searchable configuration variable.
     * @return true in case of success, failure otherwise.
     */
    bool initialize(const yarp::os::Searchable& config);

    /**
     * Set the feedback signals : position and velocity vectors
     * @param positionFeedback are the feedback position vector values;
     * @param velocityFeedback are the feedback velocity vector values;
     * @return true in case of success or false otherwise.
     */
    bool setFeedbackSignals(const iDynTree::VectorDynSize& positionFeedback, const iDynTree::VectorDynSize& velocityFeedback);

    /**
     * Set the desired signals : position and velocity vectors
     * @param positionDesired are the desired position vector values;
     * @param velocityDesired are the desired velocity vector values;
     * @return true in case of success or false otherwise.
     */
    bool setDesiredSignals(const iDynTree::VectorDynSize& positionDesired, const iDynTree::VectorDynSize& velocityDesired);

    /**
     * Set the feedforward term.
     * @param feedForward is the feedforward term.
     * @return true in case of success or false otherwise.
     */
    bool setFeedForwardSignal(const iDynTree::VectorDynSize& feedForward);

    /**
     * Evaluate the control output
     * @return true in case of success, failure otherwise.
     */
    bool evaluateControl();

    /**
     * Get the controller output
     * @return the output of the controller.
     */
    iDynTree::VectorDynSize getControllerOutput();
};

#endif
