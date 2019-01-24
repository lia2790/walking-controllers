/**
 * @file WalkingDCMReactiveController.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_DCM_REACTIVE_CONTROLLER_HPP
#define WALKING_DCM_REACTIVE_CONTROLLER_HPP


// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>

// iCub-ctrl
#include <iCub/ctrl/pids.h>

/**
 * WalkingDCMReactiveController class implements the reactive
 * control for the DCM traking .
 * u = dcm - 1/omega * (Ddcm_des + kDCM * (dcm_des - dcm))
 */
class WalkingDCMReactiveController
{
    double m_kDCM; /**< Controller gain. */
    double m_kIDCM; /**< Controller gain. */
    double m_omega; /**< LIPM time constant. */
    bool m_controlEvaluated{false}; /**< True if the control output was correctly evaluated. */

    iDynTree::Vector3 m_dcmFeedback; /**< Feedback signal containing the position of the CoM. */
    iDynTree::Vector3 m_dcmPositionDesired; /**< Desired CoM position. */
    iDynTree::Vector3 m_dcmVelocityDesired; /**< Desired CoM velocity. */

    iDynTree::Vector3 m_controllerOutput; /**< Controller output. */

    std::unique_ptr<iCub::ctrl::Integrator> m_dcmErrorIntegral; /**< left foot error integrator */

public:

    /**
     * Initialize the method
     * @param config yarp searchable configuration variable.
     * @return true/false in case of success/failure
     */
    bool initialize(const yarp::os::Searchable& config);

    /**
     * Set the feedback.
     * @param dcmFeedback is position of the robot DCM;
     */
    void setFeedback(const iDynTree::Vector3& dcmFeedback);

    /**
     * Set the desired reference signals.
     * @param dcmPositionDesired is the desired position of the DCM;
     * @param dcmVelocityDesired is the desired velocity of the DCM.
     */
    void setReferenceSignal(const iDynTree::Vector3& dcmPositionDesired,
                            const iDynTree::Vector3& dcmVelocityDesired);

    /**
     * Evaluate the control output.
     * @return true/false in case of success/failure
     */
    bool evaluateControl();

    /**
     * Get the controller output.
     * @param controllerOutput is the output of the controller.
     * @return true/false in case of success/failure
     */
    const iDynTree::Vector3& getControllerOutput() const;

    /**
     * Evaluate the integral.
     * @param integral object;
     * @param error;
     * @return the integral position.
     */
    iDynTree::VectorDynSize evaluateIntegralError(std::unique_ptr<iCub::ctrl::Integrator>& integral,
                                                  const iDynTree::Vector3& error);

};

#endif
