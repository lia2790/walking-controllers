/**
 * @file WalkingGTorqueController.hpp
 * @authors Liana Bertoni <liana.bertoni@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef WALKING_GTORQUE_CONTROLLER_HPP
#define WALKING_GTORQUE_CONTROLLER_HPP

// YARP
#include <yarp/os/Searchable.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>

/**
 * WalkingGTorqueController class
 * implements the gravity compensation control law as follows:
 * tau = J'*(fug) - Knl*C,
 * where :
 * J is the Jacobian Matrix from CoM to contact point/s, J' is the transpose one. 
 * fug is/are the force input/s,
 * Knl is a gain vector,
 * C is non linear compensation term.
 */
class WalkingGTorqueController
{
    bool  m_isInitialized{false};
    bool  m_isSetParams{false};

    iDynTree::VectorDynSize m_inputForce; /**< input force vector. */
    iDynTree::MatrixDynSize m_comToContactJacobian; /**< jacobian matrix from the com to the contact point/s. */
    iDynTree::VectorDynSize m_jointVelocity; /**< non linear compensation term. */
    iDynTree::VectorDynSize m_KD; /**< gain vector for the non linear compensation term. */
    iDynTree::VectorDynSize m_controllerOutput; /**< Controller output vector. */

public:

    /**
     * Initialize the method
     * @return true in case of success, failure otherwise.
     */
    bool initialize(const yarp::os::Searchable& config);

    /**
     * Set the elements of the control law
     * @param comToContactJacobian is the Jacobian Matrix from CoM to contact point/s;
     * @param inpuForce is the input force vector;
     * @param jointVelocity are the joint velocities;
     * @return true in case of success or false otherwise.
     */
    bool setParams(const iDynTree::MatrixDynSize& comToContactJacobian, const iDynTree::VectorDynSize& inputForce, const iDynTree::VectorDynSize& jointVelocity);

    /**
     * Evaluate the controller output
     * @return true in case of success, failure otherwise.
     */
    bool evaluateControl();

    /**
     * Print on the terminal controller output
     */
    void printOutput();

    /**
     * Get the controller output
     * @return the output of the controller.
     */
    iDynTree::VectorDynSize getControllerOutput();
};

#endif
