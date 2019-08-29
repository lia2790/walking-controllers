/**
 * @file WalkingImpedanceController.cpp
 * @authors Liana Bertoni <liana.bertoni@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

// YARP
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <WalkingImpedanceController.hpp>
#include <Utils.hpp>


bool WalkingImpedanceController::initialize(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if(config.isNull())
    {
        yError() << "[initializeImpedanceController] Empty configuration for Impedance controller.";
        return false;
    }

    // take stiffness and damping gains from conf file
    yarp::os::Value kstiff = config.find("kSTIFF");
    yarp::os::Value kdamp = config.find("kDAMP");
    yarp::os::Bottle *kstiffValue = kstiff.asList();
    yarp::os::Bottle *kdampValue = kdamp.asList();

    m_kSTIFF.resize(kstiffValue->size());
    m_kDAMP.resize(kdampValue->size());
    m_kSTIFF.zero();
    m_kDAMP.zero();

    m_isSetFeedbackSignals = false;
    m_isSetDesiredSignals = false;
    m_isSetGravityCompensationTerm = false;
    
    for(int i = 0; i < kstiffValue->size(); i++ )
    {
        m_kSTIFF(i) = kstiffValue->get(i).asDouble();
        m_kDAMP(i)  = kdampValue->get(i).asDouble();
    }

    /*
    std::cout<< "-- Impedance Controller --------------------- "<< std::endl;
    std::cout<< "-- kSTIFF : " << iDynTree::toEigen(m_kSTIFF) << std::endl;
    std::cout<< "-- kDAMP  : " << iDynTree::toEigen(m_kDAMP)  << std::endl;
    std::cout<< "------------------------------------- " << std::endl;
    */

    m_isInitialized = true;

    return true;
}

void WalkingImpedanceController::setFeedback(const iDynTree::VectorDynSize& jointPositionFeedback, const iDynTree::VectorDynSize& jointVelocityFeedback)
{
    m_jointPositionFeedback = jointPositionFeedback;
    m_jointVelocityFeedback = jointVelocityFeedback;

    m_isSetFeedbackSignals = true;
}

void WalkingImpedanceController::setReferenceSignal(const iDynTree::VectorDynSize& jointPositionDesired, const iDynTree::VectorDynSize& jointVelocityDesired)
{
    m_jointPositionDesired = jointPositionDesired;
    m_jointVelocityDesired = jointVelocityDesired;

    m_isSetDesiredSignals = true;
}

void  WalkingImpedanceController::setGravityCompensationTerm(const iDynTree::JointDOFsDoubleArray& jointGravityTorques)
{
    m_jointGravityTorques = jointGravityTorques;

    m_isSetGravityCompensationTerm = true;
}

bool WalkingImpedanceController::evaluateControl()
{
    if(!m_isInitialized)
    {
        yError() << "[evaluateImpedanceControl] The Impedance controller is not initialized. "
                 << "Please call 'initialize()'";
        return false;
    }

    if(!m_isSetFeedbackSignals)
    {
        yError() << "[evaluateImpedanceControl] Error : you have to set the feedback signals before to call it. "
                 << "Please call 'initialize()'";
        return false;
    } 

    if(!m_isSetDesiredSignals)
    {
        yError() << "[evaluateImpedanceControl] Error : you have to set the desired signals before to call it. "
                 << "Please call 'initialize()'";
        return false;
    } 

    if(!m_isSetGravityCompensationTerm)
    {
        yError() << "[evaluateImpedanceControl] Error : you have to set the gravity compensation term before to call it. "
                 << "Please call 'initialize()'";
        return false;
    }    

    iDynTree::VectorDynSize jointGravityCompensation; jointGravityCompensation.resize(m_jointGravityTorques.size());

    for(int i = 0; i < m_jointGravityTorques.size(); i++)
    {
        jointGravityCompensation(i) = m_jointGravityTorques.getVal(i);
    }

    m_controllerOutput.resize(m_jointPositionFeedback.size());

    for(int i = 0; i < m_jointPositionFeedback.size() ; i++)
    {
        // evaluating control law
        m_controllerOutput(i) = m_kSTIFF(i) * ( m_jointPositionDesired(i) - m_jointPositionFeedback(i) ) - m_kDAMP(i) * ( m_jointVelocityFeedback(i) ) + m_jointGravityTorques.getVal(i);
    }

    std::cout << " ------- ImpedanceController -------- : " << std::endl;
    std::cout << " 1 --------------  controllerOutput torque ---------------------- : " << std::endl; 
    std::cout << iDynTree::toEigen(m_controllerOutput) << std::endl;
    std::cout << " 2 --------------  jointPositionFeedback --------------------- : " << std::endl; 
    std::cout << iDynTree::toEigen(m_jointPositionFeedback) << std::endl;
    std::cout << " 3 --------------- jointPositionDesired  --------------------- : " << std::endl;
    std::cout << iDynTree::toEigen(m_jointPositionDesired) << std::endl;
    std::cout << " 4 --------------- jointVelocityFeedback --------------------- : " << std::endl;
    std::cout << iDynTree::toEigen(m_jointVelocityFeedback) << std::endl;
    std::cout << " 5 --------------- jointGravityCompensation -------------------- : " << std::endl;
    std::cout << iDynTree::toEigen(jointGravityCompensation) << std::endl;
    std::cout << " -------------------------------------- : " << std::endl; 
    std::cout << " ------- Size vector ---------------- : " << std::endl;
    std::cout << " 1 ------------------------------------ : " << std::endl; 
    std::cout << m_controllerOutput.size() << std::endl;
    std::cout << " 2 ------------------------------------ : " << std::endl; 
    std::cout << m_jointPositionFeedback.size() << std::endl;
    std::cout << " 3 ------------------------------------ : " << std::endl;
    std::cout << m_jointPositionDesired.size() << std::endl;
    std::cout << " 4 ------------------------------------ : " << std::endl;
    std::cout << m_jointVelocityFeedback.size() << std::endl;
    std::cout << " 5 ------------------------------------ : " << std::endl;
    std::cout << jointGravityCompensation.size() << std::endl;
    std::cout << " 6 stiff ------------------------------------ : " << std::endl;
    std::cout << m_kSTIFF.size() << std::endl;
    std::cout << " 7 damp ------------------------------------ : " << std::endl;
    std::cout << m_kDAMP.size() << std::endl;
    std::cout << " ------------------------------------ : " << std::endl;

    return true;
}

iDynTree::VectorDynSize WalkingImpedanceController::getControllerOutput() 
{
    return m_controllerOutput;
}
