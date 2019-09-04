/**
 * @file WalkingPDFeedforwardController.cpp
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

#include <WalkingPDFeedForwardController.hpp>
#include <Utils.hpp>


bool WalkingPDFeedForwardController::initialize(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if(config.isNull())
    {
        yError() << "[initializePDFeedForwardController] Empty configuration for PDFeedForward controller.";
        return false;
    }

    // stiffness and damping gains from conf file
    yarp::os::Value Kp = config.find("Kp");
    yarp::os::Value Kd = config.find("Kd");
    yarp::os::Value Kff = config.find("Kff");
    yarp::os::Bottle *KpValue = Kp.asList();
    yarp::os::Bottle *KdValue = Kd.asList();
    yarp::os::Bottle *KffValue = Kff.asList();

    m_Kp.resize(KpValue->size());
    m_Kd.resize(KdValue->size());
    m_Kff.resize(KffValue->size());
    m_Kp.zero();
    m_Kd.zero();
    m_Kff.zero();

    for(int i = 0; i < KpValue->size(); i++ )
    {
        m_Kp(i) = KpValue->get(i).asDouble();
        m_Kd(i) = KdValue->get(i).asDouble();
        m_Kff(i) = KffValue->get(i).asDouble();
    }

    m_isInitialized = true;
    m_isSetFeedbackSignal = false;
    m_isSetDesiredSignal = false;
    m_isSetFeedForward = false;

    return true;
}

bool WalkingPDFeedForwardController::setFeedbackSignals(const iDynTree::VectorDynSize& positionFeedback, const iDynTree::VectorDynSize& velocityFeedback)
{
    m_positionFeedback = positionFeedback;
    m_velocityFeedback = velocityFeedback;

    m_isSetFeedbackSignal = true;

    return true;
}

bool WalkingPDFeedForwardController::setDesiredSignals(const iDynTree::VectorDynSize& positionDesired, const iDynTree::VectorDynSize& velocityDesired)
{
    m_positionDesired = positionDesired;
    m_velocityDesired = velocityDesired;

    m_isSetDesiredSignal = true;

    return true;
}

bool  WalkingPDFeedForwardController::setFeedForwardSignal(const iDynTree::VectorDynSize& feedForward)
{
    m_feedForward = feedForward;

    m_isSetFeedForward = true;

    return true;
}

bool WalkingPDFeedForwardController::evaluateControl()
{
    if(!m_isInitialized)
    {
        yError() << "[evaluatePDFeedforwardControl] The PDFeedforward controller is not initialized. "
                 << "Please call 'initialize()'";
        return false;
    }

    if(!m_isSetFeedbackSignal)
    {
        yError() << "[evaluatePDFeedforwardControl] Error : you have to set the feedback signal before to call it. "
                 << "Please call the function setFeedbackSignals()' ";
        return false;
    }

    if(!m_isSetDesiredSignal)
    {
        yError() << "[evaluatePDFeedforwardControl] Error : you have to set the desired signal before to call it. "
                 << "Please call the function setDesiredSignals()' ";
        return false;
    }

    if(!m_isSetFeedForward)
    {
        yError() << "[evaluatePDFeedforwardControl] Error : you have to set the feedforward before to call it. "
                 << "Please call the function setFeedForward()' ";
        return false;
    }

    m_controllerOutput.resize(m_feedForward.size()); 
    m_controllerOutput.zero();

    for( int i = 0; i < m_controllerOutput.size(); i++)
    {
        m_controllerOutput(i) = m_Kff(i) * m_feedForward(i) + m_Kp(i) * (m_positionDesired(i) - m_positionFeedback(i)) + m_Kd(i) * (m_velocityDesired(i) - m_velocityFeedback(i));
    }

    std::cout << " ------- PDFeedForwardController -------- : " << std::endl;
    std::cout << " 1 --------------  controllerOutput ---------------------- : " << std::endl; 
    std::cout << iDynTree::toEigen(m_controllerOutput) << std::endl;
    std::cout << " 2 --------------  PositionFeedback ---------------------- : " << std::endl; 
    std::cout << iDynTree::toEigen(m_positionFeedback) << std::endl;
    std::cout << " 3 --------------- PositionDesired  ---------------------- : " << std::endl;
    std::cout << iDynTree::toEigen(m_positionDesired)  << std::endl;
    std::cout << " 4 --------------- velocityFeedback ---------------------- : " << std::endl;
    std::cout << iDynTree::toEigen(m_velocityFeedback) << std::endl;
    std::cout << " 5 --------------- velocityDesired ----------------------- : " << std::endl;
    std::cout << iDynTree::toEigen(m_velocityDesired)  << std::endl;
    std::cout << " 6 --------------- feedForward --------------------------- : " << std::endl;
    std::cout << iDynTree::toEigen(m_feedForward) << std::endl;
    std::cout << " ----------------- Kp gain ------------------------------- : " << std::endl;
    std::cout << iDynTree::toEigen(m_Kp) << std::endl;
    std::cout << " ----------------- Kd gain ------------------------------- : " << std::endl;
    std::cout << iDynTree::toEigen(m_Kd) << std::endl;
    std::cout << " ----------------- Kff gain ------------------------------ : " << std::endl;
    std::cout << iDynTree::toEigen(m_Kff) << std::endl;
    std::cout << " ------- Size vectors ------------------------------------ : " << std::endl;
    std::cout << " 1 ------------------------------------ : " << std::endl; 
    std::cout << m_controllerOutput.size() << std::endl;
    std::cout << " 2 ------------------------------------ : " << std::endl; 
    std::cout << m_positionFeedback.size() << std::endl;
    std::cout << " 3 ------------------------------------ : " << std::endl;
    std::cout << m_positionDesired.size() << std::endl;
    std::cout << " 4 ------------------------------------ : " << std::endl;
    std::cout << m_velocityFeedback.size() << std::endl;
    std::cout << " 5 ------------------------------------ : " << std::endl;
    std::cout << m_velocityDesired.size() << std::endl;
    std::cout << " 7 ------------------------------------ : " << std::endl;
    std::cout << m_feedForward.size() << std::endl;
    std::cout << " Kp ----------------------------------- : " << std::endl;
    std::cout << m_Kp.size() << std::endl;
    std::cout << " Kd ----------------------------------- : " << std::endl;
    std::cout << m_Kd.size() << std::endl;
    std::cout << " Kff ---------------------------------- : " << std::endl;
    std::cout << m_Kff.size() << std::endl;
    std::cout << " -------------------------------------- : " << std::endl;

    return true;
}

iDynTree::VectorDynSize WalkingPDFeedForwardController::getControllerOutput() 
{
    return m_controllerOutput;
}
