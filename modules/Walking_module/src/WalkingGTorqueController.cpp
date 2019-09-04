/**
 * @file WalkingGTorqueController.cpp
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

#include <WalkingGTorqueController.hpp>
#include <Utils.hpp>


bool WalkingGTorqueController::initialize(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if(config.isNull())
    {
        yError() << "[initializeTorqueController] Empty configuration for Torque controller.";
        return false;
    }

    // damping gain from conf file
    yarp::os::Value KD = config.find("D");
    yarp::os::Bottle *KDValue = KD.asList();
    m_KD.resize(KDValue->size());
    m_KD.zero();

    for(int i = 0; i < KDValue->size(); i++ )
        m_KD(i) = KDValue->get(i).asDouble();

    m_controllerOutput.resize(m_KD.size());
    m_controllerOutput.zero();

    m_isInitialized = true;
    m_isSetParams = false;

    return true;
}

bool WalkingGTorqueController::setParams(const iDynTree::MatrixDynSize& comToContactJacobian, const iDynTree::VectorDynSize& inputForce, const iDynTree::VectorDynSize& jointVelocity)
{
    m_inputForce = inputForce;
    m_comToContactJacobian = comToContactJacobian;
    m_jointVelocity = jointVelocity;

    m_isSetParams = true;

    return true;
}

bool WalkingGTorqueController::evaluateControl()
{
    if(!m_isInitialized)
    {
        yError() << "[evaluateTorqueControl] The Torque controller is not initialized. "
                 << "Please call 'initialize()' before";
        return false;
    }

    if(!m_isSetParams)
    {
        yError() << "[evaluateTorqueControl] Error : you have to set the params before to call it. "
                 << "Please call 'setParams()' before";
        return false;
    }

    for( int i = 0; i < m_comToContactJacobian.cols(); i++)
    {
        double Jf_i = 0;

        for( int j = 0; j < m_comToContactJacobian.rows(); j++)
        {
            Jf_i += m_comToContactJacobian(j,i) * m_inputForce(j);
            std::cout << " ------- j -------- : " << j << std::endl;
        }
        m_controllerOutput(i) = Jf_i - m_KD(i) * m_jointVelocity(i);
        std::cout << " ------- i -------- : " << i << std::endl;
    }

    printOutput();

    return true;
}

void WalkingGTorqueController::printOutput()
{
    std::cout << " ------- TorqueController output -------- : " << std::endl;
    std::cout << iDynTree::toEigen(m_controllerOutput)          << std::endl;
    std::cout << " ---------------------------------------- : " << std::endl;
}

iDynTree::VectorDynSize WalkingGTorqueController::getControllerOutput() 
{
    return m_controllerOutput;
}
