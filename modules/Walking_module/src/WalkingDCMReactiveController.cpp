/**
 * @file WalkingDCMReactiveController.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <WalkingDCMReactiveController.hpp>
#include <Utils.hpp>


bool WalkingDCMReactiveController::initialize(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for DCM controller.";
        return false;
    }

    // set the gain of the DCM controller
    if(!YarpHelper::getNumberFromSearchable(config, "kDCM", m_kDCM))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "kIDCM", m_kIDCM))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "com_height", m_comHeight))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    double inclPlaneAngle = 7.0;

    m_omega = sqrt(9.81 * std::cos(iDynTree::deg2rad(inclPlaneAngle)) / m_comHeight * std::cos(iDynTree::deg2rad(inclPlaneAngle)));

    yarp::sig::Vector buffer(3, 0.0);
    m_dcmErrorIntegral = std::make_unique<iCub::ctrl::Integrator>(0.01, buffer);

    return true;
}

void WalkingDCMReactiveController::setFeedback(const iDynTree::Vector3& dcmFeedback)
{
    m_dcmFeedback = dcmFeedback;
}

void WalkingDCMReactiveController::setReferenceSignal(const iDynTree::Vector3& dcmPositionDesired,
                                                      const iDynTree::Vector3& dcmVelocityDesired)
{
    m_dcmPositionDesired = dcmPositionDesired;
    m_dcmVelocityDesired = dcmVelocityDesired;
}

bool WalkingDCMReactiveController::setOmega(double inclPlaneAngle)
{
    m_omega = sqrt((9.81 * std::cos(iDynTree::deg2rad(inclPlaneAngle))) / (m_comHeight*std::cos(iDynTree::deg2rad(inclPlaneAngle))));

    return true;
}

bool WalkingDCMReactiveController::evaluateControl()
{
    if(!m_dcmErrorIntegral)
    {
        yError() << "[evaluateControl] The controller is not initialized. "
                 << "Please call 'initialize()'";
        return false;
    }

    iDynTree::Vector3 error;
    iDynTree::toEigen(error) = iDynTree::toEigen(m_dcmPositionDesired) -
      iDynTree::toEigen(m_dcmFeedback);
    // evaluate the control law
    iDynTree::toEigen(m_controllerOutput) = iDynTree::toEigen(m_dcmPositionDesired) -
      1 / m_omega * (iDynTree::toEigen(m_dcmVelocityDesired)) -
      m_kDCM * iDynTree::toEigen(error)  +
      m_kIDCM * iDynTree::toEigen(evaluateIntegralError(m_dcmErrorIntegral, error));

    return true;
}

const iDynTree::Vector3& WalkingDCMReactiveController::getControllerOutput() const
{
    return m_controllerOutput;
}

iDynTree::VectorDynSize WalkingDCMReactiveController::evaluateIntegralError(std::unique_ptr<iCub::ctrl::Integrator>& integral,
                                                                            const iDynTree::Vector3& error)
{
    yarp::sig::Vector buffer(error.size());
    yarp::sig::Vector integralErrorYarp(error.size());
    iDynTree::toYarp(error, buffer);
    integralErrorYarp = integral->integrate(buffer);

    // transform into iDynTree Vector
    iDynTree::VectorDynSize integralError(3);
    iDynTree::toiDynTree(integralErrorYarp, integralError);

    return integralError;
}
