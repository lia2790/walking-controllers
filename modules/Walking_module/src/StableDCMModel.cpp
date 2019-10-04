/**
 * @file StableDCMModel.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <math.h>

// YARP
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>

//iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <StableDCMModel.hpp>
#include <Utils.hpp>

bool StableDCMModel::initialize(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for StableDCMModel.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "com_height", m_comHeight))
    {
        yError() << "[initialize] Unable to get a double from a searchable.";
        return false;
    }
    double samplingTime;
    if(!YarpHelper::getNumberFromSearchable(config, "sampling_time", samplingTime))
    {
        yError() << "[initialize] Unable to get a double from a searchable.";
        return false;
    }
    // instantiate Integrator object
    yarp::sig::Vector buffer;
    buffer.resize(2, 0.0);
    m_comIntegrator = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buffer);

    m_inclPlaneAngle = 0.0;
    m_omega = sqrt((9.81*std::cos(iDynTree::deg2rad(m_inclPlaneAngle))) / (m_comHeight * std::cos(iDynTree::deg2rad(m_inclPlaneAngle))));
    
    m_lipCorrTerm(0) = - 9.81 * std::sin(iDynTree::deg2rad(m_inclPlaneAngle));
    m_lipCorrTerm(1) = 0;

    m_dcmCorrTerm(0) = - m_comHeight * std::tan(iDynTree::deg2rad(m_inclPlaneAngle));
    m_dcmCorrTerm(1) = 0;

    return true;
}

void StableDCMModel::setDCMPosition(const iDynTree::Vector2& input)
{
    m_dcmPosition = input;
}

void StableDCMModel::setZMPPosition(const iDynTree::Vector2& input)
{
    m_zmpPosition = input;
}

bool StableDCMModel::setStableDCMModel(double inclPlaneAngle, double yawAngle)
{
    m_omega = sqrt((9.81 * std::cos(iDynTree::deg2rad(inclPlaneAngle))) / (m_comHeight * std::cos(iDynTree::deg2rad(inclPlaneAngle))));

    m_lipCorrTerm(0) = - 9.81 * std::sin(iDynTree::deg2rad(inclPlaneAngle));
    m_lipCorrTerm(1) = 0;

    m_dcmCorrTerm(0) = - m_comHeight * std::sin(iDynTree::deg2rad(inclPlaneAngle));
    m_dcmCorrTerm(1) = 0;

    std::cout << "setStableDCMModel inclPlaneAngle " << inclPlaneAngle << std::endl;

    return true;
}

bool StableDCMModel::integrateModel()
{
    m_isModelPropagated = false;

    if(m_comIntegrator == nullptr)
    {
        yError() << "[integrateModel] The dcm integrator object is not ready. "
                 << "Please call initialize method.";
        return false;
    }

    // evaluate the acceleration of the CoM
    yarp::sig::Vector comAccelerationYarp(2);
    iDynTree::toEigen(comAccelerationYarp) = std::pow(m_omega,2) * (iDynTree::toEigen(m_comPosition) - iDynTree::toEigen(m_zmpPosition)) + iDynTree::toEigen(m_lipCorrTerm);
                                                                    

    // evaluate the velocity of the CoM
    yarp::sig::Vector comVelocityYarp(2);
    iDynTree::toEigen(comVelocityYarp) = - m_omega * ((iDynTree::toEigen(m_comPosition) + iDynTree::toEigen(m_dcmCorrTerm)) -
                                                     iDynTree::toEigen(m_dcmPosition));

        std::cout << " STABLE DCM  integrateModel() comVelocityYarp " << comVelocityYarp.toString() << std::endl;

        std::cout << " STABLE DCM  integrateModel() m_comPosition " << m_comPosition.toString() << std::endl;

        std::cout << " STABLE DCM  integrateModel() m_dcmCorrTerm " << m_dcmCorrTerm.toString() << std::endl;

        std::cout << " STABLE DCM  integrateModel() m_dcmPosition " << m_dcmPosition.toString() << std::endl;


    std::cout<< " m_lipCorrTerm " << m_lipCorrTerm.toString() << std::endl;

    // integrate velocities
    yarp::sig::Vector comPositionYarp(2);
    comPositionYarp = m_comIntegrator->integrate(comVelocityYarp);

    // convert YARP vector into iDynTree vector
    iDynTree::toiDynTree(comAccelerationYarp, m_comAcceleration);
    iDynTree::toiDynTree(comVelocityYarp, m_comVelocity);
    iDynTree::toiDynTree(comPositionYarp, m_comPosition);

    m_isModelPropagated = true;

    return true;
}

bool StableDCMModel::getCoMPosition(iDynTree::Vector2& comPosition)
{
    if(!m_isModelPropagated)
    {
        yError() << "[getCoMPosition] The Model is not prrpagated. "
                 << "Please call 'propagateModel()' method.";
        return false;
    }
    comPosition = m_comPosition;
    return true;
}

bool StableDCMModel::getCoMVelocity(iDynTree::Vector2& comVelocity)
{
    if(!m_isModelPropagated)
    {
        yError() << "[getCoMPosition] The Model is not prrpagated. "
                 << "Please call 'propagateModel()' method.";
        return false;
    }
    comVelocity = m_comVelocity;
    return true;
}

bool StableDCMModel::getCoMAcceleration(iDynTree::Vector2& comAcceleration)
{
    if(!m_isModelPropagated)
    {
        yError() << "[getCoMAcceleration] The Model is not prrpagated. "
                 << "Please call 'propagateModel()' method.";
        return false;
    }
    comAcceleration = m_comAcceleration;
    return true;
}

bool StableDCMModel::reset(const iDynTree::Vector2& initialValue)
{
    if(m_comIntegrator == nullptr)
    {
        yError() << "[reset] The dcm integrator object is not ready. "
                 << "Please call initialize method.";
        return false;
    }

    yarp::sig::Vector buffer(2);
    iDynTree::toYarp(initialValue, buffer);

    m_comIntegrator->reset(buffer);
    m_comPosition = initialValue;

    std::cout << " STABLE DCM  " << buffer.toString() << std::endl;


    return true;
}
