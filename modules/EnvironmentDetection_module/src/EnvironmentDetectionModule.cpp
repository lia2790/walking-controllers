/**
 * @file JoypadModule.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>

#include <EnvironmentDetectionModule.hpp>
#include <Utils.hpp>

bool EnvironmentDetectionModule::configure(yarp::os::ResourceFinder &rf)
{
    // check if the configuration file is empty
    if(rf.isNull())
    {
        yError() << "[EnvironmentDetectionModul::configure] Empty configuration file.";
        return false;
    }

    // get the period
    m_dT = rf.check("period", yarp::os::Value(0.1)).asDouble();
    // get the inclined plane angle
    m_inclined_plane_angle = rf.check("inclined_plane_angle", yarp::os::Value(0.1)).asDouble();

    // set the module name
    std::string name;
    if(!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[EnvironmentDetectionModul::configure] Unable to get a string from searchable";
        return false;
    }
    setName(name.c_str());

    std::string portName;
    if(!YarpHelper::getStringFromSearchable(rf, "rpcClientPort_name", portName))
    {
        yError() << "[EnvironmentDetectionModul::configure] Unable to get a string from searchable";
        return false;
    }
    m_rpcClientPortName = "/" + name + portName;
    m_rpcClientPort.open(m_rpcClientPortName);

    if(!YarpHelper::getStringFromSearchable(rf, "rpcServerPort_name", m_rpcServerPortName))
    {
        yError() << "[EnvironmentDetectionModul::configure] Unable to get a string from searchable";
        return false;
    }
    yarp::os::Network::connect(m_rpcClientPortName, m_rpcServerPortName);

    return true;
}

double EnvironmentDetectionModule::getPeriod()
{
    return m_dT;
}

bool EnvironmentDetectionModule::close()
{
    if (m_environmentDetector.isValid())
    {
        m_environmentDetector.close();
    }

    // close the ports
    m_rpcClientPort.close();
    
    return true;
}

bool EnvironmentDetectionModule::updateModule()
{
    return true;
}

