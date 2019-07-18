/**
 * @file EnvironmentDetectionModule.hpp
 * @authors Liana Bertoni <liana.bertoni@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef WALKING_ENVIRONMENT_DETECTION_MODULE_HPP
#define WALKING_ENVIRONMENT_DETECTION_MODULE_HPP

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RpcClient.h>

/**
 * RFModule to handle the Environment Detection
 */
class EnvironmentDetectionModule : public yarp::os::RFModule
{
private:
    double m_dT; /**< RFModule period. */

    yarp::dev::PolyDriver m_environmentDetector; /**< Environment Detector polydriver. */

    yarp::os::RpcClient m_rpcClientPort; /**< RPC port. */
    std::string m_rpcServerPortName; /**< Name of the walking-module RPC port. */
    std::string m_rpcClientPortName; /**< Name of the joypad-module RPC port */

    yarp::os::BufferedPort<yarp::sig::Vector> m_robotGoalPort; /**< Port used to specify the desired goal position. */
    std::string m_robotGoalOutputPortName; /**< Name of the robotGoal port (opened by the joypad module) */
    std::string m_robotGoalInputPortName; /**< Name of the robotGoal port (opened by the walking module) */

    /**
     * Standard deadzone function.
     * @param input input of the deadzone
     * @return 0 if the abs(input) < abs(deadzone) otherwise return the input scaled.
     */
    double deadzone(const double &input);

public:

    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder &rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;
};

#endif
