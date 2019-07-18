/**
 * @file main.cpp
 * @authors Liana Bertoni <liana.bertoni@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include "EnvironmentDetectionModule.hpp"

int main(int argc, char * argv[])
{
    // initialise yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"[EnvironmentDetectorModule]Unable to find YARP network";
        return EXIT_FAILURE;
    }

    // prepare and configure the resource finder
    yarp::os::ResourceFinder &rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.setDefaultConfigFile("environmentDetector.ini");
    rf.setDefaultContext("dcmWalkingEnvironmentDetection");

    rf.configure(argc, argv);

    // create the producer module
    EnvironmentDetectionModule module;

    return module.runModule(rf);
}
