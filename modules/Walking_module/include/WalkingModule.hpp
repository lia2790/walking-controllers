/**
 * @file WalkingModule.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_MODULE_HPP
#define WALKING_MODULE_HPP

// std
#include <memory>
#include <deque>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/RpcClient.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <RobotHelper.hpp>
#include <TrajectoryGenerator.hpp>
#include <WalkingDCMModelPredictiveController.hpp>
#include <WalkingDCMReactiveController.hpp>
#include <WalkingZMPController.hpp>
#include <WalkingInverseKinematics.hpp>
#include <WalkingQPInverseKinematics.hpp>
#include <WalkingQPInverseKinematics_osqp.hpp>
#include <WalkingQPInverseKinematics_qpOASES.hpp>
#include <WalkingTaskBasedTorqueController.hpp>
#include <WalkingForwardKinematics.hpp>
#include <StableDCMModel.hpp>
#include <WalkingPIDHandler.hpp>
#include <WalkingLogger.hpp>
#include <TimeProfiler.hpp>

// iCub-ctrl
#include <iCub/ctrl/filters.h>

#include <thrifts/WalkingCommands.h>

/**
 * RFModule of the Walking controller
 */
class WalkingModule: public yarp::os::RFModule, public WalkingCommands
{
    enum class WalkingFSM {Idle, Configured, Preparing, Prepared, Walking, Stance, Paused, Stopped};
    WalkingFSM m_robotState{WalkingFSM::Idle}; /**< State  of the WalkingFSM. */

    double m_dT; /**< RFModule period. */
    double m_time; /**< Current time. */
    std::string m_robot; /**< Robot name. */

    bool m_firstStep; /**< True if this is the first step. */
    bool m_useMPC; /**< True if the MPC controller is used. */
    bool m_useQPIK; /**< True if the QP-IK is used. */
    bool m_useOSQP; /**< True if osqp is used to QP-IK problem. */
    bool m_dumpData; /**< True if data are saved. */
    bool m_useTorque; /**< True if the torque controller is used. */
    bool m_useWaitCondition; /**< True the wait condition is used. */
    bool m_useConstantRegularization;

    std::unique_ptr<RobotHelper> m_robotControlHelper; /**< Robot control helper. */
    std::unique_ptr<TrajectoryGenerator> m_trajectoryGenerator; /**< Pointer to the trajectory generator object. */
    std::unique_ptr<WalkingController> m_walkingController; /**< Pointer to the walking DCM MPC object. */
    std::unique_ptr<WalkingDCMReactiveController> m_walkingDCMReactiveController; /**< Pointer to the walking DCM reactive controller object. */
    std::unique_ptr<WalkingZMPController> m_walkingZMPController; /**< Pointer to the walking ZMP controller object. */
    std::unique_ptr<WalkingIK> m_IKSolver; /**< Pointer to the inverse kinematics solver. */
    std::shared_ptr<WalkingQPIK_osqp> m_QPIKSolver_osqp; /**< Pointer to the inverse kinematics solver (osqp). */
    std::shared_ptr<WalkingQPIK_qpOASES> m_QPIKSolver_qpOASES; /**< Pointer to the inverse kinematics solver (qpOASES). */
    std::unique_ptr<WalkingTaskBasedTorqueController> m_taskBasedTorqueSolver; /**< Pointer to the task-based torque solver. */
    std::unique_ptr<WalkingFK> m_FKSolver; /**< Pointer to the forward kinematics solver. */
    std::unique_ptr<StableDCMModel> m_stableDCMModel; /**< Pointer to the stable DCM dynamics. */
    std::unique_ptr<WalkingPIDHandler> m_PIDHandler; /**< Pointer to the PID handler object. */
    std::unique_ptr<WalkingLogger> m_walkingLogger; /**< Pointer to the Walking Logger object. */
    std::unique_ptr<TimeProfiler> m_profiler; /**< Time profiler. */

    std::unique_ptr<WalkingFK> m_FKSolverDebug; /**< Pointer to the forward kinematics solver. */

    std::deque<iDynTree::Transform> m_leftTrajectory; /**< Deque containing the trajectory of the left foot. */
    std::deque<iDynTree::Transform> m_rightTrajectory; /**< Deque containing the trajectory of the right foot. */

    std::deque<iDynTree::Twist> m_leftTwistTrajectory; /**< Deque containing the twist trajectory of the left foot. */
    std::deque<iDynTree::Twist> m_rightTwistTrajectory; /**< Deque containing the twist trajectory of the right foot. */

    std::deque<iDynTree::SpatialAcc> m_leftAccelerationTrajectory; /**< Deque containing the twist trajectory of the left foot. */
    std::deque<iDynTree::SpatialAcc> m_rightAccelerationTrajectory; /**< Deque containing the twist trajectory of the right foot. */

    std::deque<iDynTree::Vector2> m_DCMPositionDesired; /**< Deque containing the desired DCM position. */
    std::deque<iDynTree::Vector2> m_DCMVelocityDesired; /**< Deque containing the desired DCM velocity. */
    std::deque<iDynTree::Vector2> m_ZMPPositionDesired; /**< Deque containing the desired ZMP position. */
    std::deque<bool> m_leftInContact; /**< Deque containing the left foot state. */
    std::deque<bool> m_rightInContact; /**< Deque containing the right foot state. */
    std::deque<double> m_comHeightTrajectory; /**< Deque containing the CoM height trajectory. */
    std::deque<double> m_comHeightVelocity; /**< Deque containing the CoM height velocity. */
    std::deque<size_t> m_mergePoints; /**< Deque containing the time position of the merge points. */
    std::deque<double> m_weightInLeft; /**< Deque containing the left foot weight percentage. */
    std::deque<double> m_weightInRight; /**< Deque containing the right foot weight percentage. */
    std::deque<bool> m_isLeftFixedFrame; /**< Deque containing when the main frame of the left foot is the fixed frame
                                            In general a main frame of a foot is the fix frame only during the
                                            stance and the switch out phases. */


    iDynTree::ModelLoader m_loader; /**< Model loader class. */

    iDynTree::VectorDynSize m_qDesired; /**< Vector containing the results of the IK algorithm [rad]. */
    iDynTree::VectorDynSize m_dqDesired; /**< Vector containing the results of the IK algorithm [rad]. */
    iDynTree::VectorDynSize m_ddqDesired; /**< Vector containing the results of the task based torque algorithm [rad/s^2]. */
    iDynTree::VectorDynSize m_torqueDesired; /**< Vector containing the desired joint torques. */

    iDynTree::Rotation m_inertial_R_worldFrame; /**< Rotation between the inertial and the world frame. */

    yarp::os::Port m_rpcPort; /**< Remote Procedure Call port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_desiredUnyciclePositionPort; /**< Desired robot position port. */

    yarp::os::RpcClient m_rpcBaseEstPort; /**< Remote Procedure Call port. */

    yarp::os::BufferedPort<yarp::sig::Vector> m_floatingBasePort; /**< Desired robot position port. */

    bool m_newTrajectoryRequired; /**< if true a new trajectory will be merged soon. (after m_newTrajectoryMergeCounter - 2 cycles). */
    size_t m_newTrajectoryMergeCounter; /**< The new trajectory will be merged after m_newTrajectoryMergeCounter - 2 cycles. */

    std::mutex m_mutex; /**< Mutex. */

    iDynTree::Vector2 m_desiredPosition;

    bool m_waitCondition;
    double m_switchInThreshold;
    double m_switchOutThreshold;

    // debug
    std::unique_ptr<iCub::ctrl::Integrator> m_velocityIntegral{nullptr};
    std::unique_ptr<iCub::ctrl::Integrator> m_accelerationIntegral{nullptr};

    double m_footHeight;
    double m_footVelocityLanding;

    // walking on inclined plane
    double m_inclPlaneAngle; /**< angle of the inclined plane. */
    double m_comHeight; /**< height of the centre of the mass. */
    int m_newContact; /**< flag to handle new contact. */
    iDynTree::Transform m_wTcontactFoot; /**< contact foot transformation which provides the inclined plane transform. */
    iDynTree::Vector2 m_desiredCoMPositionXY; /**< desired CoM XY position at the time step. */

    /**
     * Get the robot model from the resource finder and set it.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool setRobotModel(const yarp::os::Searchable& rf);

    /**
     * Propagate time.
     */
    void propagateTime();

    /**
     * Advance the reference signal.
     * @return true in case of success and false otherwise.
     */
    bool advanceReferenceSignals();

    void checkWaitCondition(const std::deque<bool>& footInContact,
                            const iDynTree::Wrench& contactWrench);

    /**
     * Update the FK solver.
     * @return true in case of success and false otherwise.
     */
    bool updateFKSolver();

    /**
     * Set the QP-IK problem.
     * @param solver is the pointer to the solver (osqp or qpOASES)
     * @param desiredCoMPosition desired CoM position;
     * @param desiredCoMVelocity desired CoM velocity;
     * @param desiredNeckOrientation desired neck orientation (rotation matrix);
     * @param output is the output of the solver (i.e. the desired joint velocity)
     * @return true in case of success and false otherwise.
     */
    bool solveQPIK(const std::shared_ptr<WalkingQPIK> solver,
                   const iDynTree::Position& desiredCoMPosition,
                   const iDynTree::Vector3& desiredCoMVelocity,
                   const iDynTree::Rotation& desiredNeckOrientation,
                   iDynTree::VectorDynSize &output);

    /**
     * Set the task based torque controller problem.
     * @param desiredCoMPosition desired CoM position;
     * @param desiredCoMVelocity desired CoM velocity;
     * @param desiredCoMAcceleration desired CoM Acceleration;
     * @param desiredCoMAcceleration desired ZMP Position;
     * @param desiredNeckOrientation desired neck orientation (rotation matrix);
     * @param output is the output of the solver (i.e. the desired joint velocity)
     * @return true in case of success and false otherwise.
     */
    bool solveTaskBased(const iDynTree::Rotation& desiredNeckOrientation,
                        const iDynTree::Position& desiredCoMPosition,
                        const iDynTree::Vector3& desiredCoMVelocity,
                        const iDynTree::Vector3& desiredCoMAcceleration,
                        const iDynTree::Vector2& desiredZMPPosition,
                        const iDynTree::Vector3& desiredVRPPosition,
                        iDynTree::VectorDynSize &outputTorque,
                        iDynTree::VectorDynSize &outputAcceleration);

    /**
     *
     * @return true in case of success and false otherwise.
     */
    bool readIMUData(iDynTree::Position gIMU);

    /**
     *
     * @return the inclined plane angle.
     */
    double imuDetectGround();

    /**
     *
     * @return the inclined plane angle..
     */
    double kinDetectGround();

    /**
     *
     * @return true in case of success and false otherwise.
     */
    bool detectGround();

    /**
     *
     * @return true in case of success and false otherwise.
     */
    // bool computeYaw();

    /**
     *
     * @return true in case of success and false otherwise.
     */
    bool updateOmegaDCM();
    

    /**
     * Generate the first trajectory.
     * This method has to be called before updateTrajectories() method.
     * @return true in case of success and false otherwise.
     */
    bool generateFirstTrajectories();

    /**
     * Generate the first trajectory. (onTheFly)
     * @param leftToRightTransform transformation between left and right feet.
     * @return true in case of success and false otherwise.
     */
    bool generateFirstTrajectories(const iDynTree::Transform &leftToRightTransform);

    /**
     * Ask for a new trajectory (The trajectory will be evaluated by a thread).
     * @param initTime is the initial time of the trajectory;
     * @param isLeftSwinging todo wrong name?;
     * @param measuredTransform transformation between the world and the (stance/swing??) foot;
     * @param mergePoint is the instant at which the old and the new trajectory will be merged;
     * @param desiredPosition final desired position of the projection of the CoM.
     * @return true/false in case of success/failure.
     */
    bool askNewTrajectories(const double& initTime, const bool& isLeftSwinging,
                            const iDynTree::Transform& measuredTransform,
                            const size_t& mergePoint, const iDynTree::Vector2& desiredPosition);

    /**
     * Update the old trajectory.
     * This method has to be called only if the trajectory generator has finished to evaluate the new trajectory.
     * The old and the new trajectory will be merged at mergePoint.
     * @param mergePoint instant at which the old and the new trajectory will be merged
     * @return true/false in case of success/failure.
     */
    bool updateTrajectories(const size_t& mergePoint);

    /**
     * Set the input of the planner. The desired position is expressed using a
     * reference frame attached to the robot. The X axis points forward while the
     * Y axis points on the left.
     * @param x desired forward position of the robot
     * @param y desired lateral position of the robot
     * @return true/false in case of success/failure.
     */
    bool setPlannerInput(double x, double y);

    /**
     * Reset the entire controller architecture
     */
    void reset();

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
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;

    /**
     * This allows you to put the robot in a home position for walking.
     * @return true in case of success and false otherwise.
     */
    virtual bool prepareRobot(bool onTheFly = false) override;

    /**
     * Start walking.
     * @return true in case of success and false otherwise.
     */
    virtual bool startWalking() override;

    /**
     * set the desired final position of the CoM.
     * @param x desired x position of the CoM;
     * @param y desired y position of the CoM.
     * @return true in case of success and false otherwise.
     */
    virtual bool setGoal(double x, double y) override;

    /**
     * Pause walking.
     * @return true in case of success and false otherwise.
     */
    virtual bool pauseWalking() override;

    /**
     * Stop walking.
     * @return true in case of success and false otherwise.
     */
    virtual bool stopWalking() override;

    /**
     * Set angle.
     * @return true in case of success and false otherwise.
     */
    virtual bool setAngle(double angleInDegrees) override;

};
#endif
