/**
 * @file WalkingForwardKinematics.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_FK_HPP
#define WALKING_FK_HPP

// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>

//iDynTree
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>

// iCub-ctrl
#include <iCub/ctrl/filters.h>
class WalkingFK
{
    iDynTree::KinDynComputations m_kinDyn; /**< KinDynComputations solver. */

    bool m_prevContactLeft; /**< Boolean is the previous contact foot the left one? */
    bool m_dcmEvaluated; /**< is the DCM evaluated? */
    bool m_comEvaluated; /**< is the CoM evaluated? */

    iDynTree::FrameIndex m_frameLeftIndex; /**< Index of the frame attached to the left foot in which all the left foot transformations are expressed. */
    iDynTree::FrameIndex m_frameRightIndex; /**< Index of the frame attached to the right foot in which all the right foot transformations are expressed. */
    iDynTree::FrameIndex m_frameRootIndex; /**< Index of the frame attached to the root_link. */
    iDynTree::FrameIndex m_frameNeckIndex; /**< Index of the frame attached to the neck_2. */
    iDynTree::FrameIndex m_frameLeftHandIndex; /**< Index of the frame attached to the left hand. */
    iDynTree::FrameIndex m_frameRightHandIndex; /**< Index of the frame attached to the right hand. */
    iDynTree::FrameIndex m_frameHeadIndex; /**< Index of the frame attached to the head. */

    std::string m_baseFrameLeft; /**< Name of the left base frame. */
    std::string m_baseFrameRight;  /**< Name of the right base frame. */

    iDynTree::Transform m_frameHlinkLeft; /**< Transformation between the l_sole and the l_foot frame (l_ankle_2?!). */
    iDynTree::Transform m_frameHlinkRight; /**< Transformation between the l_sole and the l_foot frame (l_ankle_2?!). */
    iDynTree::Transform m_worldToBaseTransform; /**< World to base transformation. */

    iDynTree::Position m_comPosition; /**< Position of the CoM. */
    iDynTree::Vector3 m_comVelocity; /**< Velocity of the CoM. */
    iDynTree::Vector2 m_dcm; /**< DCM position. */
    double m_corrTerm; /**< Correction Term relative to Inclined Plane dcm position*/
    double m_omega; /**< Inverted time constant of the 3D-LIPM. */

    std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_comPositionFilter; /**< CoM position low pass filter. */
    std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_comVelocityFilter; /**< CoM velocity low pass filter. */
    iDynTree::Position m_comPositionFiltered; /**< Filtered position of the CoM. */
    iDynTree::Vector3 m_comVelocityFiltered; /**< Filtered velocity of the CoM. */
    bool m_useFilters; /**< If it is true the filters will be used. */

    double m_totalMass; /**< is the total mass of the robot, m. */
    iDynTree::FreeFloatingGeneralizedTorques m_generalizedGravityTorques; /** < Generalized gravity Torques vector. */

    bool m_firstStep; /**< True only during the first step. */

    /**
     * Set the model of the robot.
     * @param model iDynTree model.
     * @return true/false in case of success/failure.
     */
    bool setRobotModel(const iDynTree::Model& model);

    /**
     * Set The base frames.
     * @note: During the walking task the frame shift from the left to the right foot.
     * In general the base link is coincident to the stance foot.
     * @param lFootFrame name of the frame attached to the left foot;
     * @param tFootFrame name of the frame attached to the right foot;
     * @return true/false in case of success/failure.
     */
    bool setBaseFrames(const std::string& lFootFrame, const std::string& rFootFrame);

    /**
     * Evaluate the Divergent component of motion.
     */
     void evaluateDCM();

    /**
     * Evaluate the CoM position and velocity.
     */
    void evaluateCoM();

public:

    /**
     * Initialize the walking FK solver.
     * @param config config of the FK solver;
     * @param model iDynTree model.
     * @return true on success, false otherwise
     */
    bool initialize(const yarp::os::Searchable& config,
                    const iDynTree::Model& model);

    /**
     * Evaluate the first world to base transformation.
     * @note: The first reference frame is always the left foot.
     * @note: please use this method only with evaluateWorldToBaseTransformation(const bool& isLeftFixedFrame);
     * @param leftFootTransform transformation from the world to the left foot frame (l_sole);
     * @return true/false in case of success/failure.
     */
    bool evaluateFirstWorldToBaseTransformation(const iDynTree::Transform& leftFootTransform);

    /**
     * Evaluate the world to base transformation
     * @note: During the walking task the frame shift from the left to the right foot.
     * the new base frame is attached where the foot is.
     * @note: please use this method only with evaluateFirstWorldToBaseTransformation();
     * @param isLeftFixedFrame true if the main frame of the left foot is fixed one.
     * @return true/false in case of success/failure.
     */
    bool evaluateWorldToBaseTransformation(const bool& isLeftFixedFrame);

    /**
     * Evaluate the world to base transformation
     * @note: During the walking task the frame shift from the left to the right foot.
     * The base frame is attached where the foot should be (information sent by the planner)
     * @param leftFootTransform transformation from the world to the left foot frame (l_sole);
     * @param rightFootTransform transformation from the world to the right foot frame (r_sole);
     * @param isLeftFixedFrame true if the main frame of the left foot is fixed one.
     * @return true/false in case of success/failure.
     */
    bool evaluateWorldToBaseTransformation(const iDynTree::Transform& leftFootTransform,
                                           const iDynTree::Transform& rightFootTransform,
                                           const bool& isLeftFixedFrame);

    /**
     * Set the base for the onTheFly feature
     * @return true/false in case of success/failure.
     */
    bool setBaseOnTheFly();

    /**
     * Set the internal state of the robot (joint position and velocity)
     * @param positionFeedbackInRadians joint position feedback expressed in radians;
     * @param velocityFeedbackInRadians joint velocity feedback expressed in radians per seconds.
     */
    bool setInternalRobotState(const iDynTree::VectorDynSize& positionFeedbackInRadians,
                               const iDynTree::VectorDynSize& velocityFeedbackInRadians);
    
    /**
     * Set the omega value and the correction term.
     * @param comHeight is the com height in according with lipm.
     * @param inclPlaneAngle is the angle of the inclined plane.
     * @return true in case of the success, false otherwise.
     */
    bool setForwardKinematics(const double comHeight, const double inclPlaneAngle);

    /**
     * Get the CoM position.
     * @return CoM position
     */
    const iDynTree::Position& getCoMPosition();

    /**
     * Get the CoM velocity.
     * @return CoM velocity
     */
    const iDynTree::Vector3& getCoMVelocity();

    /**
     * Get the 2d-Divergent component of motion.
     * @return the 2d-Divergent component of motion
     */
    const iDynTree::Vector2& getDCM();

    /**
     * Return the transformation between the left foot frame (l_sole) and the world reference frame.
     * @return world_H_left_frame.
     */
    iDynTree::Transform getLeftFootToWorldTransform();

    /**
     * Return the transformation between the right foot frame (r_sole) and the world reference frame.
     * @return world_H_right_frame.
     */
    iDynTree::Transform getRightFootToWorldTransform();

    /**
     * Return the transformation between the left hand frame and the world reference frame.
     * @return world_H_left_hand.
     */
    iDynTree::Transform getLeftHandToWorldTransform();

    /**
     * Return the transformation between the right hand frame and the world reference frame.
     * @return world_H_right_hand.
     */
    iDynTree::Transform getRightHandToWorldTransform();

    /**
     * Return the transformation between the head frame and the world reference frame.
     * @return world_H_head.
     */
    iDynTree::Transform getHeadToWorldTransform();

    /**
     * Return the transformation between the root frame and the world reference frame.
     * @return world_H_root_frame.
     */
    iDynTree::Transform getRootLinkToWorldTransform();

    /**
     * Return the transformation from base frame to world.
     * @return world_H_base_frame.
     */
    iDynTree::Transform getWorldToBaseTransform();

    /**
     * Return the transformation from com frame to world frame 
     * ( the rotation part is assumed to be equal to the world rotation,
     * @return world_H_com_frame.
     */
    iDynTree::Transform getCoMTransform();

    /**
     * Return the root link velocity.
     * @return the root link velocity expressed with the mixed representation.
     */
    iDynTree::Twist getRootLinkVelocity();

    /**
     * Return the neck orientation.
     * @return the rotation matrix between the neck and the reference frame.
     */
    iDynTree::Rotation getNeckOrientation();

    /**
     * Get the left foot jacobian.
     * @param jacobian is the left foot jacobian matrix
     * @return true/false in case of success/failure.
     */
    bool getLeftFootJacobian(iDynTree::MatrixDynSize &jacobian);

    /**
     * Get the right foot jacobian.
     * @param jacobian is the right foot jacobian matrix
     * @return true/false in case of success/failure.
     */
    bool getRightFootJacobian(iDynTree::MatrixDynSize &jacobian);

    /**
     * Get the left hand jacobian.
     * @param jacobian is the left hand jacobian matrix.
     * @return true/false in case of success/failure.
     */
    bool getLeftHandJacobian(iDynTree::MatrixDynSize &jacobian);

    /**
     * Get the right hand jacobian.
     * @param jacobian is the right hand jacobian matrix.
     * @return true/false in case of success/failure.
     */
    bool getRightHandJacobian(iDynTree::MatrixDynSize &jacobian);

    /**
     * Get the neck jacobian.
     * @param jacobian is the neck jacobian matrix.
     * @return true/false in case of success/failure.
     */
    bool getNeckJacobian(iDynTree::MatrixDynSize &jacobian);

    /**
     * Get the CoM jacobian.
     * @param jacobian is the CoM jacobian matrix.
     * @return true/false in case of success/failure.
     */
    bool getCoMJacobian(iDynTree::MatrixDynSize &jacobian);

    /**
     * Get the generalzied gravity force vector.
     * @param freeFloatingGeneralizedTorques the generalized gravity force vector.
     * @return true/false in case of success/failure.
     */
    bool getGeneralizedGravityTorques(iDynTree::FreeFloatingGeneralizedTorques& freeFloatingGeneralizedTorques);

    /**
     * Get the Free Floating Mass Matrix.
     * @param floatingMassMatrix is the mass matrix of the system.
     * @return true/false in case of success/failure.
     */
    bool getFreeFloatingMassMatrix(iDynTree::MatrixDynSize &freeFloatingMassMatrix);

    /**
     * Set a velocity transformation from a given jacobian.
     * @param bToAJacobian is the Jacobian expresses from B to A.
     * @param bToAVelocityTransform is the velocity transformation from B to A.
     * @return true/false in case of success/failure.
     */
    bool setChangeBaseTransformation(iDynTree::MatrixDynSize& bToAJacobian, iDynTree::MatrixDynSize& bToAVelocityTransform);

    /**
     * Set a velocity transformation from a frame (B) to the frame attached to the CoM (C) : ( B_T_C) 
     * the relative generalized representation is given as follows : ( A_c , R_b , s )
     * @param Rb is the rotation of the frame B with respect to the world/inertial frame
     * @param bRc is the CoM position vector expressed with respect to the frame B
     * @param bJc is the jacobian matrix from the frame C to the frame B
     * @return true/false in case of success/failure.
     */
    bool setWorldToCoMChangeBaseTransformation(iDynTree::MatrixDynSize Rb, iDynTree::MatrixDynSize bRc, iDynTree::MatrixDynSize bJc, iDynTree::MatrixDynSize& AToCoMVelocityTransform);

    /**
     *  
     * @return true/false in case of success/failure.
     */
    bool toMatrixDynSize(iDynTree::Rotation &Rot, iDynTree::MatrixDynSize &Mat);

    /**
     *  
     * @return true/false in case of success/failure.
     */
    bool toVectorDynSize(iDynTree::Position &pos, iDynTree::VectorDynSize &vec);

    /**
     * Get the total mass of the robot.
     * @return true/false in case of success/failure.
     */
    bool getTotalMass(double& totalMass);

    /**
     * Get the jacobian matrix from CoM to left foot : Jlc 
     * @return true/false in case of success/failure.
     */
    bool getCoMToLeftFootJacobian(iDynTree::MatrixDynSize &jacobian);

    /**
     * Get the jacobian matrix from CoM to right foot : Jrc 
     * @return true/false in case of success/failure.
     */
    bool getCoMToRightFootJacobian(iDynTree::MatrixDynSize &jacobian);

    /**
     * Get the jacobian matrix from CoM to the feet : Jlrc = [ Jlc | Jrc ]
     * @return true/false in case of success/failure.
     */
    bool getCoMToFeetJacobian(iDynTree::MatrixDynSize &jacobian);

    /**
     *
     * @return true/false in case of success/failure.
     */
    bool getSkewMatrix(iDynTree::VectorDynSize &vec, iDynTree::MatrixDynSize &skewMatrix);

    /**
     *
     * @return true/false in case of success/failure.
     */
    bool getAdjointMatrix(iDynTree::MatrixDynSize &Rotation, iDynTree::VectorDynSize &position, iDynTree::MatrixDynSize &adjointMatrix);

    /**
     *
     * @return true/false in case of success/failure.
     */
    bool getGraspMatrix(iDynTree::MatrixDynSize &Rotation, iDynTree::VectorDynSize &position, iDynTree::MatrixDynSize &contactModelMatrix, iDynTree::MatrixDynSize &graspMatrix);

    /**
     *
     * @return true/false in case of success/failure.
     */
    bool getLeftFootToCoMGraspMatrix(iDynTree::MatrixDynSize &contactModelMatrix, iDynTree::MatrixDynSize &leftFootToCoMGraspMatrix);

    /**
     *
     * @return true/false in case of success/failure.
     */
    bool getRightFootToCoMGraspMatrix(iDynTree::MatrixDynSize &contactModelMatrix, iDynTree::MatrixDynSize &rightFootToCoMGraspMatrix);

    /**
     *
     * @return true/false in case of success/failure.
     */
    bool getFeetToCoMGraspMatrix(iDynTree::MatrixDynSize &contactModelMatrix, iDynTree::MatrixDynSize &feetToCoMGraspMatrix);

    /**
     *
     * @return true/false in case of success/failure.
     */
    bool getPseudoInverseOfGraspMatrix(iDynTree::MatrixDynSize &graspMatrix, iDynTree::MatrixDynSize &pseudoInverseGraspMatrix);

};

#endif
