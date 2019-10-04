/**
 * @file LinearInvertedPendulumModel.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef STABLE_DCM_MODEL_HPP
#define STABLE_DCM_MODEL_HPP

// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>
#include <yarp/sig/Vector.h>

// iCub-ctrl
#include <iCub/ctrl/pids.h>

//iDynTree
#include <iDynTree/Core/VectorFixSize.h>

/**
 * StableDCMModel linear inverted pendulum model.
 */
class StableDCMModel
{
    double m_omega; /**< Inverted time constant of the 3D-LIPM. */
    double m_comHeight; /**< CoM height ( z component ). */
    double m_inclPlaneAngle; /**< inclined plane angle. */

    iDynTree::Vector2 m_dcmCorrTerm; /**< Correction Term vector for CoM velocity. */
    iDynTree::Vector2 m_lipCorrTerm; /**< Correction Term vector for CoM acceleration. */

    std::unique_ptr<iCub::ctrl::Integrator> m_comIntegrator{nullptr}; /**< CoM integrator object. */

    iDynTree::Vector2 m_dcmPosition; /**< Position of the DCM. */
    iDynTree::Vector2 m_zmpPosition; /**< Position of the DCM. */
    iDynTree::Vector2 m_comPosition; /**< Position of the CoM. */
    iDynTree::Vector2 m_comVelocity; /**< Velocity of the CoM. */
    iDynTree::Vector2 m_comAcceleration; /**< Acceleration of the CoM. */

    bool m_isModelPropagated{false}; /**< True if the model is propagated. */

public:

    /**
     * Initialize the 3D-LIPM.
     * @param config config of the 3D-LIPM;
     * @return true on success, false otherwise.
     */
    bool initialize(const yarp::os::Searchable& config);

    /**
     * Set the controlled input.
     * @param controlledInput of the 3D-LIPM (i.e. Position of the ZMP).
     */
    void setDCMPosition(const iDynTree::Vector2& input);

    /**
     * Set the controlled input.
     * @param controlledInput of the 3D-LIPM (i.e. Position of the ZMP).
     */
    void setZMPPosition(const iDynTree::Vector2& input);

    /**
     * Update trajectory-dependent quantities.
     * @return true on success, false otherwise.
     **/
    bool setStableDCMModel(double inclPlaneAngle, double yawAngle);

    /**
     * Integrate the model.
     * @return true on success, false otherwise.
     */
    bool integrateModel();

    /**
     * Get the position of the CoM.
     * @param comPosition position of the CoM.
     * @return true on success, false otherwise.
     */
    bool getCoMPosition(iDynTree::Vector2& comPosition);

    /**
     * Get the velocity of the CoM.
     * @param comVelocity velocity of the CoM.
     * @return true on success, false otherwise.
     */
    bool getCoMVelocity(iDynTree::Vector2& comVelocity);

    /**
     * Get the acceleration of the CoM.
     * @param comAcceleration acceleration of the CoM.
     * @return true on success, false otherwise.
     */
    bool getCoMAcceleration(iDynTree::Vector2& comAcceleration);

    /**
     * Reset the Model
     * @param initialValue initial position of the CoM
     * @return true/false in case of success/failure
     */
    bool reset(const iDynTree::Vector2& initialValue);
};

#endif
