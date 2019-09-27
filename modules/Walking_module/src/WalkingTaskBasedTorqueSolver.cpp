/**
 * @file TaskBasedTorqueSolver.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <iDynTree/yarp/YARPConfigurationsLoader.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <WalkingTaskBasedTorqueSolver.hpp>
#include <Utils.hpp>

// #include <EigenMatio/EigenMatio.hpp>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;

bool TaskBasedTorqueSolver::setCoMGains(const iDynTree::Vector3& kp, const iDynTree::Vector3& kd)
{
    if(m_useCoMConstraint)
    {
        auto constraint = m_constraints.find("com");
        if(constraint != m_constraints.end())
        {
            auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
            ptr->positionController()->setGains(kp, kd);
        }
    }

    auto constraint = m_constraints.find("com_linear_momentum_constraint");
    if(constraint != m_constraints.end())
    {
        auto ptr = std::dynamic_pointer_cast<LinearMomentumElement>(constraint->second);
        ptr->positionController()->setGains(kp, kd);
    }

    return true;
}

bool TaskBasedTorqueSolver::instantiateCoMConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yInfo() << "[instantiateCoMConstraint] Empty configuration file. The CoM Constraint will not be used";
        m_useCoMConstraint = false;
        return true;
    }

    // bool asLinearMomentum = config.check("as_linear_momentum", yarp::os::Value("False")).asBool();
    // if(asLinearMomentum)
    //     return true;

    m_useCoMConstraint = true;

    bool useDefaultKd = config.check("useDefaultKd", yarp::os::Value("False")).asBool();

    yarp::os::Value tempValue;
    iDynTree::Vector3 kp;
    tempValue = config.find("kp");
    if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kp))
    {
        yError() << "[initialize] Unable to convert a YARP list to an iDynTree::VectorFixSize, "
                 << "joint regularization";
        return false;
    }

    iDynTree::Vector3 kd;
    if(useDefaultKd)
    {
        double scaling;
        if(!YarpHelper::getNumberFromSearchable(config, "scaling", scaling))
        {
            yError() << "[initialize] Unable to get the scaling factor.";
            return false;
        }
        iDynTree::toEigen(kd) = 2 / scaling * iDynTree::toEigen(kp).array().sqrt();
    }
    else
    {
        tempValue = config.find("kd");
        if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kd))
        {
            yError() << "[initialize] Unable to convert a YARP list to an iDynTree::VectorFixSize, "
                     << "joint regularization";
            return false;
        }
    }

    m_controlOnlyCoMHeight = config.check("controllOnlyHeight", yarp::os::Value("False")).asBool();

    // resize com quantities
    std::shared_ptr<CartesianConstraint> ptr;
    if(!m_controlOnlyCoMHeight)
    {
        m_comJacobian.resize(3, m_actuatedDOFs + 6);
        m_comBiasAcceleration.resize(3);

        // memory allocation
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::POSITION);
    }
    else
    {
        m_comJacobian.resize(1, m_actuatedDOFs + 6);
        m_comBiasAcceleration.resize(1);

        // memory allocation
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::ONE_DIMENSION);
    }

    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);

    ptr->positionController()->setGains(kp, kd);
    ptr->setRoboticJacobian(m_comJacobian);
    ptr->setBiasAcceleration(m_comBiasAcceleration);

    m_constraints.insert(std::make_pair("com", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

bool TaskBasedTorqueSolver::instantiateJointRateOfChangeConstraint(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    if(config.isNull())
    {
        yError() << "[instantiateRateOfChangeConstraint] Empty configuration for rate of change constraint. This constraint will not take into account";
        return true;
    }

    tempValue = config.find("joint_torques");
    iDynTree::VectorDynSize maximumRateOfChange(m_actuatedDOFs);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, maximumRateOfChange))
    {
        yError() << "Initialization failed while reading maximumRateOfChange vector.";
        return true;
    }

    std::shared_ptr<RateOfChangeConstraint> ptr;
    ptr = std::make_shared<RateOfChangeConstraint>(m_actuatedDOFs);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, m_actuatedDOFs + 6);

    ptr->setMaximumRateOfChange(maximumRateOfChange);
    ptr->setPreviousValues(m_desiredJointTorque);

    m_constraints.insert(std::make_pair("rate_of_change_joint_torques", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

bool TaskBasedTorqueSolverSingleSupport::instantiateContactWrenchesRateOfChangeConstraint(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    if(config.isNull())
    {
        yError() << "[instantiateRateOfChangeConstraint] Empty configuration for rate of change constraint. This constraint will not take into account";
        return true;
    }

    tempValue = config.find("contact_wrenches_limit");
    iDynTree::VectorDynSize maximumRateOfChange(6);
    if(YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, maximumRateOfChange))
    {
        auto ptrConstraint = std::make_shared<RateOfChangeConstraint>(6);
        ptrConstraint->setSubMatricesStartingPosition(m_numberOfConstraints, m_actuatedDOFs + 6 + m_actuatedDOFs);

        ptrConstraint->setMaximumRateOfChange(maximumRateOfChange);
        ptrConstraint->setPreviousValues(m_desiredContactWrenches);

        m_constraints.insert(std::make_pair("rate_of_change_contact_wrench", ptrConstraint));
        m_numberOfConstraints += ptrConstraint->getNumberOfConstraints();
    }

    // cost
    tempValue = config.find("contact_wrenches_cost");
    iDynTree::VectorDynSize weight(6);
    if(YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, weight))
    {
        auto ptrCost = std::make_shared<RateOfChangeCostFunction>(6);
        ptrCost->setSubMatricesStartingPosition(m_actuatedDOFs + 6 + m_actuatedDOFs, 0);

        ptrCost->setWeight(weight);
        ptrCost->setPreviousValues(m_desiredContactWrenches);

        m_costFunctions.insert(std::make_pair("rate_of_change_contact_wrench_cost", ptrCost));
        m_hessianMatrices.insert(std::make_pair("rate_of_change_contact_wrench_cost",
                                                std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("rate_of_change_contact_wrench_cost",
                                                std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }
    return true;
}

bool TaskBasedTorqueSolverDoubleSupport::instantiateContactWrenchesRateOfChangeConstraint(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    if(config.isNull())
    {
        yError() << "[instantiateRateOfChangeConstraint] Empty configuration for rate of change constraint. This constraint will not take into account";
        return true;
    }

    tempValue = config.find("contact_wrenches");
    iDynTree::VectorDynSize maximumRateOfChange(6);
    if(YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, maximumRateOfChange))
    {
        iDynTree::VectorDynSize maximumRateOfChangeBothFeet(12);
        for(int i = 0; i < 6; i++)
        {
            maximumRateOfChangeBothFeet(i) = maximumRateOfChange(i);
            maximumRateOfChangeBothFeet(i + 6) = maximumRateOfChange(i);
        }

        std::shared_ptr<RateOfChangeConstraint> ptr;
        ptr = std::make_shared<RateOfChangeConstraint>(12);
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, m_actuatedDOFs + 6 + m_actuatedDOFs);

        ptr->setMaximumRateOfChange(maximumRateOfChangeBothFeet);
        ptr->setPreviousValues(m_desiredContactWrenches);

        m_constraints.insert(std::make_pair("rate_of_change_contact_wrench", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();
    }
    // cost
    tempValue = config.find("contact_wrenches_cost");
    iDynTree::VectorDynSize weight(6);
    if(YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, weight))
    {
        yError() << "[instantiateNeckSoftConstraint] Initialization failed while reading contact_wrenches_cost.";

        iDynTree::VectorDynSize weightBothFeet(12);
        for(int i = 0; i < 6; i++)
        {
            weightBothFeet(i) = weight(i);
            weightBothFeet(i + 6) = weight(i);
        }

        auto ptrCost = std::make_shared<RateOfChangeCostFunction>(12);
        ptrCost->setSubMatricesStartingPosition(m_actuatedDOFs + 6 + m_actuatedDOFs, 0);

        ptrCost->setWeight(weightBothFeet);
        ptrCost->setPreviousValues(m_desiredContactWrenches);

        m_costFunctions.insert(std::make_pair("rate_of_change_contact_wrench_cost", ptrCost));
        m_hessianMatrices.insert(std::make_pair("rate_of_change_contact_wrench_cost",
                                                std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("rate_of_change_contact_wrench_cost",
                                                std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }
    return true;
}

bool TaskBasedTorqueSolver::instantiateMotorReflectedInertia(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    if(config.isNull())
    {
        yWarning() << "[instantiateMotorReflectedInertia] Motor reflected inertia will not be used.";
        m_useMotorReflectedInertia = false;
        return true;
    }

    m_useMotorReflectedInertia = true;

    tempValue = config.find("gamma");
    iDynTree::VectorDynSize gamma(m_actuatedDOFs);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, gamma))
    {
        yError() << "Initialization failed while reading gamma vector.";
        return false;
    }

    tempValue = config.find("motors_inertia");
    iDynTree::VectorDynSize motorsInertia(m_actuatedDOFs);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, motorsInertia))
    {
        yError() << "Initialization failed while reading motors_inertia vector.";
        return false;
    }

    bool useHarmonicDriveInertia = config.check("use_harmonic_drive", yarp::os::Value("False")).asBool();
    if (useHarmonicDriveInertia)
    {
        tempValue = config.find("harmonic_drive_inertia");
        iDynTree::VectorDynSize harmonicDriveInertia(m_actuatedDOFs);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, harmonicDriveInertia))
        {
            yError() << "Initialization failed while reading motors_inertia vector.";
            return false;
        }

        iDynTree::toEigen(motorsInertia) += iDynTree::toEigen(harmonicDriveInertia);
    }
    else
        yWarning() << "[instantiateMotorReflectedInertia] Harmonic drive inertia is not used.";

    // parameters for coupling matrices. Updated according to the wiki:
    // http://wiki.icub.org/wiki/ICub_coupled_joints
    // and corrected according to https://github.com/robotology/robots-configuration/issues/39
    double t, r, R;
    if(!YarpHelper::getNumberFromSearchable(config, "t", t ))
    {
        yError() << "[instantiateMotorReflectedInertia] Unable to find the 't' parameters";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "r", r ))
    {
        yError() << "[instantiateMotorReflectedInertia] Unable to find the 'r' parameters";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "R", R ))
    {
        yError() << "[instantiateMotorReflectedInertia] Unable to find the 'R' parameters";
        return false;
    }

    iDynTree::MatrixDynSize couplingMatrix(m_actuatedDOFs, m_actuatedDOFs);
    for(int i = 0; i < m_actuatedDOFs; i++)
        couplingMatrix(i, i) = 1;

    // TODO do in a better way (HARD CODED JOINTS)
    iDynTree::toEigen(couplingMatrix).block(0, 0, 3, 3) <<   0.5,    -0.5,     0,
                                                             0.5,     0.5,     0,
                                                             r/(2*R), r/(2*R), r/R;

    iDynTree::toEigen(couplingMatrix).block(3, 3, 3, 3) << -1, 0, 0,
                                                           -1, -t, 0,
                                                            0, t, -t;


    iDynTree::toEigen(couplingMatrix).block(7, 7, 3, 3) << 1, 0, 0,
                                                           1, t, 0,
                                                           0, -t, t;

    // TODO
    // if(!YarpHelper::getNumberFromSearchable(config, "k_ff", m_kFF))
    // {
    //     yError() << "[instantiateMotorReflectedInertia] Unable to find the k_ff parameter";
    //     return false;
    // }

    m_reflectedInertia.resize(6 + m_actuatedDOFs, 6 + m_actuatedDOFs);
    iDynTree::toEigen(m_reflectedInertia).block(6, 6, m_actuatedDOFs, m_actuatedDOFs)  =
        (iDynTree::toEigen(couplingMatrix) * iDynTree::toEigen(gamma).asDiagonal()).inverse().transpose() *
        iDynTree::toEigen(motorsInertia).asDiagonal() *
        (iDynTree::toEigen(couplingMatrix) * iDynTree::toEigen(gamma).asDiagonal()).inverse();

    return true;
}

bool TaskBasedTorqueSolver::instantiateNeckSoftConstraint(const yarp::os::Searchable& config)
{
    bool useNeckAsConstraint = config.check("useAsConstraint", yarp::os::Value("False")).asBool();
    yarp::os::Value tempValue;
    if(config.isNull())
    {
        yError() << "[instantiateNeckSoftConstraint] Empty configuration neck soft constraint.";
        return false;
    }

    double c0, kp, kd;
    if(!YarpHelper::getNumberFromSearchable(config, "c0", c0))
        return false;

    if(!YarpHelper::getNumberFromSearchable(config, "kp", kp))
        return false;

    bool useDefaultKd = config.check("useDefaultKd", yarp::os::Value("False")).asBool();
    if(useDefaultKd)
    {
        double scaling;
        if(!YarpHelper::getNumberFromSearchable(config, "scaling", scaling))
        {
            yError() << "[initialize] Unable to get the scaling factor.";
            return false;
        }
        kd = 2 / scaling * std::sqrt(kp);
    }
    else
    {
        if(!YarpHelper::getNumberFromSearchable(config, "kd", kd))
            return false;
    }

    if(!iDynTree::parseRotationMatrix(config, "additional_rotation", m_additionalRotation))
    {
        yError() << "[instantiateNeckSoftConstraint] Unable to set the additional rotation.";
        return false;
    }

    m_neckBiasAcceleration.resize(3);
    m_neckJacobian.resize(3, m_actuatedDOFs + 6);

    if(!useNeckAsConstraint)
    {
        // get the neck weight
        tempValue = config.find("neckWeight");
        iDynTree::VectorDynSize neckWeight(3);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, neckWeight))
        {
            yError() << "[instantiateNeckSoftConstraint] Initialization failed while reading neckWeight.";
            return false;
        }


        std::shared_ptr<CartesianCostFunction> ptr;
        ptr = std::make_shared<CartesianCostFunction>(CartesianElement::Type::ORIENTATION);
        ptr->setSubMatricesStartingPosition(0, 0);

        ptr->setWeight(neckWeight);
        ptr->setBiasAcceleration(m_neckBiasAcceleration);
        ptr->setRoboticJacobian(m_neckJacobian);
        ptr->orientationController()->setGains(c0, kd, kp);

        m_costFunctions.insert(std::make_pair("neck", ptr));
        m_hessianMatrices.insert(std::make_pair("neck", std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("neck", std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }
    else
    {
        std::shared_ptr<CartesianConstraint> ptr;
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::ORIENTATION);
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);

        ptr->setBiasAcceleration(m_neckBiasAcceleration);
        ptr->setRoboticJacobian(m_neckJacobian);
        ptr->orientationController()->setGains(c0, kd, kp);

        m_constraints.insert(std::make_pair("neck", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();
    }
    return true;
}

bool TaskBasedTorqueSolver::instantiateRegularizationTaskConstraint(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    if(config.isNull())
    {
        yError() << "[instantiateRegularizationTaskConstraint] Empty configuration joint task constraint.";
        return false;
    }

    m_desiredJointPosition.resize(m_actuatedDOFs);
    m_desiredJointVelocity.resize(m_actuatedDOFs);
    m_desiredJointAcceleration.resize(m_actuatedDOFs);
    m_desiredJointVelocity.zero();
    m_desiredJointAcceleration.zero();

    tempValue = config.find("jointRegularization");
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, m_desiredJointPosition))
    {
        yError() << "[initialize] Unable to convert a YARP list to an iDynTree::VectorDynSize, "
                 << "joint regularization";
        return false;
    }

    iDynTree::toEigen(m_desiredJointPosition) = iDynTree::toEigen(m_desiredJointPosition) *
        iDynTree::deg2rad(1);

    // set the matrix related to the joint regularization
    tempValue = config.find("jointRegularizationWeights");
    iDynTree::VectorDynSize jointRegularizationWeights(m_actuatedDOFs);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, jointRegularizationWeights))
    {
        yError() << "Initialization failed while reading jointRegularizationWeights vector.";
        return false;
    }

    // set the matrix related to the joint regularization
    tempValue = config.find("proportionalGains");
    iDynTree::VectorDynSize proportionalGains(m_actuatedDOFs);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, proportionalGains))
    {
        yError() << "Initialization failed while reading proportionalGains vector.";
        return false;
    }

    iDynTree::VectorDynSize derivativeGains(m_actuatedDOFs);
    bool useDefaultKd = config.check("useDefaultKd", yarp::os::Value("False")).asBool();

    if(useDefaultKd)
    {
        double scaling;
        if(!YarpHelper::getNumberFromSearchable(config, "scaling", scaling))
        {
            yError() << "[initialize] Unable to get the scaling factor.";
            return false;
        }

        iDynTree::toEigen(derivativeGains) = 2 / scaling
            * iDynTree::toEigen(proportionalGains).array().sqrt();
    }
    else
    {
        tempValue = config.find("derivativeGains");
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, derivativeGains))
        {
            yError() << "Initialization failed while reading derivativeGains vector.";
            return false;
        }
    }

    std::shared_ptr<JointRegularizationCostFunction> ptr;
    ptr = std::make_shared<JointRegularizationCostFunction>(m_actuatedDOFs);
    bool useJointRegularizationAsCostFunction = config.check("useAsCostFunction", yarp::os::Value("False")).asBool();
    if(useJointRegularizationAsCostFunction)
    {
        std::shared_ptr<JointRegularizationCostFunction> ptr;
        ptr = std::make_shared<JointRegularizationCostFunction>(m_actuatedDOFs);

        ptr->setSubMatricesStartingPosition(6, 0);

        ptr->setWeight(jointRegularizationWeights);
        ptr->setDerivativeGains(derivativeGains);
        ptr->setProportionalGains(proportionalGains);

        ptr->setDesiredJointPosition(m_desiredJointPosition);
        ptr->setDesiredJointVelocity(m_desiredJointVelocity);
        ptr->setDesiredJointAcceleration(m_desiredJointAcceleration);
        ptr->setJointPosition(m_jointPosition);
        ptr->setJointVelocity(m_jointVelocity);

        m_costFunctions.insert(std::make_pair("regularization_joint_costFunction", ptr));

        m_hessianMatrices.insert(std::make_pair("regularization_joint_costFunction", std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));

        m_gradientVectors.insert(std::make_pair("regularization_joint_costFunction", std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }

    bool useJointRegularizationAsConstraint = config.check("useAsConstraint", yarp::os::Value("False")).asBool();
    if(useJointRegularizationAsConstraint)
    {
        std::shared_ptr<JointRegularizationConstraint> ptr;
        ptr = std::make_shared<JointRegularizationConstraint>(m_actuatedDOFs);

        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 6);
        ptr->setDerivativeGains(derivativeGains);
        ptr->setProportionalGains(proportionalGains);

        ptr->setDesiredJointPosition(m_desiredJointPosition);
        ptr->setDesiredJointVelocity(m_desiredJointVelocity);
        ptr->setDesiredJointAcceleration(m_desiredJointAcceleration);
        ptr->setJointPosition(m_jointPosition);
        ptr->setJointVelocity(m_jointVelocity);

        m_constraints.insert(std::make_pair("regularization_joint_constraint", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();
    }

    return true;
}

bool TaskBasedTorqueSolver::instantiateTorqueRegularizationConstraint(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    if(config.isNull())
    {
        yError() << "[instantiateRegularizationTaskConstraint] Empty configuration torque constraint.";
        return false;
    }

    tempValue = config.find("regularizationWeights");
    iDynTree::VectorDynSize torqueRegularizationWeights(m_actuatedDOFs);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, torqueRegularizationWeights))
    {
        yError() << "Initialization failed while reading torqueRegularizationWeights vector.";
        return false;
    }

    std::shared_ptr<InputRegularizationTerm> ptr;
    ptr = std::make_shared<InputRegularizationTerm>(m_actuatedDOFs);
    ptr->setSubMatricesStartingPosition(6 + m_actuatedDOFs, 0);
    ptr->setWeight(torqueRegularizationWeights);

    m_costFunctions.insert(std::make_pair("regularization_torque", ptr));

    m_hessianMatrices.insert(std::make_pair("regularization_torque", std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));

    m_gradientVectors.insert(std::make_pair("regularization_torque", std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));

    return true;
}

void TaskBasedTorqueSolver::instantiateJointsPositionLimit(const yarp::os::Searchable& config,
                                                           const int& actuatedDOFs,
                                                           const iDynTree::VectorDynSize& jointPositionUpperLimit,
                                                           const iDynTree::VectorDynSize& jointPositionLowerLimit)

{
    double samplingTime = config.check("sampling_time", yarp::os::Value(0.016)).asDouble();

    std::shared_ptr<JointsPositionLimit> ptr;
    ptr = std::make_shared<JointsPositionLimit>(samplingTime, m_actuatedDOFs);

    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 6);
    ptr->setJointsLimit(jointPositionUpperLimit, jointPositionLowerLimit);
    ptr->setJointPosition(m_jointPosition);
    ptr->setJointVelocity(m_jointVelocity);

    m_constraints.insert(std::make_pair("joints_limit_constraint", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

}

bool TaskBasedTorqueSolver::initializeSolver(const yarp::os::Searchable& config)
{

    bool verbosity = config.check("verbosity", yarp::os::Value(false)).asBool();
    int linearSystemSolver = config.check("linearSystemSolver", yarp::os::Value(0)).asInt();
    double maxIteraction = config.check("maxIteraction", yarp::os::Value(100000)).asDouble();
    double sigma = config.check("sigma", yarp::os::Value(0.0000001)).asDouble();
    double alpha = config.check("alpha", yarp::os::Value(1.6)).asDouble();
    double absoluteTolerance = config.check("absoluteTolerance", yarp::os::Value(0.0001)).asDouble();
    double relativeTolerance = config.check("relativeTolerance", yarp::os::Value(0.0001)).asDouble();
    double primalInfeasibilityTolerance = config.check("primalInfeasibilityTolerance", yarp::os::Value(0.00001)).asDouble();

    std::cout << " absoluteTolerance " << absoluteTolerance << std::endl;
    std::cout << " relativeTolerance " << relativeTolerance << std::endl;

    // resize
    // sparse matrix
    m_hessianEigen.resize(m_numberOfVariables, m_numberOfVariables);
    m_constraintMatrix.resize(m_numberOfConstraints, m_numberOfVariables);

    // dense vectors
    m_gradient = Eigen::VectorXd::Zero(m_numberOfVariables);
    m_lowerBound = Eigen::VectorXd::Zero(m_numberOfConstraints);
    m_upperBound = Eigen::VectorXd::Zero(m_numberOfConstraints);

    // initialize the optimization problem
    m_optimizer = std::make_unique<OsqpEigen::Solver>();
    m_optimizer->data()->setNumberOfVariables(m_numberOfVariables);
    m_optimizer->data()->setNumberOfConstraints(m_numberOfConstraints);

    m_optimizer->settings()->setVerbosity(verbosity);
    m_optimizer->settings()->setLinearSystemSolver(linearSystemSolver);
    m_optimizer->settings()->setMaxIteraction(maxIteraction);
    m_optimizer->settings()->setSigma(sigma);
    m_optimizer->settings()->setAlpha(alpha);
    m_optimizer->settings()->setAbsoluteTolerance(absoluteTolerance);
    m_optimizer->settings()->setRelativeTolerance(relativeTolerance);
    m_optimizer->settings()->setPrimalInfeasibilityTollerance(primalInfeasibilityTolerance);


}

bool TaskBasedTorqueSolver::initialize(const yarp::os::Searchable& config,
                                       const int& actuatedDOFs,
                                       const iDynTree::VectorDynSize& minJointTorque,
                                       const iDynTree::VectorDynSize& maxJointTorque,
                                       const iDynTree::VectorDynSize& jointPositionUpperLimit,
                                       const iDynTree::VectorDynSize& jointPositionLowerLimit)
{

    // m_profiler = std::make_unique<TimeProfiler>();
    // m_profiler->setPeriod(round(0.1 / 0.01));

    // m_profiler->addTimer("hessian");
    // m_profiler->addTimer("gradient");
    // m_profiler->addTimer("A");
    // m_profiler->addTimer("add A");
    // m_profiler->addTimer("bounds");
    // m_profiler->addTimer("solve");

    m_actuatedDOFs = actuatedDOFs;
    // depends on single or double support
    setNumberOfVariables();

    m_numberOfConstraints = 0;

    // resize matrices (generic)
    m_massMatrix.resize(m_actuatedDOFs + 6, m_actuatedDOFs + 6);
    m_generalizedBiasForces.resize(m_actuatedDOFs + 6);

    // results
    m_solution.resize(m_numberOfVariables);
    m_desiredJointTorque.resize(m_actuatedDOFs);
    m_desiredJointAccelerationOutput.resize(m_actuatedDOFs);

    // fine for double and single support
    m_desiredContactWrenches.resize(m_numberOfVariables - (2 * m_actuatedDOFs + 6));

    // check if the config is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for Task based torque solver.";
        return false;
    }

    instantiateJointsPositionLimit(config, actuatedDOFs, jointPositionUpperLimit, jointPositionLowerLimit);

    // instantiate constraints
    yarp::os::Bottle& comConstraintOptions = config.findGroup("COM");
    if(!instantiateCoMConstraint(comConstraintOptions))
    {
        yError() << "[initialize] Unable to instantiate the CoM constraint.";
        return false;
    }

    if(!instantiateCoMAsLinearMomentumConstraint(comConstraintOptions))
    {
        yError() << "[initialize] Unable to instantiate the CoM constraint as linear momentum.";
        return false;
    }

    yarp::os::Bottle& linearMomentumOptions = config.findGroup("LINEAR_MOMENTUM");
    instantiateLinearMomentumConstraint(linearMomentumOptions);
    if(!instantiateLinearMomentumCostFunction(linearMomentumOptions))
    {
        yError() << "[initialize] Unable to instantiate the linear momentum cost.";
        return false;
    }

    yarp::os::Bottle& angularMomentumConstraintOptions = config.findGroup("ANGULAR_MOMENTUM");
    if(!instantiateAngularMomentumConstraint(angularMomentumConstraintOptions))
    {
        yError() << "[initialize] Unable to instantiate the Angular Momentum constraint.";
        return false;
    }
    if(!instantiateAngularMomentumCostFunction(angularMomentumConstraintOptions))
    {
        yError() << "[initialize] Unable to instantiate the Angular Momentum constraint.";
        return false;
    }


    yarp::os::Bottle& feetConstraintOptions = config.findGroup("FEET");
    if(!instantiateFeetConstraint(feetConstraintOptions))
    {
        yError() << "[initialize] Unable to get the instantiate the feet constraints.";
        return false;
    }

    yarp::os::Bottle& ZMPConstraintOptions = config.findGroup("ZMP");
    if(!instantiateZMPConstraint(ZMPConstraintOptions))
    {
        yError() << "[initialize] Unable the instantiate the zmp constraints.";
        return false;
    }

    yarp::os::Bottle& contactForcesOption = config.findGroup("CONTACT_FORCES");
    if(!instantiateContactForcesConstraint(contactForcesOption))
    {
        yError() << "[initialize] Unable to get the instantiate the force feet constraints.";
        return false;
    }

    // instantiate cost function
    yarp::os::Bottle& neckOrientationOption = config.findGroup("NECK_ORIENTATION");
    if(!instantiateNeckSoftConstraint(neckOrientationOption))
    {
        yError() << "[initialize] Unable to get the instantiate the neck constraint.";
        return false;
    }

    yarp::os::Bottle& regularizationTaskOption = config.findGroup("REGULARIZATION_TASK");
    if(!instantiateRegularizationTaskConstraint(regularizationTaskOption))
    {
        yError() << "[initialize] Unable to get the instantiate the regularization constraint.";
        return false;
    }

    yarp::os::Bottle& regularizationTorqueOption = config.findGroup("REGULARIZATION_TORQUE");
    if(!instantiateTorqueRegularizationConstraint(regularizationTorqueOption))
    {
        yError() << "[initialize] Unable to get the instantiate the regularization torque constraint.";
        return false;
    }

    yarp::os::Bottle& regularizationForceOption = config.findGroup("REGULARIZATION_FORCE");
    if(!instantiateForceRegularizationConstraint(regularizationForceOption))
    {
        yError() << "[initialize] Unable to get the instantiate the regularization force constraint.";
        return false;
    }

    instantiateSystemDynamicsConstraint();

    yarp::os::Bottle& rateOfChangeOption = config.findGroup("RATE_OF_CHANGE");
    if(!instantiateJointRateOfChangeConstraint(rateOfChangeOption))
    {
        yError() << "[initialize] Unable to get the instantiate the rate of change constraint.";
        return false;
    }

    if(!instantiateContactWrenchesRateOfChangeConstraint(rateOfChangeOption))
    {
        yError() << "[initialize] Unable to get the instantiate the rate of change constraint for the contact wrenches.";
        return false;
    }

    yarp::os::Bottle& motorReflectedInertiaOption = config.findGroup("MOTOR_REFLECTED_INERTIA");
    if(!instantiateMotorReflectedInertia(motorReflectedInertiaOption))
    {
        yError() << "[initialize] Unable to get the instantiate the motor reflected inertia.";
        return false;
    }


    this->initializeSolver(config);

    // set constant element of the cost function
    for(const auto& element: m_costFunctions)
    {
        std::string key = element.first;
        element.second->setHessianConstantElements(*(m_hessianMatrices.at(key)));
        element.second->setGradientConstantElemenets(*(m_gradientVectors.at(key)));
    }

    // set constant element of the constraint
    for(const auto& constraint: m_constraints)
    {
        constraint.second->setJacobianConstantElements(m_constraintMatrix);
        constraint.second->setBoundsConstantElements(m_upperBound, m_lowerBound);
    }

    // print some usefull information
    yInfo() << "Total number of constraints " << m_numberOfConstraints;
    for(const auto& constraint: m_constraints)
        yInfo() << constraint.first << ": " << constraint.second->getNumberOfConstraints()
                << constraint.second->getJacobianStartingRow()
                << constraint.second->getJacobianStartingColumn();

    return true;
}



bool TaskBasedTorqueSolver::setMassMatrix(const iDynTree::MatrixDynSize& massMatrix)
{
    if(m_useMotorReflectedInertia)
        iDynTree::toEigen(m_massMatrix) = iDynTree::toEigen(massMatrix) + iDynTree::toEigen(m_reflectedInertia);
    else
        m_massMatrix = massMatrix;

    if(m_useLinearMomentumConstraint)
    {
        // if first time add robot mass
        if(!m_optimizer->isInitialized())
        {
            auto constraint = m_constraints.find("linear_momentum_constraint");
            if(constraint == m_constraints.end())
            {
                yError() << "[setMassMatrix] unable to find the linear momentum constraint. "
                         << "Please call 'initialize()' method";
                return false;
            }

            auto ptr = std::static_pointer_cast<LinearMomentumConstraint>(constraint->second);
            ptr->setRobotMass(m_massMatrix(0,0));
        }
    }

    if(m_useLinearMomentumCostFunction)
    {
        // if first time add robot mass
        if(!m_optimizer->isInitialized())
        {
            auto costFunction = m_costFunctions.find("linear_momentum_costFunction");
            if(costFunction ==  m_costFunctions.end())
            {
                yError() << "[setMassMatrix] unable to find the linear momentum cost function. "
                         << "Please call 'initialize()' method";
                return false;
            }

            auto ptr = std::static_pointer_cast<LinearMomentumCostFunction>(costFunction->second);
            ptr->setRobotMass(m_massMatrix(0,0));
        }
    }

    auto constraint = m_constraints.find("com_linear_momentum_constraint");
    if(constraint != m_constraints.end())
    {
        auto ptr = std::dynamic_pointer_cast<LinearMomentumElement>(constraint->second);
        ptr->setRobotMass(m_massMatrix(0,0));
    }

    return true;
}

void TaskBasedTorqueSolver::setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces)
{
    m_generalizedBiasForces = generalizedBiasForces;
}

bool TaskBasedTorqueSolver::setCentroidalTotalMomentum(const iDynTree::SpatialMomentum& centroidalTotalMomentum)
{
    if(m_useAngularMomentumConstraint)
    {
        auto constraint = m_constraints.find("angular_momentum_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setLinearAngularMomentum] unable to find the angular momentum constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(constraint->second);
        ptr->setAngularMomentum(centroidalTotalMomentum.getAngularVec3());
    }

    if(m_useAngularMomentumCostFunction)
    {
        auto costFunction = m_costFunctions.find("angular_momentum_costFunction");
        if(costFunction == m_costFunctions.end())
        {
            yError() << "[setCoMState] unable to find the angular momentum costFunction. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(costFunction->second);
        ptr->setAngularMomentum(centroidalTotalMomentum.getAngularVec3());
    }
    return true;
}

void TaskBasedTorqueSolver::setDesiredJointTrajectory(const iDynTree::VectorDynSize& desiredJointPosition,
                                                      const iDynTree::VectorDynSize& desiredJointVelocity,
                                                      const iDynTree::VectorDynSize& desiredJointAcceleration)
{

    m_desiredJointPosition = desiredJointPosition;
    m_desiredJointVelocity = desiredJointVelocity;
    m_desiredJointAcceleration = desiredJointAcceleration;
}

void TaskBasedTorqueSolver::setInternalRobotState(const iDynTree::VectorDynSize& jointPosition,
                                                  const iDynTree::VectorDynSize& jointVelocity)
{
    m_jointPosition = jointPosition;
    m_jointVelocity = jointVelocity;
}

bool TaskBasedTorqueSolver::setDesiredNeckTrajectory(const iDynTree::Rotation& desiredNeckOrientation,
                                                     const iDynTree::Vector3& desiredNeckVelocity,
                                                     const iDynTree::Vector3& desiredNeckAcceleration)
{

    auto cost = m_costFunctions.find("neck");
    if(cost != m_costFunctions.end())
    {
        auto ptr = std::static_pointer_cast<CartesianCostFunction>(cost->second);
        ptr->orientationController()->setDesiredTrajectory(desiredNeckAcceleration,
                                                           desiredNeckVelocity,
                                                           desiredNeckOrientation * m_additionalRotation);

        m_desiredNeckOrientation = desiredNeckOrientation * m_additionalRotation;
        return true;
    }

    auto constraint = m_constraints.find("neck");
    if(constraint != m_constraints.end())
    {
        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        ptr->orientationController()->setDesiredTrajectory(desiredNeckAcceleration,
                                                           desiredNeckVelocity,
                                                           desiredNeckOrientation * m_additionalRotation);

        m_desiredNeckOrientation = desiredNeckOrientation * m_additionalRotation;
        return true;
    }

    yError() << "[setDesiredNeckTrajectory] Unable to find neither the cost nor the constraint, this is not possibile";
    return false;
}

bool TaskBasedTorqueSolver::setNeckState(const iDynTree::Rotation& neckOrientation,
                                         const iDynTree::Twist& neckVelocity)
{
    auto cost = m_costFunctions.find("neck");
    if(cost != m_costFunctions.end())
    {

        auto ptr = std::static_pointer_cast<CartesianCostFunction>(cost->second);
        ptr->orientationController()->setFeedback(neckVelocity.getAngularVec3(), neckOrientation);

        return true;
    }

    auto constraint = m_constraints.find("neck");
    if(constraint != m_constraints.end())
    {
        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        ptr->orientationController()->setFeedback(neckVelocity.getAngularVec3(), neckOrientation);

        return true;
    }

    yError() << "[setDesiredNeckTrajectory] Unable to find neither the cost nor the constraint, this is not possibile";
    return false;
}

void TaskBasedTorqueSolver::setNeckJacobian(const iDynTree::MatrixDynSize& jacobian)
{
    iDynTree::toEigen(m_neckJacobian) = iDynTree::toEigen(jacobian).block(3, 0, 3, m_actuatedDOFs +6);
}

void TaskBasedTorqueSolver::setNeckBiasAcceleration(const iDynTree::Vector6 &biasAcceleration)
{
    // get only the angular part
    iDynTree::toEigen(m_neckBiasAcceleration) = iDynTree::toEigen(biasAcceleration).block(3, 0, 3, 1);
}


bool TaskBasedTorqueSolver::setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                                    const iDynTree::Vector3& comVelocity,
                                                    const iDynTree::Vector3& comAcceleration)
{
    if(m_useCoMConstraint)
    {
        auto constraint = m_constraints.find("com");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredCoMTrajectory] unable to find the linear momentum constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        ptr->positionController()->setDesiredTrajectory(comAcceleration, comVelocity, comPosition);
    }

    auto constraint = m_constraints.find("com_linear_momentum_constraint");
    if(constraint != m_constraints.end())
    {
        auto ptr = std::dynamic_pointer_cast<LinearMomentumElement>(constraint->second);
        ptr->positionController()->setDesiredTrajectory(comAcceleration, comVelocity, comPosition);
    }

    return true;
}

bool TaskBasedTorqueSolver::setCoMState(const iDynTree::Position& comPosition,
                                        const iDynTree::Vector3& comVelocity)
{
    if(m_useLinearMomentumConstraint)
    {
        // save com desired trajectory
        auto constraint = m_constraints.find("linear_momentum_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setCoMState] unable to find the linear momentum constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<LinearMomentumConstraint>(constraint->second);
        ptr->setCoMPosition(comPosition);
    }

    if(m_useLinearMomentumCostFunction)
    {
        // save com desired trajectory
        auto costFunction = m_costFunctions.find("linear_momentum_costFunction");
        if(costFunction == m_costFunctions.end())
        {
            yError() << "[setCoMState] unable to find the linear momentum costFunction. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<LinearMomentumCostFunction>(costFunction->second);
        ptr->setCoMPosition(comPosition);
    }

    if(m_useCoMConstraint)
    {
        auto constraint = m_constraints.find("com");
        if(constraint != m_constraints.end())
        {
            auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
            ptr->positionController()->setFeedback(comVelocity, comPosition);
        }
    }

    auto constraint = m_constraints.find("com_linear_momentum_constraint");
    if(constraint != m_constraints.end())
    {
        auto ptr = std::dynamic_pointer_cast<LinearMomentumElement>(constraint->second);
        ptr->positionController()->setFeedback(comVelocity, comPosition);
    }


    if(m_useAngularMomentumConstraint)
    {
        auto constraint = m_constraints.find("angular_momentum_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setCoMState] unable to find the angular momentum constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(constraint->second);
        ptr->setCoMPosition(comPosition);
    }

    if(m_useAngularMomentumCostFunction)
    {
        auto costFunction = m_costFunctions.find("angular_momentum_costFunction");
        if(costFunction == m_costFunctions.end())
        {
            yError() << "[setCoMState] unable to find the angular momentum costFunction. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(costFunction->second);
        ptr->setCoMPosition(comPosition);
    }
    return true;
}

void TaskBasedTorqueSolver::setCoMJacobian(const iDynTree::MatrixDynSize& comJacobian)
{
    if(m_useCoMConstraint)
    {
        if(!m_controlOnlyCoMHeight)
            m_comJacobian = comJacobian;
        else
            iDynTree::toEigen(m_comJacobian) = iDynTree::toEigen(comJacobian).block(2, 0, 1, m_actuatedDOFs + 6);
    }
}

void TaskBasedTorqueSolver::setCoMBiasAcceleration(const iDynTree::Vector3 &comBiasAcceleration)
{
    if(m_useCoMConstraint)
    {
        if(!m_controlOnlyCoMHeight)
            iDynTree::toEigen(m_comBiasAcceleration) = iDynTree::toEigen(comBiasAcceleration);
        else
            m_comBiasAcceleration(0) = comBiasAcceleration(2);
    }
}

bool TaskBasedTorqueSolver::setDesiredVRP(const iDynTree::Vector3 &vrp)
{
    if(m_useLinearMomentumConstraint)
    {
        auto constraint = m_constraints.find("linear_momentum_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredVRP] Unable to find the linear_momentum constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }
        auto ptr = std::static_pointer_cast<LinearMomentumConstraint>(constraint->second);
        ptr->setDesiredVRP(vrp);
    }

    if(m_useLinearMomentumCostFunction)
    {
        auto costFunction = m_costFunctions.find("linear_momentum_costFunction");
        if(costFunction == m_costFunctions.end())
        {
            yError() << "[setDesiredVRP] Unable to find the linear_momentum costFunction. "
                     << "Please call 'initialize()' method";
            return false;
        }
        auto ptr = std::static_pointer_cast<LinearMomentumCostFunction>(costFunction->second);
        ptr->setDesiredVRP(vrp);
    }
    return true;
}

bool TaskBasedTorqueSolver::setDesiredZMP(const iDynTree::Vector2 &zmp)
{
    if(m_useZMPConstraint)
    {
        std::shared_ptr<ZMPConstraint> ptr;

        auto constraint = m_constraints.find("zmp");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredZMP] Unable to find the zmp constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }
        ptr = std::static_pointer_cast<ZMPConstraint>(constraint->second);
        ptr->setDesiredZMP(zmp);
    }
    return true;
}

bool TaskBasedTorqueSolver::setMeasuredZMP(const iDynTree::Vector2 &zmp)
{
    if(m_useZMPConstraint)
    {
        std::shared_ptr<ZMPConstraint> ptr;

        auto constraint = m_constraints.find("zmp");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredZMP] Unable to find the zmp constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }
        ptr = std::static_pointer_cast<ZMPConstraint>(constraint->second);
        ptr->setMeasuredZMP(zmp);
    }
    return true;
}

bool TaskBasedTorqueSolverDoubleSupport::setFeetWeightPercentage(const double &weightInLeft,
                                                                 const double &weightInRight)
{
    iDynTree::VectorDynSize weightLeft(6), weightRight(6);

    for(int i = 0; i < 6; i++)
    {
        weightLeft(i) = m_regularizationForceScale * std::fabs(weightInLeft)
            + m_regularizationForceOffset;

        weightRight(i) = m_regularizationForceScale * std::fabs(weightInRight)
            + m_regularizationForceOffset;
    }

    auto cost = m_costFunctions.find("regularization_left_force");
    if(cost == m_costFunctions.end())
    {
        yError() << "[setDesiredNeckTrajectory] unable to find the neck trajectory element. "
                 << "Please call 'initialize()' method";
        return false;
    }
    auto ptr = std::static_pointer_cast<InputRegularizationTerm>(cost->second);
    ptr->setWeight(weightLeft);

    cost = m_costFunctions.find("regularization_right_force");
    if(cost == m_costFunctions.end())
    {
        yError() << "[setDesiredNeckTrajectory] unable to find the neck trajectory element. "
                 << "Please call 'initialize()' method";
        return false;
    }
    ptr = std::static_pointer_cast<InputRegularizationTerm>(cost->second);
    ptr->setWeight(weightRight);

    return true;
}

bool TaskBasedTorqueSolver::setHessianMatrix()
{
    std::string key;
    Eigen::SparseMatrix<double> hessianEigen(m_numberOfVariables, m_numberOfVariables);
    for(const auto& element: m_costFunctions)
    {
        key = element.first;
        element.second->evaluateHessian(*(m_hessianMatrices.at(key)));
        hessianEigen += *(m_hessianMatrices.at(key));
    }

    if(m_optimizer->isInitialized())
    {
        if(!m_optimizer->updateHessianMatrix(hessianEigen))
        {
            yError() << "[setHessianMatrix] Unable to update the hessian matrix.";
            return false;
        }
    }
    else
    {
        if(!m_optimizer->data()->setHessianMatrix(hessianEigen))
        {
            yError() << "[setHessianMatrix] Unable to set first time the hessian matrix.";
            return false;
        }
    }

    m_hessianEigen = hessianEigen;

    return true;
}

bool TaskBasedTorqueSolver::setGradientVector()
{
    std::string key;
    m_gradient = MatrixXd::Zero(m_numberOfVariables, 1);
    for(const auto& element: m_costFunctions)
    {
        key = element.first;
        element.second->evaluateGradient(*(m_gradientVectors.at(key)));
        m_gradient += *(m_gradientVectors.at(key));
    }

    if(m_optimizer->isInitialized())
    {
        if(!m_optimizer->updateGradient(m_gradient))
        {
            yError() << "[setGradient] Unable to update the gradient.";
            return false;
        }
    }
    else
    {
        if(!m_optimizer->data()->setGradient(m_gradient))
        {
            yError() << "[setGradient] Unable to set first time the gradient.";
            return false;
        }
    }

    return true;
}

bool TaskBasedTorqueSolver::setLinearConstraintMatrix()
{
    for(const auto& constraint: m_constraints)
    {
        // m_profiler->setInitTime("add A");
        constraint.second->evaluateJacobian(m_constraintMatrix);
        // m_profiler->setEndTime("add A");
    }

    if(m_optimizer->isInitialized())
    {
        if(!m_optimizer->updateLinearConstraintsMatrix(m_constraintMatrix))
        {
            yError() << "[setLinearConstraintsMatrix] Unable to update the constraints matrix.";
            return false;
        }
    }
    else
    {
        if(!m_optimizer->data()->setLinearConstraintsMatrix(m_constraintMatrix))
        {
            yError() << "[setLinearConstraintsMatrix] Unable to set the constraints matrix.";
            return false;
        }
    }

    return true;
}

bool TaskBasedTorqueSolver::setBounds()
{
    for(const auto& constraint: m_constraints)
        constraint.second->evaluateBounds(m_upperBound, m_lowerBound);

    if(m_optimizer->isInitialized())
    {
        if(!m_optimizer->updateBounds(m_lowerBound, m_upperBound))
        {
            yError() << "[setBounds] Unable to update the bounds.";
            return false;
        }
    }
    else
    {
        if(!m_optimizer->data()->setLowerBound(m_lowerBound))
        {
            yError() << "[setBounds] Unable to set the first time the lower bound.";
            return false;
        }

        if(!m_optimizer->data()->setUpperBound(m_upperBound))
        {
            yError() << "[setBounds] Unable to set the first time the upper bound.";
            return false;
        }
    }
    return true;
}

bool TaskBasedTorqueSolver::solve()
{
    // m_profiler->setInitTime("hessian");

    if(!setHessianMatrix())
    {
        yError() << "[solve] Unable to set the hessian matrix.";
        return false;
    }

    // m_profiler->setEndTime("hessian");

    // m_profiler->setInitTime("gradient");

    if(!setGradientVector())
    {
        yError() << "[solve] Unable to set the gradient vector matrix.";
        return false;
    }

    // m_profiler->setEndTime("gradient");

    // m_profiler->setInitTime("A");

    if(!setLinearConstraintMatrix())
    {
        yError() << "[solve] Unable to set the linear constraint matrix.";
        return false;
    }

    // m_profiler->setEndTime("A");

    // m_profiler->setInitTime("bounds");
    if(!setBounds())
    {
        yError() << "[solve] Unable to set the bounds.";
        return false;
    }

    // m_profiler->setEndTime("bounds");

    // m_profiler->setInitTime("solve");
    if(!m_optimizer->isInitialized())
    {
        if(!m_optimizer->initSolver())
        {

            // Eigen::MatioFile file("data.mat");
            // file.write_mat("hessian", Eigen::MatrixXd(m_hessianEigen));
            // file.write_mat("gradient", Eigen::MatrixXd(m_gradient));
            // file.write_mat("constraint", Eigen::MatrixXd(m_constraintMatrix));
            // file.write_mat("lowerBound", Eigen::MatrixXd(m_lowerBound));
            // file.write_mat("upperBound", Eigen::MatrixXd(m_upperBound));
            // file.write_mat("massMatrix", iDynTree::toEigen(m_massMatrix));
            // file.write_mat("comJacobian", iDynTree::toEigen(m_comJacobian));

            // for(const auto& element: m_costFunctions)
            // {
            //     std::string key = element.first;
            //     std::string hessianKey = key + "_hessian";
            //     std::string gradientKey = key + "_gradient";
            //     file.write_mat(hessianKey.c_str(), Eigen::MatrixXd(*(m_hessianMatrices.at(key))));
            //     file.write_mat(gradientKey.c_str(), Eigen::MatrixXd(*(m_gradientVectors.at(key))));
            // }


            yError() << "[solve] Unable to initialize the solver";
            return false;
        }
    }

    if(!m_optimizer->solve())
    {
        // Eigen::MatioFile file("data.mat");
        // file.write_mat("hessian", Eigen::MatrixXd(m_hessianEigen));
        // file.write_mat("gradient", Eigen::MatrixXd(m_gradient));
        // file.write_mat("constraint", Eigen::MatrixXd(m_constraintMatrix));
        // file.write_mat("lowerBound", Eigen::MatrixXd(m_lowerBound));
        // file.write_mat("upperBound", Eigen::MatrixXd(m_upperBound));
        // file.write_mat("massMatrix", iDynTree::toEigen(m_massMatrix));
        // file.write_mat("comJacobian", iDynTree::toEigen(m_comJacobian));

        // for(const auto& element: m_costFunctions)
        // {
        //     std::string key = element.first;
        //     std::string hessianKey = key + "_hessian";
        //     std::string gradientKey = key + "_gradient";
        //     file.write_mat(hessianKey.c_str(), Eigen::MatrixXd(*(m_hessianMatrices.at(key))));
        //     file.write_mat(gradientKey.c_str(), Eigen::MatrixXd(*(m_gradientVectors.at(key))));
        // }


        yError() << "[solve] Unable to solve the problem.";
        return false;
    }

    // if(m_isSingleSupport)
    // {
    //     Eigen::MatioFile file("data.mat");
    //     file.write_mat("hessian", Eigen::MatrixXd(m_hessianEigen));
    //     file.write_mat("gradient", Eigen::MatrixXd(m_gradient));
    //     file.write_mat("constraint", Eigen::MatrixXd(m_constraintMatrix));
    //     file.write_mat("lowerBound", Eigen::MatrixXd(m_lowerBound));
    //     file.write_mat("upperBound", Eigen::MatrixXd(m_upperBound));
    //     file.write_mat("massMatrix", iDynTree::toEigen(m_massMatrix));
    //     file.write_mat("comJacobian", iDynTree::toEigen(m_comJacobian));

    //     for(const auto& element: m_costFunctions)
    //     {
    //         std::string key = element.first;
    //         std::string hessianKey = key + "_hessian";
    //         std::string gradientKey = key + "_gradient";
    //         file.write_mat(hessianKey.c_str(), Eigen::MatrixXd(*(m_hessianMatrices.at(key))));
    //         file.write_mat(gradientKey.c_str(), Eigen::MatrixXd(*(m_gradientVectors.at(key))));
    //     }

    //     return false;
    // }

    iDynTree::toEigen(m_solution) = m_optimizer->getSolution();

    // check equality constraints
    // if(!isSolutionFeasible())
    // {
    //     yError() << "[solve] The solution is not feasible.";
    //     return false;
    // }

    for(int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointTorque(i) = m_solution(i + m_actuatedDOFs + 6) +  0*m_desiredJointVelocity(i);

    for(int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointAccelerationOutput(i) = m_solution(i + 6);

    for(int i = 0; i < m_numberOfVariables - (2 * m_actuatedDOFs + 6); i++)
        m_desiredContactWrenches(i) = m_solution(i + 2 * m_actuatedDOFs + 6);

    if(!tempPrint())
        return false;

    // Eigen::VectorXd product;
    // auto leftWrench = getLeftWrench();
    // auto rightWrench = getRightWrench();
    // Eigen::VectorXd wrenches(12);
    // wrenches.block(0,0,6,1) = iDynTree::toEigen(leftWrench);
    // wrenches.block(6,0,6,1) = iDynTree::toEigen(rightWrench);

    // auto constraint = m_constraints.find("angular_momentum");
    // if(constraint == m_constraints.end())
    // {
    //     yError() << "[setLinearAngularMomentum] unable to find the linear constraint. "
    //              << "Please call 'initialize()' method";
    //     return false;
    // }


    // yInfo() << "zmp jacobian starting row " << constraint->second->getJacobianStartingRow();
    // yInfo() << "zmp jacobian starting column " << constraint->second->getJacobianStartingColumn();
    // Eigen::MatrixXd JacobianZMP;
    // JacobianZMP = m_constraintMatrix.block(constraint->second->getJacobianStartingRow(),
    //                                             0,
    //                                             3, m_numberOfVariables);


    // product = JacobianZMP * m_solution;

    // std::cerr << "Jacobian \n" << JacobianZMP << "\n";
    // std::cerr << "product \n" << product << "\n";

    // std::cerr << "upper bounds \n"
    //           << m_upperBound.block(constraint->second->getJacobianStartingRow(),
    //                                 0, 3, 1)<< "\n";

    // std::cerr << "lower bounds \n"
    //           << m_lowerBound.block(constraint->second->getJacobianStartingRow(),
    //                                 0, 3, 1)<< "\n";


    // Eigen::VectorXd product;
    // auto leftWrench = getLeftWrench();
    // auto rightWrench = getRightWrench();
    // Eigen::VectorXd wrenches(12);
    // wrenches.block(0,0,6,1) = iDynTree::toEigen(leftWrench);
    // wrenches.block(6,0,6,1) = iDynTree::toEigen(rightWrench);

    // auto constraint = m_constraints.find("zmp");
    // if(constraint == m_constraints.end())
    // {
    //     yError() << "[setLinearAngularMomentum] unable to find the linear constraint. "
    //              << "Please call 'initialize()' method";
    //     return false;
    // }


    // yInfo() << "zmp jacobian starting row " << constraint->second->getJacobianStartingRow();
    // yInfo() << "zmp jacobian starting column " << constraint->second->getJacobianStartingColumn();
    // Eigen::MatrixXd JacobianZMP(2,12);
    // JacobianZMP = m_constraintMatrix.block(constraint->second->getJacobianStartingRow(),
    //                                             constraint->second->getJacobianStartingColumn(),
    //                                             2, 12);


    // product = JacobianZMP * wrenches;

    // std::cerr << "JacaobianZMP \n" << JacobianZMP << "\n";
    // std::cerr << "product \n" << product << "\n";

    // std::cerr << "upper bounds \n"
    //           << m_upperBound.block(constraint->second->getJacobianStartingRow(),
    //                                 0, 2, 1)<< "\n";

    // std::cerr << "lower bounds \n"
    //           << m_lowerBound.block(constraint->second->getJacobianStartingRow(),
    //                                 0, 2, 1)<< "\n";

    // m_profiler->setEndTime("solve");

    // m_profiler->profiling();

    return true;
}

bool TaskBasedTorqueSolver::isSolutionFeasible()
{
    double tolerance = 0.5;
    Eigen::VectorXd constrainedOutput = m_constraintMatrix * iDynTree::toEigen(m_solution);
    // std::cerr<<"m_constraintMatrix"<<std::endl;
    // std::cerr<<Eigen::MatrixXd(m_constraintMatrix)<<std::endl;

    // std::cerr<<"upper\n";
    // std::cerr<<constrainedOutput - m_upperBound<<std::endl;

    // std::cerr<<"lower\n";
    // std::cerr<<constrainedOutput - m_lowerBound<<std::endl;

    // std::cerr<<"solution\n";
    // std::cerr<<m_solution<<"\n";

    if(((constrainedOutput - m_upperBound).maxCoeff() < tolerance)
       && ((constrainedOutput - m_lowerBound).minCoeff() > -tolerance))
        return true;

    yError() << "[isSolutionFeasible] The constraints are not satisfied.";
    return false;
}

const iDynTree::VectorDynSize& TaskBasedTorqueSolver::desiredJointTorque() const
{
    return m_desiredJointTorque;
}

const iDynTree::VectorDynSize& TaskBasedTorqueSolver::desiredJointAcceleration() const
{
    return m_desiredJointAccelerationOutput;
}

const iDynTree::VectorDynSize& TaskBasedTorqueSolver::solution() const
{
    return m_solution;
}

iDynTree::Wrench TaskBasedTorqueSolverDoubleSupport::getLeftWrench()
{
    iDynTree::Wrench wrench;
    for(int i = 0; i < 6; i++)
        wrench(i) = m_solution(6 + m_actuatedDOFs + m_actuatedDOFs + i);

    return wrench;
}

iDynTree::Wrench TaskBasedTorqueSolverDoubleSupport::getRightWrench()
{

    iDynTree::Wrench wrench;
    for(int i = 0; i < 6; i++)
        wrench(i) = m_solution(6 + m_actuatedDOFs + m_actuatedDOFs + 6 + i);

    return wrench;
}

iDynTree::Vector3 TaskBasedTorqueSolver::getDesiredNeckOrientation()
{
    return m_desiredNeckOrientation.asRPY();
}

void TaskBasedTorqueSolver::setInitialValue(const iDynTree::VectorDynSize initialValue)
{
    m_solution = initialValue;
    for(int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointTorque(i) = m_solution(i + m_actuatedDOFs + 6);

    for(int i = 0; i < m_numberOfVariables - (2 * m_actuatedDOFs + 6); i++)
        m_desiredContactWrenches(i) = m_solution(i + 2 * m_actuatedDOFs + 6);

    // TODO change API osqp
    if(m_optimizer->isInitialized())
    {
        Eigen::VectorXd init = iDynTree::toEigen(initialValue);
        m_optimizer->setPrimalVariable<double, Eigen::Dynamic>(init);
    }
}

bool TaskBasedTorqueSolverDoubleSupport::instantiateFeetConstraint(const yarp::os::Searchable& config)
{
    m_controlContact = config.check("controlContact", yarp::os::Value("False")).asBool();
    m_isSingleSupport = false;
    if(config.isNull())
    {
        yError() << "[instantiateFeetConstraint] Empty configuration file.";
        return false;
    }

    bool useDefaultKd = config.check("useDefaultKd", yarp::os::Value("False")).asBool();

    iDynTree::Vector3 kpLinear;
    auto tempValue = config.find("kpLinear");
    if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kpLinear))
    {
        yError() << "[instantiateFeetConstraint] Unable to get proportional gain";
        return false;
    }

    iDynTree::Vector3 kdLinear;
    if(useDefaultKd)
    {
        double scaling;
        if(!YarpHelper::getNumberFromSearchable(config, "scaling", scaling))
        {
            yError() << "[initialize] Unable to get the scaling factor.";
            return false;
        }

        iDynTree::toEigen(kdLinear) = 2 / scaling * iDynTree::toEigen(kpLinear).array().sqrt();
    }
    else
    {
        tempValue = config.find("kdLinear");
        if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kdLinear))
        {
            yError() << "[instantiateFeetConstraint] Unable to get derivative gain";
            return false;
        }
    }

    double c0, kpAngular, kdAngular;
    if(!YarpHelper::getNumberFromSearchable(config, "c0", c0))
        return false;

    if(!YarpHelper::getNumberFromSearchable(config, "kpAngular", kpAngular))
        return false;

    if(useDefaultKd)
    {
        double scaling;
        if(!YarpHelper::getNumberFromSearchable(config, "scaling", scaling))
        {
            yError() << "[initialize] Unable to get the scaling factor.";
            return false;
        }

        kdAngular = 2 / scaling * std::sqrt(kpAngular);
    }
    else
    {
        if(!YarpHelper::getNumberFromSearchable(config, "kdAngular", kdAngular))
            return false;
    }

    std::shared_ptr<CartesianConstraint> ptr;

    // left foot
    // resize quantities
    m_leftFootJacobian.resize(6, m_actuatedDOFs + 6);
    m_leftFootBiasAcceleration.resize(6);
    if(m_controlContact)
    {
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::POSE);
        ptr->positionController()->setGains(kpLinear, kdLinear);
        ptr->orientationController()->setGains(c0, kdAngular, kpAngular);
    }
    else
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::CONTACT);

    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
    ptr->setRoboticJacobian(m_leftFootJacobian);
    ptr->setBiasAcceleration(m_leftFootBiasAcceleration);

    m_constraints.insert(std::make_pair("left_foot", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    // right foot
    // resize quantities
    m_rightFootJacobian.resize(6, m_actuatedDOFs + 6);
    m_rightFootBiasAcceleration.resize(6);

    if(m_controlContact)
    {
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::POSE);
        ptr->positionController()->setGains(kpLinear, kdLinear);
        ptr->orientationController()->setGains(c0, kdAngular, kpAngular);
    }
    else
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::CONTACT);

    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
    ptr->setRoboticJacobian(m_rightFootJacobian);
    ptr->setBiasAcceleration(m_rightFootBiasAcceleration);

    m_constraints.insert(std::make_pair("right_foot", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

bool TaskBasedTorqueSolverDoubleSupport::instantiateZMPConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yInfo() << "[instantiateZMPConstraint] Empty configuration file. The ZMP Constraint will not be used";
        m_useZMPConstraint = false;
        return true;
    }
    m_useZMPConstraint = true;

    iDynTree::Vector2 kp;
    yarp::os::Value tempValue = config.find("kp");
    if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kp))
    {
        yError() << "[TaskBasedTorqueSolverDoubleSupport::instantiateZMPConstraint] Initialization failed while reading kp vector.";
        return false;
    }

    auto ptr = std::make_shared<ZMPConstraintDoubleSupport>();
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 6 + m_actuatedDOFs + m_actuatedDOFs);
    ptr->setLeftFootToWorldTransform(m_leftFootToWorldTransform);
    ptr->setRightFootToWorldTransform(m_rightFootToWorldTransform);
    ptr->setKp(kp);

    m_constraints.insert(std::make_pair("zmp", ptr));

    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

void TaskBasedTorqueSolverDoubleSupport::instantiateSystemDynamicsConstraint()
{
    auto ptr = std::make_shared<SystemDynamicConstraintDoubleSupport>(m_actuatedDOFs);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
    ptr->setLeftFootJacobian(m_leftFootJacobian);
    ptr->setRightFootJacobian(m_rightFootJacobian);
    ptr->setMassMatrix(m_massMatrix);
    ptr->setGeneralizedBiasForces(m_generalizedBiasForces);

    m_constraints.insert(std::make_pair("system_dynamics", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();
}

bool TaskBasedTorqueSolverDoubleSupport::instantiateContactForcesConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yError() << "[instantiateContactForcesConstraint] Empty configuration file.";
        return false;
    }

    double staticFrictionCoefficient;
    if(!YarpHelper::getNumberFromSearchable(config, "staticFrictionCoefficient",
                                            staticFrictionCoefficient))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    int numberOfPoints;
    if(!YarpHelper::getNumberFromSearchable(config, "numberOfPoints", numberOfPoints))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    double torsionalFrictionCoefficient;
    if(!YarpHelper::getNumberFromSearchable(config, "torsionalFrictionCoefficient",
                                            torsionalFrictionCoefficient))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    // feet dimensions
    yarp::os::Value feetDimensions = config.find("foot_size");
    if(feetDimensions.isNull() || !feetDimensions.isList())
    {
        yError() << "Please set the foot_size in the configuration file.";
        return false;
    }

    yarp::os::Bottle *feetDimensionsPointer = feetDimensions.asList();
    if(!feetDimensionsPointer || feetDimensionsPointer->size() != 2)
    {
        yError() << "Error while reading the feet dimensions. Wrong number of elements.";
        return false;
    }

    yarp::os::Value& xLimits = feetDimensionsPointer->get(0);
    if(xLimits.isNull() || !xLimits.isList())
    {
        yError() << "Error while reading the X limits.";
        return false;
    }

    yarp::os::Bottle *xLimitsPtr = xLimits.asList();
    if(!xLimitsPtr || xLimitsPtr->size() != 2)
    {
        yError() << "Error while reading the X limits. Wrong dimensions.";
        return false;
    }

    iDynTree::Vector2 footLimitX;
    footLimitX(0) = xLimitsPtr->get(0).asDouble();
    footLimitX(1) = xLimitsPtr->get(1).asDouble();

    yarp::os::Value& yLimits = feetDimensionsPointer->get(1);
    if(yLimits.isNull() || !yLimits.isList())
    {
        yError() << "Error while reading the Y limits.";
        return false;
    }

    yarp::os::Bottle *yLimitsPtr = yLimits.asList();
    if(!yLimitsPtr || yLimitsPtr->size() != 2)
    {
        yError() << "Error while reading the Y limits. Wrong dimensions.";
        return false;
    }

    iDynTree::Vector2 footLimitY;
    footLimitY(0) = yLimitsPtr->get(0).asDouble();
    footLimitY(1) = yLimitsPtr->get(1).asDouble();

    double minimalNormalForce;
    if(!YarpHelper::getNumberFromSearchable(config, "minimalNormalForce", minimalNormalForce))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    std::shared_ptr<ForceConstraint> ptr;

    // left foot
    ptr = std::make_shared<ForceConstraint>(numberOfPoints, staticFrictionCoefficient,
                                            torsionalFrictionCoefficient, minimalNormalForce,
                                            footLimitX, footLimitY);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 2 * m_actuatedDOFs + 6);
    ptr->setFootToWorldTransform(m_leftFootToWorldTransform);

    m_constraints.insert(std::make_pair("left_force", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    // right foot
    ptr = std::make_shared<ForceConstraint>(numberOfPoints, staticFrictionCoefficient,
                                            torsionalFrictionCoefficient, minimalNormalForce,
                                            footLimitX, footLimitY);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 2 * m_actuatedDOFs + 6 + 6);
    ptr->setFootToWorldTransform(m_rightFootToWorldTransform);

    m_constraints.insert(std::make_pair("right_force", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

bool TaskBasedTorqueSolverDoubleSupport::instantiateForceRegularizationConstraint(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    if(config.isNull())
    {
        yError() << "[instantiateRegularizationTaskConstraint] Empty configuration torque constraint.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "regularizationForceScale", m_regularizationForceScale))
    {
        yError() << "[instantiateForceRegularizationConstraint] Unable to get regularization force scale";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "regularizationForceOffset", m_regularizationForceOffset))
    {
        yError() << "[instantiateForceRegularizationConstraint] Unable to get regularization force offset";
        return false;
    }

    // left foot
    std::shared_ptr<InputRegularizationTerm> ptr;
    ptr = std::make_shared<InputRegularizationTerm>(6);
    ptr->setSubMatricesStartingPosition(6 + m_actuatedDOFs + m_actuatedDOFs, 0);
    m_costFunctions.insert(std::make_pair("regularization_left_force", ptr));
    m_hessianMatrices.insert(std::make_pair("regularization_left_force",
                                            std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
    m_gradientVectors.insert(std::make_pair("regularization_left_force",
                                            std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));

    // right foot
    ptr = std::make_shared<InputRegularizationTerm>(6);
    ptr->setSubMatricesStartingPosition(6 + m_actuatedDOFs + m_actuatedDOFs + 6, 0);
    m_costFunctions.insert(std::make_pair("regularization_right_force", ptr));
    m_hessianMatrices.insert(std::make_pair("regularization_right_force",
                                            std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
    m_gradientVectors.insert(std::make_pair("regularization_right_force",
                                            std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    return true;
}

void TaskBasedTorqueSolverDoubleSupport::instantiateLinearMomentumConstraint(const yarp::os::Searchable& config)
{
    m_useLinearMomentumConstraint = config.check("useAsConstraint", yarp::os::Value("False")).asBool();
    if(m_useLinearMomentumConstraint)
    {
        // memory allocation
        auto ptr = std::make_shared<LinearMomentumConstraint>(LinearMomentumConstraint::Type::DOUBLE_SUPPORT);
        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 6 + m_actuatedDOFs + m_actuatedDOFs);

        m_constraints.insert(std::make_pair("linear_momentum_constraint", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();
    }
    return;
}

bool TaskBasedTorqueSolverDoubleSupport::instantiateCoMAsLinearMomentumConstraint(const yarp::os::Searchable& config)
{
    bool asLinearMomentum = config.check("as_linear_momentum", yarp::os::Value("False")).asBool();
    if(asLinearMomentum)
    {
        // memory allocation
        auto ptr = std::make_shared<LinearMomentumConstraint>(LinearMomentumConstraint::Type::DOUBLE_SUPPORT, true);
        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 6 + m_actuatedDOFs + m_actuatedDOFs);

        bool useDefaultKd = config.check("useDefaultKd", yarp::os::Value("False")).asBool();

        yarp::os::Value tempValue;
        iDynTree::Vector3 kp;
        tempValue = config.find("kp");
        if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kp))
        {
            yError() << "[initialize] Unable to convert a YARP list to an iDynTree::VectorFixSize, "
                     << "joint regularization";
            return false;
        }

        iDynTree::Vector3 kd;
        if(useDefaultKd)
        {
            double scaling;
            if(!YarpHelper::getNumberFromSearchable(config, "scaling", scaling))
            {
                yError() << "[initialize] Unable to get the scaling factor.";
                return false;
            }
            iDynTree::toEigen(kd) = 2 / scaling * iDynTree::toEigen(kp).array().sqrt();
        }
        else
        {
            tempValue = config.find("kd");
            if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kd))
            {
                yError() << "[initialize] Unable to convert a YARP list to an iDynTree::VectorFixSize, "
                         << "joint regularization";
                return false;
            }
        }

        ptr->positionController()->setGains(kp, kd);
        m_constraints.insert(std::make_pair("com_linear_momentum_constraint", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();
    }
    return true;
}

bool TaskBasedTorqueSolverDoubleSupport::instantiateLinearMomentumCostFunction(const yarp::os::Searchable& config)
{
    m_useLinearMomentumCostFunction = config.check("useAsCostFunction", yarp::os::Value("False")).asBool();
    if(m_useLinearMomentumCostFunction)
    {
        yarp::os::Value tempValue = config.find("weight");
        iDynTree::VectorDynSize weight(3);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, weight))
        {
            yError() << "[TaskBasedTorqueSolverDoubleSupport::instantiateLinearMomentumCostFunction] Initialization failed while reading weight vector.";
            return false;
        }

        // memory allocation
        auto ptr = std::make_shared<LinearMomentumCostFunction>(LinearMomentumCostFunction::Type::DOUBLE_SUPPORT);
        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(6 + m_actuatedDOFs + m_actuatedDOFs, 0);
        ptr->setWeight(weight);

        m_costFunctions.insert(std::make_pair("linear_momentum_costFunction", ptr));

        m_hessianMatrices.insert(std::make_pair("linear_momentum_costFunction",
                                                std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("linear_momentum_costFunction",
                                                std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }
    return true;
}

void TaskBasedTorqueSolverDoubleSupport::setNumberOfVariables()
{
    // the optimization variable is given by
    // 1. base + joint acceleration (6 + m_actuatedDOFs)
    // 2. joint torque (m_actuatedDOFs)
    // 3. left and right foot contact wrench (6 + 6)

    m_numberOfVariables = 6 + m_actuatedDOFs + m_actuatedDOFs + 6 + 6;
}

void TaskBasedTorqueSolverDoubleSupport::setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                                                      const iDynTree::Twist& leftFootTwist,
                                                      const iDynTree::Transform& rightFootToWorldTransform,
                                                      const iDynTree::Twist& rightFootTwist)
{
    m_leftFootToWorldTransform = leftFootToWorldTransform;
    m_rightFootToWorldTransform = rightFootToWorldTransform;

    if(m_controlContact)
    {
        auto constraint = m_constraints.find("left_foot");
        if(constraint == m_constraints.end())
        {
            yError() << "[setFeetState] unable to find the left foot constraint. "
                     << "Please call 'initialize()' method";
            return ;
        }

        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        ptr->positionController()->setFeedback(leftFootTwist.getLinearVec3(),
                                               leftFootToWorldTransform.getPosition());


        ptr->orientationController()->setFeedback(leftFootTwist.getAngularVec3(),
                                                  leftFootToWorldTransform.getRotation());


        constraint = m_constraints.find("right_foot");
        if(constraint == m_constraints.end())
        {
            yError() << "[setFeetState] unable to find the right foot constraint. "
                     << "Please call 'initialize()' method";
            return ;
        }

        ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        ptr->positionController()->setFeedback(rightFootTwist.getLinearVec3(),
                                               rightFootToWorldTransform.getPosition());


        ptr->orientationController()->setFeedback(rightFootTwist.getAngularVec3(),
                                                  rightFootToWorldTransform.getRotation());
    }
}


bool TaskBasedTorqueSolverDoubleSupport::setDesiredFeetTrajectory(const iDynTree::Transform& leftFootToWorldTransform,
                                                                  const iDynTree::Transform& rightFootToWorldTransform)
{
    if(m_controlContact)
    {
        iDynTree::Vector3 footAcceleration;
        iDynTree::Twist footTwist;
        footTwist.zero();
        footAcceleration.zero();


        auto constraint = m_constraints.find("left_foot");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredFeetTrajectory] unable to find the left foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);

        ptr->positionController()->setDesiredTrajectory(footAcceleration,
                                                        footTwist.getLinearVec3(),
                                                        leftFootToWorldTransform.getPosition());


        ptr->orientationController()->setDesiredTrajectory(footAcceleration,
                                                           footTwist.getAngularVec3(),
                                                           leftFootToWorldTransform.getRotation());

        constraint = m_constraints.find("right_foot");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredFeetTrajectory] unable to find the right foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);

        ptr->positionController()->setDesiredTrajectory(footAcceleration,
                                                        footTwist.getLinearVec3(),
                                                        rightFootToWorldTransform.getPosition());


        ptr->orientationController()->setDesiredTrajectory(footAcceleration,
                                                           footTwist.getAngularVec3(),
                                                           rightFootToWorldTransform.getRotation());
    }

    return true;
}

void TaskBasedTorqueSolverDoubleSupport::setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                                                         const iDynTree::MatrixDynSize& rightFootJacobian)
{
    m_rightFootJacobian = rightFootJacobian;
    m_leftFootJacobian = leftFootJacobian;
}

void TaskBasedTorqueSolverDoubleSupport::setFeetBiasAcceleration(const iDynTree::Vector6 &leftFootBiasAcceleration,
                                                                 const iDynTree::Vector6 &rightFootBiasAcceleration)
{

    iDynTree::toEigen(m_leftFootBiasAcceleration) = iDynTree::toEigen(leftFootBiasAcceleration);
    iDynTree::toEigen(m_rightFootBiasAcceleration) = iDynTree::toEigen(rightFootBiasAcceleration);
}

iDynTree::Vector2 TaskBasedTorqueSolverDoubleSupport::getZMP()
{
    iDynTree::Position zmpLeft, zmpRight, zmpWorld;
    double zmpLeftDefined = 0.0, zmpRightDefined = 0.0;

    iDynTree::Vector2 zmp;

    auto leftWrench = getLeftWrench();
    auto rightWrench = getRightWrench();

    if(rightWrench.getLinearVec3()(2) < 10)
        zmpRightDefined = 0.0;
    else
    {
        zmpRight(0) = -rightWrench.getAngularVec3()(1) / rightWrench.getLinearVec3()(2);
        zmpRight(1) = rightWrench.getAngularVec3()(0) / rightWrench.getLinearVec3()(2);
        zmpRight(2) = 0.0;
        zmpRightDefined = 1.0;
    }

    if(leftWrench.getLinearVec3()(2) < 10)
        zmpLeftDefined = 0.0;
    else
    {
        zmpLeft(0) = -leftWrench.getAngularVec3()(1) / leftWrench.getLinearVec3()(2);
        zmpLeft(1) = leftWrench.getAngularVec3()(0) / leftWrench.getLinearVec3()(2);
        zmpLeft(2) = 0.0;
        zmpLeftDefined = 1.0;
    }

    double totalZ = rightWrench.getLinearVec3()(2) + leftWrench.getLinearVec3()(2);

    iDynTree::Transform leftTrans(iDynTree::Rotation::Identity(), m_leftFootToWorldTransform.getPosition());
    iDynTree::Transform rightTrans(iDynTree::Rotation::Identity(), m_rightFootToWorldTransform.getPosition());
    zmpLeft = leftTrans * zmpLeft;
    zmpRight = rightTrans * zmpRight;

    // the global zmp is given by a weighted average
    iDynTree::toEigen(zmpWorld) = ((leftWrench.getLinearVec3()(2) * zmpLeftDefined) / totalZ)
        * iDynTree::toEigen(zmpLeft) +
        ((rightWrench.getLinearVec3()(2) * zmpRightDefined)/totalZ) * iDynTree::toEigen(zmpRight);

    zmp(0) = zmpWorld(0);
    zmp(1) = zmpWorld(1);



    return zmp;
}

bool TaskBasedTorqueSolverSingleSupport::instantiateFeetConstraint(const yarp::os::Searchable& config)
{
    m_useSwingFootAsConstraint = config.check("useAsConstraint", yarp::os::Value("False")).asBool();
    m_useSwingFootAsCostFunction = config.check("useAsCostFunction", yarp::os::Value("False")).asBool();
    // m_controlContact = config.check("controlContact", yarp::os::Value("False")).asBool();
    m_controlContact = false;

    yInfo() << "m_useSwingFootAsConstraint " << m_useSwingFootAsConstraint;

    // TODO remove this line
    m_isSingleSupport = true;
    if(config.isNull())
    {
        yError() << "[instantiateFeetConstraint] Empty configuration file.";
        return false;
    }

    bool useDefaultKd = config.check("useDefaultKd", yarp::os::Value("False")).asBool();

    iDynTree::Vector3 kpLinear;
    auto tempValue = config.find("kpLinear");
    if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kpLinear))
    {
        yError() << "[instantiateFeetConstraint] Unable to get proportional gain";
        return false;
    }

    iDynTree::Vector3 kdLinear;
    if(useDefaultKd)
    {
        double scaling;
        if(!YarpHelper::getNumberFromSearchable(config, "scaling", scaling))
        {
            yError() << "[initialize] Unable to get the scaling factor.";
            return false;
        }

        iDynTree::toEigen(kdLinear) = 2 / scaling * iDynTree::toEigen(kpLinear).array().sqrt();
    }
    else
    {
        tempValue = config.find("kdLinear");
        if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kdLinear))
        {
            yError() << "[instantiateFeetConstraint] Unable to get derivative gain";
            return false;
        }
    }

    double c0, kpAngular, kdAngular;
    if(!YarpHelper::getNumberFromSearchable(config, "c0", c0))
        return false;

    if(!YarpHelper::getNumberFromSearchable(config, "kpAngular", kpAngular))
        return false;

    if(useDefaultKd)
    {
        double scaling;
        if(!YarpHelper::getNumberFromSearchable(config, "scaling", scaling))
        {
            yError() << "[initialize] Unable to get the scaling factor.";
            return false;
        }

        kdAngular = 2 / scaling * std::sqrt(kpAngular);
    }
    else
    {
        if(!YarpHelper::getNumberFromSearchable(config, "kdAngular", kdAngular))
            return false;
    }

    // stance_foot
    std::shared_ptr<CartesianConstraint> ptrConstraint;
    // resize quantities
    m_stanceFootJacobian.resize(6, m_actuatedDOFs + 6);
    m_stanceFootBiasAcceleration.resize(6);

    if(m_controlContact)
    {
        ptrConstraint = std::make_shared<CartesianConstraint>(CartesianElement::Type::POSE);
        ptrConstraint->positionController()->setGains(kpLinear, kdLinear);
        ptrConstraint->orientationController()->setGains(c0, kdAngular, kpAngular);
    }
    else
        ptrConstraint = std::make_shared<CartesianConstraint>(CartesianElement::Type::CONTACT);

    ptrConstraint->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
    ptrConstraint->setRoboticJacobian(m_stanceFootJacobian);
    ptrConstraint->setBiasAcceleration(m_stanceFootBiasAcceleration);
    m_constraints.insert(std::make_pair("stance_foot", ptrConstraint));
    m_numberOfConstraints += ptrConstraint->getNumberOfConstraints();

    iDynTree::VectorDynSize weight(6);
    tempValue = config.find("weight");
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, weight))
    {
        yError() << "[TaskBasedTorqueSolverSingleSupport::instantiateFeetConstraint] Initialization failed while reading weight vector.";
        return false;
    }

    // swing foot
    m_swingFootBiasAcceleration.resize(6);
    m_swingFootJacobian.resize(6, m_actuatedDOFs + 6);

    if(m_useSwingFootAsCostFunction)
    {
        auto ptrCostFunction = std::make_shared<CartesianCostFunction>(CartesianElement::Type::POSE);
        ptrCostFunction->setSubMatricesStartingPosition(0, 0);
        ptrCostFunction->setWeight(weight);
        ptrCostFunction->setRoboticJacobian(m_swingFootJacobian);
        ptrCostFunction->setBiasAcceleration(m_swingFootBiasAcceleration);
        ptrCostFunction->positionController()->setGains(kpLinear, kdLinear);
        ptrCostFunction->orientationController()->setGains(c0, kdAngular, kpAngular);
        m_costFunctions.insert(std::make_pair("swing_foot_costFunction", ptrCostFunction));
        m_hessianMatrices.insert(std::make_pair("swing_foot_costFunction",
                                                std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("swing_foot_costFunction",
                                                std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }

    if(m_useSwingFootAsConstraint)
    {
        auto ptrConstraint = std::make_shared<CartesianConstraint>(CartesianElement::Type::POSE);
        ptrConstraint->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
        ptrConstraint->positionController()->setGains(kpLinear, kdLinear);
        ptrConstraint->orientationController()->setGains(c0, kdAngular, kpAngular);
        ptrConstraint->setRoboticJacobian(m_swingFootJacobian);
        ptrConstraint->setBiasAcceleration(m_swingFootBiasAcceleration);
        m_constraints.insert(std::make_pair("swing_foot_constraint", ptrConstraint));
        m_numberOfConstraints += ptrConstraint->getNumberOfConstraints();
    }
    return true;
}

bool TaskBasedTorqueSolverSingleSupport::instantiateZMPConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yInfo() << "[instantiateZMPConstraint] Empty configuration file. The ZMP Constraint will not be used";
        m_useZMPConstraint = false;
        return true;
    }
    m_useZMPConstraint = true;

    iDynTree::Vector2 kp;
    yarp::os::Value tempValue = config.find("kp");
    if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kp))
    {
        yError() << "[TaskBasedTorqueSolverSingleSupport::instantiateZMPConstraint] Initialization failed while reading kp vector.";
        return false;
    }

    auto ptr = std::make_shared<ZMPConstraintSingleSupport>();
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 6 + m_actuatedDOFs + m_actuatedDOFs);
    ptr->setStanceFootToWorldTransform(m_stanceFootToWorldTransform);
    ptr->setKp(kp);

    m_constraints.insert(std::make_pair("zmp", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

void TaskBasedTorqueSolverSingleSupport::instantiateSystemDynamicsConstraint()
{
    auto ptr = std::make_shared<SystemDynamicConstraintSingleSupport>(m_actuatedDOFs);
    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
    ptr->setStanceFootJacobian(m_stanceFootJacobian);
    ptr->setMassMatrix(m_massMatrix);
    ptr->setGeneralizedBiasForces(m_generalizedBiasForces);

    m_constraints.insert(std::make_pair("system_dynamics", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();
}

bool TaskBasedTorqueSolverSingleSupport::instantiateContactForcesConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yError() << "[instantiateFeetConstraint] Empty configuration file.";
        return false;
    }

    double staticFrictionCoefficient;
    if(!YarpHelper::getNumberFromSearchable(config, "staticFrictionCoefficient",
                                            staticFrictionCoefficient))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    int numberOfPoints;
    if(!YarpHelper::getNumberFromSearchable(config, "numberOfPoints", numberOfPoints))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    double torsionalFrictionCoefficient;
    if(!YarpHelper::getNumberFromSearchable(config, "torsionalFrictionCoefficient",
                                            torsionalFrictionCoefficient))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    // feet dimensions
    yarp::os::Value feetDimensions = config.find("foot_size");
    if(feetDimensions.isNull() || !feetDimensions.isList())
    {
        yError() << "Please set the foot_size in the configuration file.";
        return false;
    }

    yarp::os::Bottle *feetDimensionsPointer = feetDimensions.asList();
    if(!feetDimensionsPointer || feetDimensionsPointer->size() != 2)
    {
        yError() << "Error while reading the feet dimensions. Wrong number of elements.";
        return false;
    }

    yarp::os::Value& xLimits = feetDimensionsPointer->get(0);
    if(xLimits.isNull() || !xLimits.isList())
    {
        yError() << "Error while reading the X limits.";
        return false;
    }

    yarp::os::Bottle *xLimitsPtr = xLimits.asList();
    if(!xLimitsPtr || xLimitsPtr->size() != 2)
    {
        yError() << "Error while reading the X limits. Wrong dimensions.";
        return false;
    }

    iDynTree::Vector2 footLimitX;
    footLimitX(0) = xLimitsPtr->get(0).asDouble();
    footLimitX(1) = xLimitsPtr->get(1).asDouble();

    yarp::os::Value& yLimits = feetDimensionsPointer->get(1);
    if(yLimits.isNull() || !yLimits.isList())
    {
        yError() << "Error while reading the Y limits.";
        return false;
    }

    yarp::os::Bottle *yLimitsPtr = yLimits.asList();
    if(!yLimitsPtr || yLimitsPtr->size() != 2)
    {
        yError() << "Error while reading the Y limits. Wrong dimensions.";
        return false;
    }

    iDynTree::Vector2 footLimitY;
    footLimitY(0) = yLimitsPtr->get(0).asDouble();
    footLimitY(1) = yLimitsPtr->get(1).asDouble();

    double minimalNormalForce;
    if(!YarpHelper::getNumberFromSearchable(config, "minimalNormalForce", minimalNormalForce))
    {
        yError() << "[initialize] Unable to get the number from searchable.";
        return false;
    }

    std::shared_ptr<ForceConstraint> ptr;

    // stance foot
    ptr = std::make_shared<ForceConstraint>(numberOfPoints, staticFrictionCoefficient,
                                            torsionalFrictionCoefficient, minimalNormalForce,
                                            footLimitX, footLimitY);

    ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 2 * m_actuatedDOFs + 6);
    ptr->setFootToWorldTransform(m_stanceFootToWorldTransform);

    m_constraints.insert(std::make_pair("stance_force", ptr));
    m_numberOfConstraints += ptr->getNumberOfConstraints();

    return true;
}

bool TaskBasedTorqueSolverSingleSupport::instantiateForceRegularizationConstraint(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    if(config.isNull())
    {
        yError() << "[instantiateRegularizationTaskConstraint] Empty configuration torque constraint.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "regularizationForceScale", m_regularizationForceScale))
    {
        yError() << "[instantiateForceRegularizationConstraint] Unable to get regularization force scale";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "regularizationForceOffset", m_regularizationForceOffset))
    {
        yError() << "[instantiateForceRegularizationConstraint] Unable to get regularization force offset";
        return false;
    }

    // the weight is constant in stance phase
    iDynTree::VectorDynSize weight(6);
    for(int i = 0; i < 6; i++)
        weight(i) = m_regularizationForceScale + m_regularizationForceOffset;

    std::shared_ptr<InputRegularizationTerm> ptr;
    ptr = std::make_shared<InputRegularizationTerm>(6);
    ptr->setSubMatricesStartingPosition(6 + m_actuatedDOFs + m_actuatedDOFs, 0);
    ptr->setWeight(weight);


    m_costFunctions.insert(std::make_pair("regularization_stance_force", ptr));

    m_hessianMatrices.insert(std::make_pair("regularization_stance_force",
                                            std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
    m_gradientVectors.insert(std::make_pair("regularization_stance_force",
                                            std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));

    return true;
}

void TaskBasedTorqueSolverSingleSupport::instantiateLinearMomentumConstraint(const yarp::os::Searchable& config)
{
    m_useLinearMomentumConstraint = config.check("useAsConstraint", yarp::os::Value("False")).asBool();
    if(!m_useLinearMomentumConstraint)
    {
        yWarning() << "[TaskBasedTorqueSolverSingleSupport::instantiateLinearMomentumConstraint] The linear momentum will not be used as a constraint";
        return;
    }
    if(m_useLinearMomentumConstraint)
    {
        // memory allocation
        auto ptr = std::make_shared<LinearMomentumConstraint>(LinearMomentumConstraint::Type::SINGLE_SUPPORT);
        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 6 + m_actuatedDOFs + m_actuatedDOFs);

        m_constraints.insert(std::make_pair("linear_momentum_constraint", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();
    }
    return;
}


bool TaskBasedTorqueSolverSingleSupport::instantiateCoMAsLinearMomentumConstraint(const yarp::os::Searchable& config)
{
    bool asLinearMomentum = config.check("as_linear_momentum", yarp::os::Value("False")).asBool();
    if(asLinearMomentum)
    {
        // memory allocation
        auto ptr = std::make_shared<LinearMomentumConstraint>(LinearMomentumConstraint::Type::SINGLE_SUPPORT, true);
        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 6 + m_actuatedDOFs + m_actuatedDOFs);

        bool useDefaultKd = config.check("useDefaultKd", yarp::os::Value("False")).asBool();

        yarp::os::Value tempValue;
        iDynTree::Vector3 kp;
        tempValue = config.find("kp");
        if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kp))
        {
            yError() << "[initialize] Unable to convert a YARP list to an iDynTree::VectorFixSize, "
                     << "joint regularization";
            return false;
        }

        iDynTree::Vector3 kd;
        if(useDefaultKd)
        {
            double scaling;
            if(!YarpHelper::getNumberFromSearchable(config, "scaling", scaling))
            {
                yError() << "[initialize] Unable to get the scaling factor.";
                return false;
            }
            iDynTree::toEigen(kd) = 2 / scaling * iDynTree::toEigen(kp).array().sqrt();
        }
        else
        {
            tempValue = config.find("kd");
            if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, kd))
            {
                yError() << "[initialize] Unable to convert a YARP list to an iDynTree::VectorFixSize, "
                         << "joint regularization";
                return false;
            }
        }


        ptr->positionController()->setGains(kp, kd);
        m_constraints.insert(std::make_pair("com_linear_momentum_constraint", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();
    }
    return true;
}


bool TaskBasedTorqueSolverSingleSupport::instantiateLinearMomentumCostFunction(const yarp::os::Searchable& config)
{
    m_useLinearMomentumCostFunction = config.check("useAsCostFunction", yarp::os::Value("False")).asBool();
    if(m_useLinearMomentumCostFunction)
    {
        yarp::os::Value tempValue = config.find("weight");
        iDynTree::VectorDynSize weight(3);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, weight))
        {
            yError() << "[TaskBasedTorqueSolverSingleSupport::instantiateLinearMomentumCostFunction] Initialization failed while reading weight vector.";
            return false;
        }

        // memory allocation
        auto ptr = std::make_shared<LinearMomentumCostFunction>(LinearMomentumCostFunction::Type::SINGLE_SUPPORT);
        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(6 + m_actuatedDOFs + m_actuatedDOFs, 0);
        ptr->setWeight(weight);

        m_costFunctions.insert(std::make_pair("linear_momentum_costFunction", ptr));

        m_hessianMatrices.insert(std::make_pair("linear_momentum_costFunction",
                                                std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("linear_momentum_costFunction",
                                                std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }
    return true;
}

void TaskBasedTorqueSolverSingleSupport::setNumberOfVariables()
{
    // the optimization variable is given by
    // 1. base + joint acceleration
    // 2. joint torque
    // 3. stance foot contact wrench

    m_numberOfVariables = 6 + m_actuatedDOFs + m_actuatedDOFs + 6;
}

bool TaskBasedTorqueSolverSingleSupport::setDesiredFeetTrajectory(const iDynTree::Transform& stanceFootToWorldTransform,
                                                                  const iDynTree::Transform& swingFootToWorldTransform,
                                                                  const iDynTree::Twist& swingFootTwist,
                                                                  const iDynTree::SpatialAcc& swingFootAcceleration)
{
    // save left foot trajectory
    if(m_useSwingFootAsConstraint)
    {
        auto constraint = m_constraints.find("swing_foot_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredFeetTrajectory] unable to find the swing foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        ptr->positionController()->setDesiredTrajectory(swingFootAcceleration.getLinearVec3(),
                                                        swingFootTwist.getLinearVec3(),
                                                        swingFootToWorldTransform.getPosition());

        ptr->orientationController()->setDesiredTrajectory(swingFootAcceleration.getAngularVec3(),
                                                           swingFootTwist.getAngularVec3(),
                                                           swingFootToWorldTransform.getRotation());
    }

    if(m_useSwingFootAsCostFunction)
    {
        auto costFunction = m_costFunctions.find("swing_foot_costFunction");
        if(costFunction == m_costFunctions.end())
        {
            yError() << "[setDesiredFeetTrajectory] unable to find the swing foot cost function. "
                     << "Please call 'initialize()' method";
            return false;
        }


        auto ptr = std::static_pointer_cast<CartesianCostFunction>(costFunction->second);
        iDynTree::Vector3 swingFootLinearAcceleration;
        swingFootLinearAcceleration(0) = swingFootAcceleration(0);
        swingFootLinearAcceleration(1) = swingFootAcceleration(1);
        swingFootLinearAcceleration(2) = swingFootAcceleration(2);
        ptr->positionController()->setDesiredTrajectory(swingFootAcceleration.getLinearVec3(),
                                                        swingFootTwist.getLinearVec3(),
                                                        swingFootToWorldTransform.getPosition());


        ptr->orientationController()->setDesiredTrajectory(swingFootAcceleration.getAngularVec3(),
                                                           swingFootTwist.getAngularVec3(),
                                                           swingFootToWorldTransform.getRotation());
    }

    // in single support we cannot control the position of the stance foot
    if(m_controlContact)
    {
        auto constraint = m_constraints.find("stance_foot");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredFeetTrajectory] unable to find the stance foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        iDynTree::Vector3 stanceFootAcceleration;
        iDynTree::Twist stanceFootTwist;
        stanceFootTwist.zero();
        stanceFootAcceleration.zero();
        ptr->positionController()->setDesiredTrajectory(stanceFootAcceleration,
                                                        stanceFootTwist.getLinearVec3(),
                                                        stanceFootToWorldTransform.getPosition());


        ptr->orientationController()->setDesiredTrajectory(stanceFootAcceleration,
                                                           stanceFootTwist.getAngularVec3(),
                                                           stanceFootToWorldTransform.getRotation());
    }

    return true;
}

bool TaskBasedTorqueSolverSingleSupport::setFeetState(const iDynTree::Transform& stanceFootToWorldTransform,
                                                      const iDynTree::Twist& stanceFootTwist,
                                                      const iDynTree::Transform& swingFootToWorldTransform,
                                                      const iDynTree::Twist& swingFootTwist)
{
    m_stanceFootToWorldTransform = stanceFootToWorldTransform;

    // swing foot
    if(m_useSwingFootAsCostFunction)
    {
        auto costFunction = m_costFunctions.find("swing_foot_costFunction");
        if(costFunction == m_costFunctions.end())
        {
            yError() << "[setFeetState] unable to find the swing foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }
        auto ptr = std::static_pointer_cast<CartesianCostFunction>(costFunction->second);
        ptr->positionController()->setFeedback(swingFootTwist.getLinearVec3(),
                                               swingFootToWorldTransform.getPosition());

        ptr->orientationController()->setFeedback(swingFootTwist.getAngularVec3(),
                                                  swingFootToWorldTransform.getRotation());
    }

    if(m_useSwingFootAsConstraint)
    {
        auto constraint = m_constraints.find("swing_foot_constraint");
        if(constraint == m_constraints.end())
        {
            yError() << "[setFeetState] unable to find the swing foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }
        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        ptr->positionController()->setFeedback(swingFootTwist.getLinearVec3(),
                                               swingFootToWorldTransform.getPosition());

        ptr->orientationController()->setFeedback(swingFootTwist.getAngularVec3(),
                                                  swingFootToWorldTransform.getRotation());
    }

    if(m_controlContact)
    {
        auto constraint = m_constraints.find("stance_foot");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredFeetTrajectory] unable to find the swing foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        ptr->positionController()->setFeedback(stanceFootTwist.getLinearVec3(),
                                               stanceFootToWorldTransform.getPosition());


        ptr->orientationController()->setFeedback(stanceFootTwist.getAngularVec3(),
                                                  stanceFootToWorldTransform.getRotation());
    }

    return true;
}

void TaskBasedTorqueSolverSingleSupport::setFeetJacobian(const iDynTree::MatrixDynSize& stanceFootJacobian,
                                                         const iDynTree::MatrixDynSize& swingFootJacobian)
{
    m_stanceFootJacobian = stanceFootJacobian;
    m_swingFootJacobian = swingFootJacobian;
}

void TaskBasedTorqueSolverSingleSupport::setFeetBiasAcceleration(const iDynTree::Vector6 &stanceFootBiasAcceleration,
                                                                 const iDynTree::Vector6 &swingFootBiasAcceleration)
{

    iDynTree::toEigen(m_stanceFootBiasAcceleration) = iDynTree::toEigen(stanceFootBiasAcceleration);
    iDynTree::toEigen(m_swingFootBiasAcceleration) = iDynTree::toEigen(swingFootBiasAcceleration);
}

iDynTree::Wrench TaskBasedTorqueSolverSingleSupport::getStanceWrench()
{
    iDynTree::Wrench wrench;
    for(int i = 0; i < 6; i++)
        wrench(i) = m_solution(6 + m_actuatedDOFs + m_actuatedDOFs + i);

    return wrench;
}

iDynTree::Vector2 TaskBasedTorqueSolverSingleSupport::getZMP()
{
    iDynTree::Position zmpPos;

    iDynTree::Vector2 zmp;

    auto stanceWrench = getStanceWrench();

    zmpPos(0) = -stanceWrench.getAngularVec3()(1) / stanceWrench.getLinearVec3()(2);
    zmpPos(1) = stanceWrench.getAngularVec3()(0) / stanceWrench.getLinearVec3()(2);
    zmpPos(2) = 0.0;

    iDynTree::Transform trans(iDynTree::Rotation::Identity(),
                              m_stanceFootToWorldTransform.getPosition());

    zmpPos = trans * zmpPos;
    zmp(0) = zmpPos(0);
    zmp(1) = zmpPos(1);

    return zmp;
}

bool TaskBasedTorqueSolverSingleSupport::instantiateAngularMomentumConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yInfo() << "[instantiateAngularMomentumConstraint] Empty configuration file. The angular momentum Constraint will not be used";
        m_useAngularMomentumConstraint = false;
        return true;
    }
    m_useAngularMomentumConstraint = config.check("useAsConstraint", yarp::os::Value("False")).asBool();
    if(m_useAngularMomentumConstraint)
    {
        double kp;
        if(!YarpHelper::getNumberFromSearchable(config, "kp", kp))
        {
            yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
            return false;
        }

        std::shared_ptr<AngularMomentumConstraintSingleSupport> ptr;
        ptr = std::make_shared<AngularMomentumConstraintSingleSupport>();
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 6 + m_actuatedDOFs + m_actuatedDOFs);
        ptr->setKp(kp);

        ptr->setStanceFootToWorldTransform(m_stanceFootToWorldTransform);

        m_constraints.insert(std::make_pair("angular_momentum_constraint", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();
    }
    return true;
}

bool TaskBasedTorqueSolverDoubleSupport::instantiateAngularMomentumConstraint(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yInfo() << "[instantiateAngularMomentumConstraint] Empty configuration file. The angular momentum Constraint will not be used";
        m_useAngularMomentumConstraint = false;
        return true;
    }
    m_useAngularMomentumConstraint = config.check("useAsConstraint", yarp::os::Value("False")).asBool();
    if(m_useAngularMomentumConstraint)
    {
        double kp;
        if(!YarpHelper::getNumberFromSearchable(config, "kp", kp))
        {
            yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
            return false;
        }

        std::shared_ptr<AngularMomentumConstraintDoubleSupport> ptr;
        ptr = std::make_shared<AngularMomentumConstraintDoubleSupport>();
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 6 + m_actuatedDOFs + m_actuatedDOFs);
        ptr->setKp(kp);

        ptr->setLeftFootToWorldTransform(m_leftFootToWorldTransform);
        ptr->setRightFootToWorldTransform(m_rightFootToWorldTransform);

        m_constraints.insert(std::make_pair("angular_momentum_constraint", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();
    }
    return true;
}

bool TaskBasedTorqueSolverSingleSupport::instantiateAngularMomentumCostFunction(const yarp::os::Searchable& config)
{
    m_useAngularMomentumCostFunction = config.check("useAsCostFunction", yarp::os::Value("False")).asBool();
    if(m_useAngularMomentumCostFunction)
    {
        yarp::os::Value tempValue = config.find("weight");
        iDynTree::VectorDynSize weight(3);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, weight))
        {
            yError() << "[TaskBasedTorqueSolverSingleSupport::instantiateAngularMomentumCostFunction] Initialization failed while reading weight vector.";
            return false;
        }

        double kp;
        if(!YarpHelper::getNumberFromSearchable(config, "kp", kp))
        {
            yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
            return false;
        }

        auto ptr = std::make_shared<AngularMomentumCostFunctionSingleSupport>();
        ptr->setSubMatricesStartingPosition(6 + m_actuatedDOFs + m_actuatedDOFs, 0);
        ptr->setWeight(weight);
        ptr->setKp(kp);

        ptr->setStanceFootToWorldTransform(m_stanceFootToWorldTransform);

        m_costFunctions.insert(std::make_pair("angular_momentum_costFunction", ptr));

        m_hessianMatrices.insert(std::make_pair("angular_momentum_costFunction",
                                                std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("angular_momentum_costFunction",
                                                std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }
    return true;
}

bool TaskBasedTorqueSolverDoubleSupport::instantiateAngularMomentumCostFunction(const yarp::os::Searchable& config)
{
    m_useAngularMomentumCostFunction = config.check("useAsCostFunction", yarp::os::Value("False")).asBool();
    if(m_useAngularMomentumCostFunction)
    {
        yarp::os::Value tempValue = config.find("weight");
        iDynTree::VectorDynSize weight(3);
        if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, weight))
        {
            yError() << "[TaskBasedTorqueSolverSingleSupport::instantiateAngularMomentumCostFunction] Initialization failed while reading weight vector.";
            return false;
        }

        double kp;
        if(!YarpHelper::getNumberFromSearchable(config, "kp", kp))
        {
            yError() << "[instantiateAngularMomentumConstraint] Unable to get proportional gain";
            return false;
        }

        auto ptr = std::make_shared<AngularMomentumCostFunctionDoubleSupport>();
        ptr->setSubMatricesStartingPosition(6 + m_actuatedDOFs + m_actuatedDOFs, 0);
        ptr->setWeight(weight);
        ptr->setKp(kp);


        ptr->setLeftFootToWorldTransform(m_leftFootToWorldTransform);
        ptr->setRightFootToWorldTransform(m_rightFootToWorldTransform);

        m_costFunctions.insert(std::make_pair("angular_momentum_costFunction", ptr));

        m_hessianMatrices.insert(std::make_pair("angular_momentum_costFunction",
                                                std::make_unique<Eigen::SparseMatrix<double>>(m_numberOfVariables, m_numberOfVariables)));
        m_gradientVectors.insert(std::make_pair("angular_momentum_costFunction",
                                                std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(m_numberOfVariables))));
    }
    return true;
}

bool TaskBasedTorqueSolverSingleSupport::tempPrint()
{

    // Eigen::VectorXd swingFootAcceleration = iDynTree::toEigen(m_swingFootJacobian)
    //     * m_solution.block(0,0,m_actuatedDOFs + 6, 1);

    // auto constraint = m_constraints.find("swing_foot_constraint");
    // auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);

    // std::cerr << "swingFootAcceleration (asked by the user) " <<  m_upperBound.block(ptr->getJacobianStartingRow(), 0, 6, 1) << "\n";


    // std::cerr << "swingFootAcceleration (asked by the qp) "<< swingFootAcceleration << "\n";


    // Eigen::MatioFile file("data.mat");
    // file.write_mat("hessian", Eigen::MatrixXd(m_hessianEigen));
    // file.write_mat("gradient", Eigen::MatrixXd(m_gradient));
    // file.write_mat("constraint", Eigen::MatrixXd(m_constraintMatrix));
    // file.write_mat("lowerBound", Eigen::MatrixXd(m_lowerBound));
    // file.write_mat("upperBound", Eigen::MatrixXd(m_upperBound));
    // file.write_mat("massMatrix", iDynTree::toEigen(m_massMatrix));

    // file.write_mat("stanceFootJacobian", iDynTree::toEigen(m_stanceFootJacobian));
    // file.write_mat("swingFootJacobian", iDynTree::toEigen(m_swingFootJacobian));

    // file.write_mat("solution", m_solution);

    return true;
}
