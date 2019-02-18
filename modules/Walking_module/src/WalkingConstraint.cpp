/**
 * @file WalkingConstraint.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <OsqpEigen/OsqpEigen.h>
#include <WalkingConstraint.hpp>

template<class T>
void copyDenseIntoSparse(const T& dense, const int& startingRow, const int& startingColumn, Eigen::SparseMatrix<double>& sparse)
{
    for(int i = 0; i < dense.rows(); i++)
        for(int j = 0; j < dense.cols(); j++)
            sparse.coeffRef(startingRow + i, startingColumn + j) = dense(i, j);
}

void OptimizationElement::setSubMatricesStartingPosition(const int& startingRow, const int& startingColumn)
{
    // set the jacobian starting raw and column
    m_jacobianStartingRow = startingRow;
    m_jacobianStartingColumn = startingColumn;

    // set the hessian staryint row and column.
    // it is important to notice that the m_hessian is a vector containing the hessian matrices
    // (for a nonlinear constraints the number depends on the constraints)
    // we suppose that m_vector has been already populated (notice in general it occurs in the
    // construct of the derived classes)
    m_hessianStartingRow = startingRow;

    // Notice this is not an error. The hessian matrix is quadratic and the blocks
    // different from zero depends on the function between the constraint and the conditional
    // variables
    m_hessianStartingColumn = startingRow;
}


CartesianElement::CartesianElement(const Type& elementType)
{
    switch(elementType)
    {
    case Type::POSE:
        m_controllers.insert({"position_pid", std::make_shared<LinearPID>()});
        m_controllers.insert({"orientation_pid", std::make_shared<RotationalPID>()});
        m_desiredAcceleration.resize(6);
        m_desiredAcceleration.zero();
        break;

    case Type::POSITION:
        m_controllers.insert({"position_pid", std::make_shared<LinearPID>()});
        m_desiredAcceleration.resize(3);
        m_desiredAcceleration.zero();
        break;

    case Type::ORIENTATION:
        m_controllers.insert({"orientation_pid", std::make_shared<RotationalPID>()});
        m_desiredAcceleration.resize(3);
        m_desiredAcceleration.zero();
        break;

    case Type::ONE_DIMENSION:
        m_controllers.insert({"position_pid", std::make_shared<LinearPID>()});
        m_desiredAcceleration.resize(1);
        m_desiredAcceleration.zero();
        break;

    case Type::CONTACT:
        m_desiredAcceleration.resize(6);
        m_desiredAcceleration.zero();
        break;
    }

    m_elementType = elementType;
}

std::shared_ptr<LinearPID> CartesianElement::positionController()
{
    std::unordered_map<std::string, std::shared_ptr<CartesianPID>>::const_iterator controller;
    controller = m_controllers.find("position_pid");

    if(controller == m_controllers.end())
        return nullptr;

    return std::static_pointer_cast<LinearPID>(controller->second);
}

std::shared_ptr<RotationalPID> CartesianElement::orientationController()
{
    std::unordered_map<std::string, std::shared_ptr<CartesianPID>>::const_iterator controller;
    controller = m_controllers.find("orientation_pid");

    if(controller == m_controllers.end())
        return nullptr;

    return std::static_pointer_cast<RotationalPID>(controller->second);
}

void CartesianElement::evaluateDesiredAcceleration()
{
    switch(m_elementType)
    {
    case Type::POSE:
        m_controllers["position_pid"]->evaluateControl();
        iDynTree::toEigen(m_desiredAcceleration).block(0, 0, 3, 1)
            = iDynTree::toEigen(m_controllers["position_pid"]->getControllerOutput());

        m_controllers["orientation_pid"]->evaluateControl();
        iDynTree::toEigen(m_desiredAcceleration).block(3, 0, 3, 1)
            = iDynTree::toEigen(m_controllers["orientation_pid"]->getControllerOutput());
        break;

    case Type::POSITION:
        m_controllers["position_pid"]->evaluateControl();
        iDynTree::toEigen(m_desiredAcceleration)
            = iDynTree::toEigen(m_controllers["position_pid"]->getControllerOutput());
        break;

    case Type::ORIENTATION:
        m_controllers["orientation_pid"]->evaluateControl();
        iDynTree::toEigen(m_desiredAcceleration)
            = iDynTree::toEigen(m_controllers["orientation_pid"]->getControllerOutput());
        break;

    case Type::ONE_DIMENSION:
        m_controllers["position_pid"]->evaluateControl();
        m_desiredAcceleration(0) = m_controllers["position_pid"]->getControllerOutput()(2);
        break;

    case Type::CONTACT:
        break;
    }
}

CartesianConstraint::CartesianConstraint(const Type& elementType)
    :CartesianElement(elementType)
{
    switch(elementType)
    {
    case Type::POSE:
        m_sizeOfElement = 6;
        break;

    case Type::POSITION:
        m_sizeOfElement = 3;
        break;

    case Type::ORIENTATION:
        m_sizeOfElement = 3;
        break;

    case Type::ONE_DIMENSION:
        m_sizeOfElement = 1;
        break;

    case Type::CONTACT:
        m_sizeOfElement = 6;
        break;
    }
}

void CartesianConstraint::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    copyDenseIntoSparse(*m_roboticJacobian, m_jacobianStartingRow, m_jacobianStartingColumn,
                        jacobian);
}

void CartesianConstraint::evaluateBounds(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds)
{
    evaluateDesiredAcceleration();
    upperBounds.block(m_jacobianStartingRow, 0, m_sizeOfElement, 1) =
        iDynTree::toEigen(m_desiredAcceleration) - iDynTree::toEigen(*m_biasAcceleration);

    lowerBounds.block(m_jacobianStartingRow, 0, m_sizeOfElement, 1) =
        upperBounds.block(m_jacobianStartingRow, 0, m_sizeOfElement, 1);
}

ForceConstraint::ForceConstraint(const int& numberOfPoints, const double& staticFrictionCoefficient,
                                 const double& torsionalFrictionCoefficient, const double& minimalNormalForce,
                                 const iDynTree::Vector2& footLimitX, const iDynTree::Vector2& footLimitY)
{
    // split the friction cone into slices
    double segmentAngle = M_PI / 2 / (numberOfPoints - 1);
    int numberOfEquationsFrictionCone =  4 * (numberOfPoints - 2) + 4;

    // equation used to ensures COP feasibility and unilateral force
    int numberOfEquationsFeasibility = 7;
    int numberOfEquations = numberOfEquationsFrictionCone + numberOfEquationsFeasibility;

    m_sizeOfElement = numberOfEquations;

    m_jacobianLeftTrivialized.resize(numberOfEquations, 6);
    m_upperBound.resize(numberOfEquations);
    m_lowerBound.resize(numberOfEquations);

    // evaluate friction cone constraint
    Eigen::VectorXd angles(numberOfEquationsFrictionCone);
    Eigen::VectorXd pointsX(numberOfEquationsFrictionCone);
    Eigen::VectorXd pointsY(numberOfEquationsFrictionCone);

    for(int i = 0; i < numberOfEquationsFrictionCone; i++)
    {
        angles(i) = i * segmentAngle;
        pointsX(i) = cos(angles(i));
        pointsY(i) = sin(angles(i));
    }

    for(int i = 0; i < numberOfEquationsFrictionCone; i++)
    {
        double firstPointX, firstPointY, secondPointX, secondPointY;
        firstPointX = pointsX(i);
        firstPointY = pointsY(i);

        secondPointX = pointsX((i + 1) % numberOfEquationsFrictionCone);
        secondPointY = pointsY((i + 1) % numberOfEquationsFrictionCone);

        double angularCoefficients;
        angularCoefficients = (secondPointY - firstPointY) / (secondPointX - firstPointX);

        double offset;
        offset = firstPointY - angularCoefficients * firstPointX;

        int inequalityFactor = 1;
        if(angles(i) > M_PI || angles((i + 1) % numberOfEquationsFrictionCone) > M_PI)
            inequalityFactor = -1;

        //  A_ineq(i,:) = inequalityFactor.* [-angularCoefficients, 1, (-offsets*staticFrictionCoefficient), 0, 0, 0];
        m_jacobianLeftTrivialized(i, 0) = -inequalityFactor * angularCoefficients;
        m_jacobianLeftTrivialized(i, 1) = inequalityFactor;
        m_jacobianLeftTrivialized(i, 2) = -inequalityFactor * offset * staticFrictionCoefficient;
    }

    // Unilateral constraint and COP position
    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone, 2) = -torsionalFrictionCoefficient;
    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 1, 2) = -torsionalFrictionCoefficient;
    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 2, 2) = -1;
    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 3, 2) = footLimitX(0);
    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 4, 2) = -footLimitX(1);
    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 5, 2) = footLimitY(0);
    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 6, 2) = -footLimitY(1);

    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 5, 3) = -1;
    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 6, 3) = 1;

    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 3, 4) = 1;
    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 4, 4) = -1;

    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone, 5) = 1;
    m_jacobianLeftTrivialized(numberOfEquationsFrictionCone + 1, 5) = -1;

    for(int i = 0; i < m_sizeOfElement; i++)
    {
        m_lowerBound(i) = -OsqpEigen::INFTY;
        if(i != 2 + numberOfEquationsFrictionCone)
            m_upperBound(i) = 0;
        else
            m_upperBound(i) = -minimalNormalForce;
    }
}

void ForceConstraint::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    auto tmp1 = iDynTree::toEigen(m_jacobianLeftTrivialized).block(0, 0, m_sizeOfElement, 3)
        * iDynTree::toEigen(m_footToWorldTransform->getRotation().inverse());

    auto tmp2 = iDynTree::toEigen(m_jacobianLeftTrivialized).block(0, 3, m_sizeOfElement, 3)
        * iDynTree::toEigen(m_footToWorldTransform->getRotation().inverse());

    for(int i = 0; i < m_jacobianLeftTrivialized.rows(); i++)
    {
        for(int j = 0; j < 3; j++)
            jacobian.coeffRef(i + m_jacobianStartingRow, j + m_jacobianStartingColumn) = tmp1(i,j);

        for(int j = 3; j < 6; j++)
            jacobian.coeffRef(i + m_jacobianStartingRow, j + m_jacobianStartingColumn) = tmp2(i,j - 3);
    }
}

void ForceConstraint::setBoundsConstantElements(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds)
{
    upperBounds.block(m_jacobianStartingRow, 0, m_sizeOfElement, 1) = iDynTree::toEigen(m_upperBound);
    lowerBounds.block(m_jacobianStartingRow, 0, m_sizeOfElement, 1) = iDynTree::toEigen(m_lowerBound);
    return;
}

ZMPConstraint::ZMPConstraint()
{
    m_sizeOfElement = 2;
}

void ZMPConstraint::setBoundsConstantElements(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds)
{
    for(int i = 0; i < m_sizeOfElement; i++)
    {
        lowerBounds(m_jacobianStartingRow + i) = 0;
        upperBounds(m_jacobianStartingRow + i) = 0;
    }
}

void ZMPConstraintDoubleSupport::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    double xL = m_leftFootToWorldTransform->getPosition()(0);
    double yL = m_leftFootToWorldTransform->getPosition()(1);

    double xR = m_rightFootToWorldTransform->getPosition()(0);
    double yR = m_rightFootToWorldTransform->getPosition()(1);

    // left
    jacobian.coeffRef(m_jacobianStartingRow, m_jacobianStartingColumn + 2) = m_desiredZMP(0) - xL;
    jacobian.coeffRef(m_jacobianStartingRow, m_jacobianStartingColumn + 4) = 1;
    jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 2) = m_desiredZMP(1) - yL;
    jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 3) = -1;

    // right
    jacobian.coeffRef(m_jacobianStartingRow, m_jacobianStartingColumn + 2 + 6) = m_desiredZMP(0) - xR;
    jacobian.coeffRef(m_jacobianStartingRow, m_jacobianStartingColumn + 4 + 6) = 1;
    jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 2 + 6) = m_desiredZMP(1) - yR;
    jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 3 + 6) = -1;
}

void ZMPConstraintSingleSupport::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    double x = m_stanceFootToWorldTransform->getPosition()(0);
    double y = m_stanceFootToWorldTransform->getPosition()(1);

    // x
    jacobian.coeffRef(m_jacobianStartingRow, m_jacobianStartingColumn + 2) = m_desiredZMP(0) - x;
    jacobian.coeffRef(m_jacobianStartingRow, m_jacobianStartingColumn + 4) = 1;

    // y
    jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 2) = m_desiredZMP(1) -y;
    jacobian.coeffRef(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 3) = -1;
}

SystemDynamicConstraint::SystemDynamicConstraint(const int& systemSize)
{
    m_sizeOfElement = systemSize + 6;
    m_systemSize = systemSize;
}

void SystemDynamicConstraint::evaluateBounds(Eigen::VectorXd &upperBounds,
                                             Eigen::VectorXd &lowerBounds)
{
    upperBounds.block(m_jacobianStartingRow, 0, m_sizeOfElement, 1) = iDynTree::toEigen(*m_generalizedBiasForces);
    lowerBounds.block(m_jacobianStartingRow, 0, m_sizeOfElement, 1) = iDynTree::toEigen(*m_generalizedBiasForces);
}

void SystemDynamicConstraint::setJacobianConstantElements(Eigen::SparseMatrix<double>& jacobian)
{
    // The selection matrix is [ zeros(6,n); eye(n,n)]
    int startingRow = m_jacobianStartingRow + 6;
    int startingColumns = m_jacobianStartingColumn + m_systemSize + 6;

    for(int i = 0; i < m_systemSize; i++)
        jacobian.insert(startingRow + i, startingColumns + i) = 1;
}

void SystemDynamicConstraintDoubleSupport::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    // copy the sparse matrix
    copyDenseIntoSparse(-iDynTree::toEigen(*m_massMatrix),
                        m_jacobianStartingRow, m_jacobianStartingColumn, jacobian);

    copyDenseIntoSparse(iDynTree::toEigen(*m_leftFootJacobian).transpose(),
                        m_jacobianStartingRow,
                        m_jacobianStartingColumn + m_systemSize + 6 + m_systemSize,
                        jacobian);

    copyDenseIntoSparse(iDynTree::toEigen(*m_rightFootJacobian).transpose(),
                        m_jacobianStartingRow,
                        m_jacobianStartingColumn + m_systemSize + 6 + m_systemSize + 6,
                        jacobian);
}

void SystemDynamicConstraintSingleSupport::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    copyDenseIntoSparse(-iDynTree::toEigen(*m_massMatrix),
                        m_jacobianStartingRow, m_jacobianStartingColumn, jacobian);

    copyDenseIntoSparse(iDynTree::toEigen(*m_stanceFootJacobian).transpose(),
                        m_jacobianStartingRow,
                        m_jacobianStartingColumn + m_systemSize + 6 + m_systemSize,
                        jacobian);
}


LinearMomentumConstraint::LinearMomentumConstraint(const Type& elementType)
    :LinearMomentumElement(elementType)
{
    m_sizeOfElement = 3;
}

void LinearMomentumConstraint::setJacobianConstantElements(Eigen::SparseMatrix<double>& jacobian)
{
    // the jacobian is constant so it can be evaluated only once
    // notice here we suppose that the Jacobian matrix is not reinitialized
    jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 0) = 1;
    jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 1) = 1;
    jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 2) = 1;

    if(m_elementType == Type::DOUBLE_SUPPORT)
    {
        jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 0 + 6) = 1;
        jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 1 + 6) = 1;
        jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 2 + 6) = 1;
    }
}

void LinearMomentumConstraint::evaluateBounds(Eigen::VectorXd &upperBounds,
                                              Eigen::VectorXd &lowerBounds)
{
    iDynTree::Vector3 weightForce;
    weightForce.zero();
    weightForce(2) = -m_robotMass * 9.81;

    // TODO remove magic number
    double omegaSquare = 9.81 / 0.53;

    upperBounds.block(m_jacobianStartingRow, 0, 3, 1) = -iDynTree::toEigen(weightForce) +
        m_robotMass * omegaSquare * (iDynTree::toEigen(m_comPosition) -
                                     iDynTree::toEigen(m_desiredVRPPosition));

    lowerBounds.block(m_jacobianStartingRow, 0, 3, 1)
        = upperBounds.block(m_jacobianStartingRow, 0, 3, 1);
}

AngularMomentumConstraint::AngularMomentumConstraint()
{
    m_sizeOfElement = 3;

    m_controller = std::make_shared<LinearPID>();
    // set the desired trajectory (it is constant)
    iDynTree::Vector3 dummy;
    dummy.zero();
    m_controller->setDesiredTrajectory(dummy, dummy, dummy);
}

void AngularMomentumConstraint::setJacobianConstantElements(Eigen::SparseMatrix<double>& jacobian)
{

    jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 3) = 1;
    jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 4) = 1;
    jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 5) = 1;

    jacobian.insert(m_jacobianStartingRow + 0, m_jacobianStartingColumn + 3 + 6) = 1;
    jacobian.insert(m_jacobianStartingRow + 1, m_jacobianStartingColumn + 4 + 6) = 1;
    jacobian.insert(m_jacobianStartingRow + 2, m_jacobianStartingColumn + 5 + 6) = 1;
}

void AngularMomentumConstraint::evaluateJacobian(Eigen::SparseMatrix<double>& jacobian)
{
    Eigen::Vector3d leftFootToCoMPosition, rightFootToCoMPosition;
    leftFootToCoMPosition = iDynTree::toEigen(m_leftFootToWorldTransform->getPosition())
        - iDynTree::toEigen(*m_comPosition);

    rightFootToCoMPosition = iDynTree::toEigen(m_rightFootToWorldTransform->getPosition())
        - iDynTree::toEigen(*m_comPosition);

    auto leftFootToCoMPositionSkew = iDynTree::skew(leftFootToCoMPosition);
    auto rightFootToCoMPositionSkew = iDynTree::skew(rightFootToCoMPosition);

    copyDenseIntoSparse(leftFootToCoMPositionSkew, m_jacobianStartingRow,
                        m_jacobianStartingColumn,jacobian);

    copyDenseIntoSparse(rightFootToCoMPositionSkew, m_jacobianStartingRow,
                        m_jacobianStartingColumn + 6, jacobian);
}

void AngularMomentumConstraint::evaluateBounds(Eigen::VectorXd &upperBounds,
                                               Eigen::VectorXd &lowerBounds)
{
    m_controller->evaluateControl();

    upperBounds.block(m_jacobianStartingRow, 0, 3, 1) = iDynTree::toEigen(m_controller->getControllerOutput());
    lowerBounds.block(m_jacobianStartingRow, 0, 3, 1) = iDynTree::toEigen(m_controller->getControllerOutput());
}

RateOfChangeConstraint::RateOfChangeConstraint(const int& sizeOfTheConstraintVector)
{
    m_sizeOfElement = sizeOfTheConstraintVector;
}

void RateOfChangeConstraint::setJacobianConstantElements(Eigen::SparseMatrix<double>& jacobian)
{
    for(int i = 0; i < m_sizeOfElement; i++)
        jacobian.insert(m_jacobianStartingRow + i, m_jacobianStartingColumn + i) = 1;
}


void RateOfChangeConstraint::evaluateBounds(Eigen::VectorXd &upperBounds, Eigen::VectorXd &lowerBounds)
{
    for(int i = 0; i < m_sizeOfElement; i++)
    {
        lowerBounds(m_jacobianStartingRow + i) = (*m_previousValues)(i) - m_maximumRateOfChange(i);
        upperBounds(m_jacobianStartingRow + i) = (*m_previousValues)(i) + m_maximumRateOfChange(i);
    }
}

CartesianCostFunction::CartesianCostFunction(const Type& elementType)
    :CartesianElement(elementType)
{
    switch(elementType)
    {
    case Type::POSE:
        m_sizeOfElement = 6;
        break;

    case Type::POSITION:
        m_sizeOfElement = 3;
        break;

    case Type::ORIENTATION:
        m_sizeOfElement = 3;
        break;

    case Type::ONE_DIMENSION:
        m_sizeOfElement = 1;
        break;

    case Type::CONTACT:
        throw "Invalid element type";
        break;
    }
}

void CartesianCostFunction::evaluateHessian(Eigen::SparseMatrix<double>& hessian)
{
    m_hessianSubMatrix = iDynTree::toEigen(*m_roboticJacobian).transpose()
        * iDynTree::toEigen(m_weight).asDiagonal() * iDynTree::toEigen(*m_roboticJacobian);

    copyDenseIntoSparse(m_hessianSubMatrix, m_hessianStartingRow,
                        m_hessianStartingColumn, hessian);
}

void CartesianCostFunction::evaluateGradient(Eigen::VectorXd& gradient)
{
    evaluateDesiredAcceleration();

    m_gradientSubMatrix = iDynTree::toEigen(*m_roboticJacobian).transpose() *
        iDynTree::toEigen(m_weight).asDiagonal();

    gradient.block(m_hessianStartingRow, 0, m_roboticJacobian->cols(), 1) = -m_gradientSubMatrix *
        (iDynTree::toEigen(m_desiredAcceleration) - iDynTree::toEigen(*m_biasAcceleration));
}

void JointRegularizationTerm::evaluateHessian(Eigen::SparseMatrix<double>& hessian)
{
    for(int i = 0; i < m_sizeOfElement; i++)
        hessian.coeffRef(m_hessianStartingRow + i, m_hessianStartingColumn + i) = m_weight(i);
}

void JointRegularizationTerm::evaluateGradient(Eigen::VectorXd& gradient)
{
    double desiredJointAccelerationControlled;

    for(int i = 0; i < m_sizeOfElement; i++)
    {
        desiredJointAccelerationControlled = m_desiredJointAcceleration->getVal(i)
            + m_derivativeGains(i) * (m_desiredJointVelocity->getVal(i) - m_jointVelocity->getVal(i))
            + m_proportionalGains(i) * (m_desiredJointPosition->getVal(i) - m_jointPosition->getVal(i));

        gradient(i + m_hessianStartingRow) = -m_weight(i) * desiredJointAccelerationControlled;
    }
}

void InputRegularizationTerm::evaluateHessian(Eigen::SparseMatrix<double>& hessian)
{
    for(int i = 0; i < m_sizeOfElement; i++)
        hessian.coeffRef(m_hessianStartingRow + i, m_hessianStartingColumn + i) = m_weight(i);
}

LinearMomentumCostFunction::LinearMomentumCostFunction(const Type &elemetType)
    : LinearMomentumElement(elemetType)
{
    m_sizeOfElement = 3;
}

void LinearMomentumCostFunction::setHessianConstantElements(Eigen::SparseMatrix<double>& hessian)
{
    // suppose that the weight is a diagonal matrix.
    // in case of double support the hessian sub-matrix is given by
    // hessian = [weight   0   weight   0
    //            weight   0   weight   0]
    // in case of single support the hessian sub-matrix is given by
    // hessian = [weight    0
    //               0      0]

    for(int i = 0; i < m_sizeOfElement; i++)
        hessian.insert(m_hessianStartingRow + i, m_hessianStartingColumn + i) = m_weight(i);

    if(m_elementType == Type::DOUBLE_SUPPORT)
    {
        for(int i = 0; i < m_sizeOfElement; i++)
        {
            hessian.insert(m_hessianStartingRow + i + 6, m_hessianStartingColumn + i) = m_weight(i);
            hessian.insert(m_hessianStartingRow + i, m_hessianStartingColumn + i + 6) = m_weight(i);
            hessian.insert(m_hessianStartingRow + i + 6, m_hessianStartingColumn + i + 6) = m_weight(i);
        }
    }
}

void LinearMomentumCostFunction::evaluateGradient(Eigen::VectorXd& gradient)
{
    iDynTree::Vector3 weightForce;
    weightForce.zero();
    weightForce(2) = -m_robotMass * 9.81;

    // TODO remove magic number
    double omegaSquare = 9.81 / 0.53;

    gradient.block(m_hessianStartingRow, 0, m_sizeOfElement, 1) =
        (-iDynTree::toEigen(m_weight)).asDiagonal() *
        (m_robotMass * omegaSquare * (iDynTree::toEigen(m_comPosition) -
                                      iDynTree::toEigen(m_desiredVRPPosition))
         -iDynTree::toEigen(weightForce));

    if(m_elementType == Type::DOUBLE_SUPPORT)
    {
        gradient.block(m_hessianStartingRow + 6, 0, m_sizeOfElement, 1)
            = gradient.block(m_hessianStartingRow, 0, m_sizeOfElement, 1);
    }
}
