#include "eigenUtils.h"

using namespace Eigen;
/**********************************************************************************
  check sign of quaternion
**********************************************************************************/
bool eigenUtils::checkQuaternionSign(const Quaterniond &quat)
{
    // return true if w < 0
    if(quat.w()<0.0)
        return true;
    return false;
}
/**********************************************************************************
  flip sign of quaternion
**********************************************************************************/
Quaterniond eigenUtils::flipQuaternionSign(const Quaterniond &quat)
{
    return Quaterniond(-quat.w(), -quat.x(), -quat.y(), -quat.z());
}

/**********************************************************************************
  check and flip sign of quaternion if necessary
**********************************************************************************/
Quaterniond eigenUtils::checkFlipQuaternionSign(const Quaterniond &quat)
{
    if(checkQuaternionSign(quat))
        return flipQuaternionSign(quat);
    return quat;
}

/**********************************************************************************
  Virtual torsional spring from Quaternion displacement
**********************************************************************************/

void eigenUtils::virtualSpringPD(VectorXd &n, VectorXd &e_or, VectorXd &edot_or,
                                 Quaterniond &Q, Quaterniond &Qd,
                                 VectorXd &w, VectorXd &wd,
                                 MatrixXd &K, MatrixXd &D ) {

    n = VectorXd::Zero(3);
    e_or = VectorXd::Zero(3);
    edot_or = VectorXd::Zero(3);

    Quaterniond delta_Q;
    delta_Q =  Qd * Q.inverse();
    delta_Q = eigenUtils::checkFlipQuaternionSign(delta_Q);

    n = -2.0*(delta_Q.w() * MatrixXd::Identity(3,3) + eigenUtils::skewSymmetric(delta_Q.vec()) ) * K *delta_Q.vec()
            - D*(w - wd);

    e_or = (delta_Q.w() * MatrixXd::Identity(3,3) + eigenUtils::skewSymmetric(delta_Q.vec()) ) *delta_Q.vec();
    edot_or = w-wd;



}

/**********************************************************************************
  Compute Skew-Symmetric Matrix from vector
**********************************************************************************/

Matrix3d eigenUtils::skewSymmetric(const Vector3d &p) {
    Matrix3d S;
    S <<    0,    -p(2),  p(1),
            p(2),     0, -p(0),
            -p(1), p(0),    0 ;
    return S;
}

/**********************************************************************************
  Compute Adjoint Transformation Matrix
**********************************************************************************/

MatrixXd eigenUtils::adjointTransformation(const Vector3d &p, const Matrix3d &R) {
    MatrixXd Ad = MatrixXd::Zero(6,6);

    Ad.block(0,0,3,3) = R;

    Ad.block(3,0,3,3) = skewSymmetric(p)* R;
    Ad.block(3,3,3,3) = R;


    return Ad;
}
