#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H

#include <eigen3/Eigen/Eigen>

namespace eigenUtils{
using namespace Eigen;

bool checkQuaternionSign(const Quaterniond &quat);
Quaterniond flipQuaternionSign(const Quaterniond &quat);
Quaterniond checkFlipQuaternionSign(const Quaterniond &quat);


void virtualSpringPD(VectorXd &n, VectorXd &e_or, VectorXd &edot_or,
                     Quaterniond &Q, Quaterniond &Qd,
                     VectorXd &w, VectorXd &wd,
                     MatrixXd &K, MatrixXd &D );


Matrix3d skewSymmetric(const Vector3d &p);
MatrixXd adjointTransformation(const Vector3d &p, const Matrix3d &R);



}


#endif


