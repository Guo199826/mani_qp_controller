#include "../include/geomJac.h"


MatrixXd geomJac(const DQ_SerialManipulator &robot, const MatrixXd &poseJacobian, const VectorXd &q, const int n){
            Matrix<double, 8, 1> v;
            Matrix<double, 3, 4> CJ4_2_J3;
            Matrix<double, 6, 7> J;
            v << -1, 1, 1, 1, -1, 1, 1, 1;
            MatrixXd C8 = v.array().matrix().asDiagonal();
            MatrixXd C4m = -C8.block(0, 0, 4, 4);
            CJ4_2_J3 << 0, 1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;
            DQ xm = robot.fkm(q,n-1);
            J.block(0, 0, 3, n) = CJ4_2_J3 * 2 * xm.P().conj().haminus4() * poseJacobian.topRows(4);
            J.block(3, 0, 3, n) = CJ4_2_J3 * 2 * (xm.D().hamiplus4() * C4m * poseJacobian.topRows(4) + xm.P().conj().haminus4() * poseJacobian.middleRows(4, 4));
            return J.block(0, 0, 6, n);
        }