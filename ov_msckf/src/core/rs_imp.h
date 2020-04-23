//
// Created by dji on 20-4-23.
//

#ifndef PROJECT_RS_IMP_H
#define PROJECT_RS_IMP_H
#include <Eigen/Dense>

struct RsPropOption {
    int image_height;
    double rs_tr_row;
    double rs_td;
};
struct RsPreintegState {
  public:
    RsPreintegState() {
        delta_p.setZero();
        delta_v.setZero();
        delta_q = Eigen::Matrix<double, 4, 1>{0,0,0,1};
        ba.setZero();
        bg.setZero();
        cov.setIdentity();
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Matrix<double, 3, 1> delta_p, delta_v;
    Eigen::Matrix<double, 4, 1> delta_q;
    Eigen::Matrix<double, 3, 1> ba, bg;
    Eigen::MatrixXd cov;
};
struct RsImuState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Matrix<double, 3, 1> p, v;
    Eigen::Matrix<double, 4, 1> q;
    Eigen::Matrix<double, 3, 1> ba, bg;
    Eigen::MatrixXd cov;
};


#endif //PROJECT_RS_IMP_H
