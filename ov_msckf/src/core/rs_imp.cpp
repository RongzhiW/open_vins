//
// Created by dji on 20-4-24.
//

#include "rs_imp.h"
#include "rs_info.h"
#include "state/Propagator.h"

void get_rs_feat_clonesCam(State* state, Feature* feat, std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>>& clones_cam) {
    for (auto const& pair : feat->timestamps) {
        std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
        for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++) {
            double timestamp = feat->timestamps.at(pair.first).at(m);
            int u = int(feat->uvs.at(pair.first).at(m)(0));
            int v = int(feat->uvs.at(pair.first).at(m)(1));
//            std::cout << "feat_id: " << feat->featid << " u/v: " << u << "/" << v << "\n";

            RsImuState rs_state_v;
            feat_clone_imu_at_v(state, timestamp, v, rs_state_v);

            std::vector<RsImuState>& rs_imu_states = state->get_clone_rs_imu_states().at(timestamp);

            assert((rs_state_v.timestamp-rs_imu_states[v].timestamp)<1e-20);
            assert((rs_state_v.p-rs_imu_states[v].p).norm()<1e-20);
            assert((rs_state_v.q-rs_imu_states[v].q).norm()<1e-20);
            assert((rs_state_v.v-rs_imu_states[v].v).norm()<1e-20);
//            assert(rs_imu_states.size() == camera_wh.at(0).second);
//            assert(v < rs_imu_states.size());

            Eigen::Matrix<double, 4, 1> q_IG = rs_state_v.q;
            Eigen::Matrix<double, 3, 3> R_IG = quat_2_Rot(q_IG);
            Eigen::Matrix<double, 3, 1> p_GI = rs_state_v.p;
            Eigen::Matrix<double, 3, 3> R_CI = state->get_calib_IMUtoCAM(pair.first)->Rot();
            Eigen::Matrix<double, 3, 1> t_CI = state->get_calib_IMUtoCAM(pair.first)->pos();
            Eigen::Matrix<double, 3, 3> R_CG = R_CI * R_IG;
            Eigen::Matrix<double, 3, 1> p_GC = p_GI - R_CG.transpose() * t_CI;
            clones_cami.insert({timestamp, FeatureInitializer::ClonePose(R_CG, p_GC)});
        }
        clones_cam.insert({pair.first, clones_cami});
    }
}

void feat_clone_imu_at_v(State* state, double timestamp, int v, RsImuState& rs_state_v) {
    assert(state != nullptr);
    RsImuState rs_state_0;
    rs_state_0.timestamp = timestamp;
    rs_state_0.p = state->get_clone(timestamp)->pos();
    rs_state_0.q = state->get_clone(timestamp)->quat();
    rs_state_0.v = state->get_clones_vel()[timestamp]->value();
    rs_state_0.ba = state->imu()->bias_a();
    rs_state_0.bg = state->imu()->bias_g();
    assert(v < state->get_clones_rs_imu_preintegs()[timestamp].size());
    RsPreintegState& rs_preinteg = state->get_clones_rs_imu_preintegs()[timestamp].at(v);
    Propagator::rs_pose_each_v(rs_state_0, rs_preinteg, state->options().gravity, rs_state_v);
}

void correct_pFinA(FeatureInitializer::ClonePose& rs_anchor_pose, FeatureInitializer::ClonePose& anchor_pose, Feature* feat) {
    Eigen::Matrix<double,3,3> &R_GtoRsA = rs_anchor_pose.Rot();
    Eigen::Matrix<double,3,1> &p_RsAinG = rs_anchor_pose.pos();
    Eigen::Matrix<double,3,3> &R_GtoA = anchor_pose.Rot();
    Eigen::Matrix<double,3,1> &p_AinG = anchor_pose.pos();
    Eigen::Matrix<double,3,3> R_RsAtoA = R_GtoA * R_GtoRsA.transpose();
    Eigen::Matrix<double,3,1> p_RsAtoA = R_GtoA * (p_RsAinG - p_AinG);
    feat->p_FinA = R_RsAtoA * feat->p_FinA + p_RsAtoA;
    Eigen::Matrix<double,3,1> p_FinG = R_GtoA.transpose() * feat->p_FinA + p_AinG;
//    std::cout << "p_RsAinG: " << p_RsAinG.transpose() << " p_AinG: " << p_AinG.transpose() << "\n";
//    std::cout << "p_FinG: " << p_FinG.transpose() << " feat->p_FinG: " << feat->p_FinG.transpose() << "\n";
    assert(fabs((p_FinG - feat->p_FinG).norm()) < 1e-10);
}
