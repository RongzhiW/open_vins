//
// Created by dji on 20-4-24.
//

#include "rs_imp.h"
#include "rs_info.h"

void get_rs_feat_clonesCam(State* state, Feature* feat, std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>>& clones_cam) {
    for (auto const& pair : feat->timestamps) {
        std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
        for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++) {
            double timestamp = feat->timestamps.at(pair.first).at(m);
            int u = int(feat->uvs.at(pair.first).at(m)(0));
            int v = int(feat->uvs.at(pair.first).at(m)(1));
//            std::cout << "feat_id: " << feat->featid << " u/v: " << u << "/" << v << "\n";
            std::vector<RsImuState>& rs_imu_states = state->get_clone_rs_imu_states().at(timestamp);
//            assert(rs_imu_states.size() == camera_wh.at(0).second);
            assert(v < rs_imu_states.size());
            Eigen::Matrix<double, 4, 1> q_IG = rs_imu_states[v].q;
            Eigen::Matrix<double, 3, 3> R_IG = quat_2_Rot(q_IG);
            Eigen::Matrix<double, 3, 1> p_GI = rs_imu_states[v].p;
            Eigen::Matrix<double, 3, 3> R_CI = state->get_calib_IMUtoCAM(pair.first)->Rot();
            Eigen::Matrix<double, 3, 1> t_CI = state->get_calib_IMUtoCAM(pair.first)->pos();
            Eigen::Matrix<double, 3, 3> R_CG = R_CI * R_IG;
            Eigen::Matrix<double, 3, 1> p_GC = p_GI - R_CG.transpose() * t_CI;
            clones_cami.insert({timestamp, FeatureInitializer::ClonePose(R_CG, p_GC)});
        }
        clones_cam.insert({pair.first, clones_cami});
    }
}
