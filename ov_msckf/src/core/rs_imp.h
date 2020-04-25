//
// Created by dji on 20-4-24.
//

#ifndef PROJECT_RS_IMP_H
#define PROJECT_RS_IMP_H

#include <feat/Feature.h>
#include <feat/FeatureInitializer.h>
#include "state/State.h"
using namespace ov_msckf;

// 计算rs上的每个feat的各个观测对应的camera pose
void get_rs_feat_clonesCam(State* state, Feature* feat, std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>>& clones_cam);

void correct_pFinA(FeatureInitializer::ClonePose& rs_anchor_pose, FeatureInitializer::ClonePose& anchor_pose, Feature* feat);

void feat_clone_imu_at_v(State* state, double timestamp, int v, RsImuState& rs_state_v);

#endif //PROJECT_RS_IMP_H
