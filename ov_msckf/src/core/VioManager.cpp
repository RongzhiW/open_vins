/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "VioManager.h"
#include "types/Landmark.h"
#include <opencv2/core/eigen.hpp>



using namespace ov_core;
using namespace ov_msckf;





VioManager::VioManager(ros::NodeHandle &nh) {


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Load our state options
    StateOptions state_options;
    nh.param<bool>("use_fej", state_options.do_fej, false);
    nh.param<bool>("use_imuavg", state_options.imu_avg, false);
    nh.param<bool>("use_rk4int", state_options.use_rk4_integration, true);
    nh.param<bool>("use_stereo", use_stereo, true);
    nh.param<bool>("calib_cam_extrinsics", state_options.do_calib_camera_pose, false);
    nh.param<bool>("calib_cam_intrinsics", state_options.do_calib_camera_intrinsics, false);
    nh.param<bool>("calib_cam_timeoffset", state_options.do_calib_camera_timeoffset, false);
    nh.param<int>("max_clones", state_options.max_clone_size, 10);
    nh.param<int>("max_slam", state_options.max_slam_features, 0);
    nh.param<int>("max_aruco", state_options.max_aruco_features, 0);
    nh.param<int>("max_cameras", state_options.num_cameras, 1);
    nh.param<double>("dt_slam_delay", dt_statupdelay, 3);

    // Enforce that if we are doing stereo tracking, we have two cameras
    if(state_options.num_cameras < 1) {
        ROS_ERROR("VioManager(): Specified number of cameras needs to be greater than zero");
        ROS_ERROR("VioManager(): num cameras = %d", state_options.num_cameras);
        std::exit(EXIT_FAILURE);
    }

    // Read in what representation our feature is
    std::string feat_rep_str;
    nh.param<std::string>("feat_representation", feat_rep_str, "GLOBAL_3D");
    std::transform(feat_rep_str.begin(), feat_rep_str.end(),feat_rep_str.begin(), ::toupper);

    // Set what representation we should be using
    if(feat_rep_str == "GLOBAL_3D") state_options.feat_representation = FeatureRepresentation::Representation::GLOBAL_3D;
    else if(feat_rep_str == "GLOBAL_FULL_INVERSE_DEPTH") state_options.feat_representation = FeatureRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH;
    else if(feat_rep_str == "ANCHORED_3D") state_options.feat_representation = FeatureRepresentation::Representation::ANCHORED_3D;
    else if(feat_rep_str == "ANCHORED_FULL_INVERSE_DEPTH") state_options.feat_representation = FeatureRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;
    else if(feat_rep_str == "ANCHORED_MSCKF_INVERSE_DEPTH") state_options.feat_representation = FeatureRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    else {
        ROS_ERROR("VioManager(): invalid feature representation specified = %s", feat_rep_str.c_str());
        ROS_ERROR("VioManager(): the valid types are:");
        ROS_ERROR("\t- GLOBAL_3D");
        ROS_ERROR("\t- GLOBAL_FULL_INVERSE_DEPTH");
        ROS_ERROR("\t- ANCHORED_3D");
        ROS_ERROR("\t- ANCHORED_FULL_INVERSE_DEPTH");
        ROS_ERROR("\t- ANCHORED_MSCKF_INVERSE_DEPTH");
        std::exit(EXIT_FAILURE);
    }

    // Create the state!!
    state = new State(state_options);

    // Timeoffset from camera to IMU
    double calib_camimu_dt;
    nh.param<double>("calib_camimu_dt", calib_camimu_dt, 0.0);
    Eigen::VectorXd temp_camimu_dt;
    temp_camimu_dt.resize(1);
    temp_camimu_dt(0) = calib_camimu_dt;
    state->calib_dt_CAMtoIMU()->set_value(temp_camimu_dt);
    state->calib_dt_CAMtoIMU()->set_fej(temp_camimu_dt);

    // Global gravity
    Eigen::Matrix<double,3,1> gravity;
    std::vector<double> vec_gravity;
    std::vector<double> vec_gravity_default = {0.0,0.0,9.81};
    nh.param<std::vector<double>>("gravity", vec_gravity, vec_gravity_default);
    gravity << vec_gravity.at(0),vec_gravity.at(1),vec_gravity.at(2);

    // Debug, print to the console!
    ROS_INFO("FILTER PARAMETERS:");
    ROS_INFO("\t- do fej: %d", state_options.do_fej);
    ROS_INFO("\t- do imu avg: %d", state_options.imu_avg);
    ROS_INFO("\t- calibrate cam to imu: %d", state_options.do_calib_camera_pose);
    ROS_INFO("\t- calibrate cam intrinsics: %d", state_options.do_calib_camera_intrinsics);
    ROS_INFO("\t- calibrate cam imu timeoff: %d", state_options.do_calib_camera_timeoffset);
    ROS_INFO("\t- max clones: %d", state_options.max_clone_size);
    ROS_INFO("\t- max slam: %d", state_options.max_slam_features);
    ROS_INFO("\t- max aruco: %d", state_options.max_aruco_features);
    ROS_INFO("\t- max cameras: %d", state_options.num_cameras);
    ROS_INFO("\t- slam startup delay: %.1f", dt_statupdelay);
    ROS_INFO("\t- feature representation: %s", feat_rep_str.c_str());


    // Debug print initial values our state use
    ROS_INFO("STATE INIT VALUES:");
    ROS_INFO("\t- calib_camimu_dt: %.4f", calib_camimu_dt);
    ROS_INFO("\t- gravity: %.3f, %.3f, %.3f", vec_gravity.at(0), vec_gravity.at(1), vec_gravity.at(2));


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Debug message
    ROS_INFO("=====================================");
    ROS_INFO("CAMERA PARAMETERS:");

    // Loop through through, and load each of the cameras
    for(int i=0; i<state->options().num_cameras; i++) {

        // If our distortions are fisheye or not!
        bool is_fisheye;
        nh.param<bool>("cam"+std::to_string(i)+"_is_fisheye", is_fisheye, false);
        state->get_model_CAM(i) = is_fisheye;

        // If the desired fov we should simulate
        std::vector<int> matrix_wh;
        std::vector<int> matrix_wd_default = {752,480};
        nh.param<std::vector<int>>("cam"+std::to_string(i)+"_wh", matrix_wh, matrix_wd_default);
        std::pair<int,int> wh(matrix_wh.at(0),matrix_wh.at(1));

        // Camera intrinsic properties
        Eigen::Matrix<double,8,1> cam_calib;
        std::vector<double> matrix_k, matrix_d;
        std::vector<double> matrix_k_default = {458.654,457.296,367.215,248.375};
        std::vector<double> matrix_d_default = {-0.28340811,0.07395907,0.00019359,1.76187114e-05};
        nh.param<std::vector<double>>("cam"+std::to_string(i)+"_k", matrix_k, matrix_k_default);
        nh.param<std::vector<double>>("cam"+std::to_string(i)+"_d", matrix_d, matrix_d_default);
        cam_calib << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);

        // Save this representation in our state
        state->get_intrinsics_CAM(i)->set_value(cam_calib);
        state->get_intrinsics_CAM(i)->set_fej(cam_calib);

        // Our camera extrinsics transform
        Eigen::Matrix4d T_CtoI;
        std::vector<double> matrix_TCtoI;
        std::vector<double> matrix_TtoI_default = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

        // Read in from ROS, and save into our eigen mat
        nh.param<std::vector<double>>("T_C"+std::to_string(i)+"toI", matrix_TCtoI, matrix_TtoI_default);
        T_CtoI << matrix_TCtoI.at(0),matrix_TCtoI.at(1),matrix_TCtoI.at(2),matrix_TCtoI.at(3),
                matrix_TCtoI.at(4),matrix_TCtoI.at(5),matrix_TCtoI.at(6),matrix_TCtoI.at(7),
                matrix_TCtoI.at(8),matrix_TCtoI.at(9),matrix_TCtoI.at(10),matrix_TCtoI.at(11),
                matrix_TCtoI.at(12),matrix_TCtoI.at(13),matrix_TCtoI.at(14),matrix_TCtoI.at(15);


        // Load these into our state
        Eigen::Matrix<double,7,1> cam_eigen;
        cam_eigen.block(0,0,4,1) = rot_2_quat(T_CtoI.block(0,0,3,3).transpose());
        cam_eigen.block(4,0,3,1) = -T_CtoI.block(0,0,3,3).transpose()*T_CtoI.block(0,3,3,1);
        state->get_calib_IMUtoCAM(i)->set_value(cam_eigen);
        state->get_calib_IMUtoCAM(i)->set_fej(cam_eigen);

        // Append to our maps for our feature trackers
        camera_fisheye.insert({i,is_fisheye});
        camera_calib.insert({i,cam_calib});
        camera_wh.insert({i,wh});

        // opencv stuff for rectify

        camera_k_cv[i] = (cv::Mat_<double>(3, 3)
            << matrix_k[0], 0, matrix_k[2],
               0, matrix_k[1], matrix_k[3],
               0, 0, 1);
        camera_d_cv[i] = (cv::Mat_<double>(4, 1)
            << matrix_d[0], matrix_d[1], matrix_d[2], matrix_d[3]);
        camera_T[i] = T_CtoI;
        std::cout << "camera " << i << "k: " << camera_k_cv[i] << "\n";
        std::cout << "camera " << i << "d: " << camera_d_cv[i] << "\n";

        // Debug printing
        cout << "cam_" << i << "wh:" << endl << wh.first << " x " << wh.second << endl;
        cout << "cam_" << i << "K:" << endl << cam_calib.block(0,0,4,1).transpose() << endl;
        cout << "cam_" << i << "d:" << endl << cam_calib.block(4,0,4,1).transpose() << endl;
        cout << "T_C" << i << "toI:" << endl << T_CtoI << endl << endl;

    }

    if (state->options().num_cameras == 2) {
        Eigen::Matrix3d R_IC0 = camera_T[0].block(0, 0, 3, 3);
        Eigen::Vector3d t_IC0 = camera_T[0].block(0, 3, 3, 1);
        Eigen::Matrix3d R_IC1 = camera_T[1].block(0, 0, 3, 3);
        Eigen::Vector3d t_IC1 = camera_T[1].block(0, 3, 3, 1);
        Eigen::Matrix3d R10 = R_IC1.transpose() * R_IC0;
        Eigen::Vector3d t10 = R_IC1.transpose() * (t_IC0 - t_IC1);
        cv::eigen2cv(R10, R10_cv);
        cv::eigen2cv(t10, t10_cv);
        cout << "\ncam10 R: " << R10_cv << "\nt: " << t10_cv << "\n";
        cv::Size img_size0(camera_wh[0].first, camera_wh[0].second);
        cv::Size img_size1(camera_wh[1].first, camera_wh[1].second);
        if (camera_fisheye[0] && camera_fisheye[1]) {
            RectifyFisheyeCameras(camera_k_cv[0], camera_d_cv[0], img_size0,
                           camera_k_cv[1], camera_d_cv[1], img_size1, R10_cv, t10_cv,
                           mapx_cam0, mapy_cam0, mapx_cam1, mapy_cam1);

            rectify_done = true;
        } else if (!camera_fisheye[0] && !camera_fisheye[1]) {
            RectifyCameras(camera_k_cv[0], camera_d_cv[0], img_size0,
                                  camera_k_cv[1], camera_d_cv[1], img_size1, R10_cv, t10_cv,
                                  mapx_cam0, mapy_cam0, mapx_cam1, mapy_cam1);
        }
        img01_rectify.create(camera_wh[0].second, 2*camera_wh[0].first, CV_8UC1);
        img0_rectify.create(camera_wh[0].second, camera_wh[0].first, CV_8UC1);
        img1_rectify.create(camera_wh[1].second, camera_wh[1].first, CV_8UC1);
    }
    // rolling shutter related
    nh.param<bool>("rs_enabled", rs_enabled, false);
    nh.param<double>("rs_row_tr_us", rs_row_tr, false);
    rs_row_tr *= 1e-6;
    rs_tr = rs_row_tr * camera_wh[0].second;
    ROS_INFO("rs_enabled: %d", rs_enabled);
    ROS_INFO("rs_row_tr: %f", rs_row_tr);
    ROS_INFO("rs_tr: %f", rs_tr);
    // Debug message
    ROS_INFO("=====================================");

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Load config values for our feature initializer
    FeatureInitializerOptions featinit_options;
    nh.param<int>("fi_max_runs", featinit_options.max_runs, 20);
    nh.param<double>("fi_init_lamda", featinit_options.init_lamda, 1e-3);
    nh.param<double>("fi_max_lamda", featinit_options.max_lamda, 1e10);
    nh.param<double>("fi_min_dx", featinit_options.min_dx, 1e-6);
    nh.param<double>("fi_min_dcost", featinit_options.min_dcost, 1e-6);
    nh.param<double>("fi_lam_mult", featinit_options.lam_mult, 10);
    nh.param<double>("fi_min_dist", featinit_options.min_dist, 0.25);
    nh.param<double>("fi_max_dist", featinit_options.max_dist, 40);
    nh.param<double>("fi_max_baseline", featinit_options.max_baseline, 40);
    nh.param<double>("fi_max_cond_number", featinit_options.max_cond_number, 1000);

    // Debug, print to the console!
    ROS_INFO("FEATURE INITIALIZER PARAMETERS:");
    ROS_INFO("\t- max runs: %d", featinit_options.max_runs);
    ROS_INFO("\t- init lambda: %e", featinit_options.init_lamda);
    ROS_INFO("\t- max lambda: %.4f", featinit_options.max_lamda);
    ROS_INFO("\t- min final dx: %.4f", featinit_options.min_dx);
    ROS_INFO("\t- min delta cost: %.4f", featinit_options.min_dcost);
    ROS_INFO("\t- lambda multiple: %.4f", featinit_options.lam_mult);
    ROS_INFO("\t- closest feature dist: %.4f", featinit_options.min_dist);
    ROS_INFO("\t- furthest feature dist: %.4f", featinit_options.max_dist);
    ROS_INFO("\t- max baseline ratio: %.4f", featinit_options.max_baseline);
    ROS_INFO("\t- max condition number: %.4f", featinit_options.max_cond_number);

    // Parameters for our extractor
    int num_pts, fast_threshold, grid_x, grid_y, min_px_dist;
    double knn_ratio;
    bool use_klt, use_aruco, do_downsizing;
    nh.param<bool>("use_klt", use_klt, true);
    nh.param<bool>("use_aruco", use_aruco, false);
    nh.param<int>("num_pts", num_pts, 500);
    nh.param<int>("fast_threshold", fast_threshold, 10);
    nh.param<int>("grid_x", grid_x, 10);
    nh.param<int>("grid_y", grid_y, 8);
    nh.param<int>("min_px_dist", min_px_dist, 10);
    nh.param<double>("knn_ratio", knn_ratio, 0.85);
    nh.param<bool>("downsize_aruco", do_downsizing, true);

    // Debug, print to the console!
    ROS_INFO("TRACKING PARAMETERS:");
    ROS_INFO("\t- use klt: %d", use_klt);
    ROS_INFO("\t- use aruco: %d", use_aruco);
    ROS_INFO("\t- max track features: %d", num_pts);
    ROS_INFO("\t- max aruco tags: %d", state->options().max_aruco_features);
    ROS_INFO("\t- grid size: %d x %d", grid_x, grid_y);
    ROS_INFO("\t- fast threshold: %d", fast_threshold);
    ROS_INFO("\t- min pixel distance: %d", min_px_dist);
    ROS_INFO("\t- downsize aruco image: %d", do_downsizing);


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our noise values for inertial sensor
    Propagator::NoiseManager imu_noises;
    nh.param<double>("gyroscope_noise_density", imu_noises.sigma_w, 1.6968e-04);
    nh.param<double>("accelerometer_noise_density", imu_noises.sigma_a, 2.0000e-3);
    nh.param<double>("gyroscope_random_walk", imu_noises.sigma_wb, 1.9393e-05);
    nh.param<double>("accelerometer_random_walk", imu_noises.sigma_ab, 3.0000e-03);


    // Debug print out
    ROS_INFO("PROPAGATOR NOISES:");
    ROS_INFO("\t- sigma_w: %.4f", imu_noises.sigma_w);
    ROS_INFO("\t- sigma_a: %.4f", imu_noises.sigma_a);
    ROS_INFO("\t- sigma_wb: %.4f", imu_noises.sigma_wb);
    ROS_INFO("\t- sigma_ab: %.4f", imu_noises.sigma_ab);

    // Load inertial state initialize parameters
    double init_window_time, init_imu_thresh;
    nh.param<double>("init_window_time", init_window_time, 0.5);
    nh.param<double>("init_imu_thresh", init_imu_thresh, 1.0);

    // Debug print out
    ROS_INFO("INITIALIZATION PARAMETERS:");
    ROS_INFO("\t- init_window_time: %.4f", init_window_time);
    ROS_INFO("\t- init_imu_thresh: %.4f", init_imu_thresh);


    // Read in update parameters
    UpdaterOptions msckf_options;
    UpdaterOptions slam_options;
    UpdaterOptions aruco_options;
    nh.param<double>("up_msckf_sigma_px", msckf_options.sigma_pix, 1);
    nh.param<int>("up_msckf_chi2_multipler", msckf_options.chi2_multipler, 5);
    nh.param<double>("up_slam_sigma_px", slam_options.sigma_pix, 1);
    nh.param<int>("up_slam_chi2_multipler", slam_options.chi2_multipler, 5);
    nh.param<double>("up_aruco_sigma_px", aruco_options.sigma_pix, 1);
    nh.param<int>("up_aruco_chi2_multipler", aruco_options.chi2_multipler, 5);

    // If downsampling aruco, then double our noise values
    aruco_options.sigma_pix = (do_downsizing) ? 2*aruco_options.sigma_pix : aruco_options.sigma_pix;

    ROS_INFO("MSCKFUPDATER PARAMETERS:");
    ROS_INFO("\t- sigma_pxmsckf: %.4f", msckf_options.sigma_pix);
    ROS_INFO("\t- sigma_pxslam: %.4f", slam_options.sigma_pix);
    ROS_INFO("\t- sigma_pxaruco: %.4f", aruco_options.sigma_pix);
    ROS_INFO("\t- chi2_multipler msckf: %d", msckf_options.chi2_multipler);
    ROS_INFO("\t- chi2_multipler slam: %d", slam_options.chi2_multipler);
    ROS_INFO("\t- chi2_multipler aruco: %d", aruco_options.chi2_multipler);


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Lets make a feature extractor
    if(use_klt) {
        trackFEATS = new TrackKLT(num_pts,state->options().max_aruco_features,fast_threshold,grid_x,grid_y,min_px_dist);
        trackFEATS->set_calibration(camera_calib, camera_fisheye);
    } else {
//        trackFEATS = new TrackDescriptor(num_pts,state->options().max_aruco_features,fast_threshold,grid_x,grid_y,knn_ratio);
//        trackFEATS->set_calibration(camera_calib, camera_fisheye);
    }

    // Initialize our aruco tag extractor
    if(use_aruco) {
//        trackARUCO = new TrackAruco(state->options().max_aruco_features,do_downsizing);
//        trackARUCO->set_calibration(camera_calib, camera_fisheye);
    }

    // Initialize our state propagator
    propagator = new Propagator(imu_noises,gravity);

    // Our state initialize
    initializer = new InertialInitializer(gravity,init_window_time, init_imu_thresh);

    // Make the updater!
    updaterMSCKF = new UpdaterMSCKF(msckf_options, featinit_options);
    updaterSLAM = new UpdaterSLAM(slam_options, aruco_options, featinit_options);


}


void VioManager::RectifyCameras(const cv::Mat& k0, const cv::Mat& d0, const cv::Size& size0,
                                       const cv::Mat& k1, const cv::Mat& d1, const cv::Size& size1,
                                       const cv::Mat& R10, const cv::Mat& t10,
                                       cv::Mat& mapx0, cv::Mat& mapy0, cv::Mat& mapx1, cv::Mat& mapy1) {
    assert(size0.width == size1.width);
    assert(size0.height == size1.height);
    std::cout << "img0 w/h: " << size0.width << " / " << size0.height << "\n";
    cv::Mat R0_rectify, R1_rectify, P0_rectify, P1_rectify, Q;
    double focal0 = k0.at<double>(0, 0);
    double focal1 = k0.at<double>(0, 0);
    std::cout << "cam0 focal: " << focal0 << "\n";
    std::cout << "cam1 focal: " << focal1 << "\n";
    cv::Mat desired_k = (cv::Mat_<double>(3, 3) << (focal0+focal1)/2.0, 0.0, size0.width/2, 0.0, (focal0+focal1)/2.0, size0.height/2, 0.0, 0.0, 1.0);
    cv::stereoRectify(desired_k, d0, desired_k, d1, size0, R10, t10, R0_rectify, R1_rectify, P0_rectify, P1_rectify, Q, cv::CALIB_ZERO_DISPARITY);
    std::cout << "rectify result R0: " << R0_rectify << "\n";
    std::cout << "rectify result R1: " << R1_rectify << "\n";
    std::cout << "rectify result P0: " << P0_rectify << "\n";
    std::cout << "rectify result P1: " << P1_rectify << "\n";
    cv::initUndistortRectifyMap(k0, d0, R0_rectify, P0_rectify, size0, CV_32FC1, mapx0, mapy0);
    cv::initUndistortRectifyMap(k1, d1, R1_rectify, P1_rectify, size1, CV_32FC1, mapx1, mapy1);
}

void VioManager::RectifyFisheyeCameras(const cv::Mat& k0, const cv::Mat& d0, const cv::Size& size0,
                    const cv::Mat& k1, const cv::Mat& d1, const cv::Size& size1,
                    const cv::Mat& R10, const cv::Mat& t10,
                    cv::Mat& mapx0, cv::Mat& mapy0, cv::Mat& mapx1, cv::Mat& mapy1) {
    assert(size0.width == size1.width);
    assert(size0.height == size1.height);
    std::cout << "fisheye img0 w/h: " << size0.width << " / " << size0.height << "\n";
    cv::Mat R0_rectify, R1_rectify, P0_rectify, P1_rectify, Q;
    double focal0 = k0.at<double>(0, 0);
    double focal1 = k0.at<double>(0, 0);
    std::cout << "fisheye cam0 focal: " << focal0 << "\n";
    std::cout << "fisheye cam1 focal: " << focal1 << "\n";
    cv::Mat desired_k = (cv::Mat_<double>(3, 3) << (focal0+focal1)/2.0, 0.0, size0.width/2, 0.0, (focal0+focal1)/2.0, size0.height/2, 0.0, 0.0, 1.0);
    cv::fisheye::stereoRectify(desired_k, d0, desired_k, d1, size0, R10, t10, R0_rectify, R1_rectify, P0_rectify, P1_rectify, Q, cv::CALIB_ZERO_DISPARITY, cv::Size(), 0.0, 0.75);
    std::cout << "fisheye rectify result R0: " << R0_rectify << "\n";
    std::cout << "fisheye rectify result R1: " << R1_rectify << "\n";
    std::cout << "fisheye rectify result P0: " << P0_rectify << "\n";
    std::cout << "fisheye rectify result P1: " << P1_rectify << "\n";
    cv::fisheye::initUndistortRectifyMap(k0, d0, R0_rectify, P0_rectify, size0, CV_32FC1, mapx0, mapy0);
    cv::fisheye::initUndistortRectifyMap(k1, d1, R1_rectify, P1_rectify, size1, CV_32FC1, mapx1, mapy1);
}


void VioManager::feed_measurement_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am) {

    // Push back to our propagator
    propagator->feed_imu(timestamp,wm,am);

    // Push back to our initializer
    if(!is_initialized_vio) {
        initializer->feed_imu(timestamp, wm, am);
    }

}





void VioManager::feed_measurement_monocular(double timestamp, cv::Mat& img0, size_t cam_id) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Feed our trackers
    trackFEATS->feed_monocular(timestamp, img0, cam_id);

    // If aruoc is avalible, the also pass to it
//    if(trackARUCO != nullptr) {
//        trackARUCO->feed_monocular(timestamp, img0, cam_id);
//    }
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then try to initialize
    // TODO: Or if we are trying to reset the system, then do that here!
    if(!is_initialized_vio) {
        is_initialized_vio = try_to_initialize();
        if(!is_initialized_vio) return;
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);


}


void VioManager::feed_measurement_stereo(double timestamp, cv::Mat& img0, cv::Mat& img1, size_t cam_id0, size_t cam_id1) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Assert we have good ids
    assert(cam_id0!=cam_id1);

    // Feed our stereo trackers, if we are not doing binocular
    if(use_stereo) {
        trackFEATS->feed_stereo(timestamp, img0, img1, cam_id0, cam_id1);
    } else {
        boost::thread t_l = boost::thread(&TrackBase::feed_monocular, trackFEATS, boost::ref(timestamp), boost::ref(img0), boost::ref(cam_id0));
        boost::thread t_r = boost::thread(&TrackBase::feed_monocular, trackFEATS, boost::ref(timestamp), boost::ref(img1), boost::ref(cam_id1));
        t_l.join();
        t_r.join();
    }

    // If aruoc is avalible, the also pass to it
    // NOTE: binocular tracking for aruco doesn't make sense as we by default have the ids
    // NOTE: thus we just call the stereo tracking if we are doing binocular!
//    if(trackARUCO != nullptr) {
//        trackARUCO->feed_stereo(timestamp, img0, img1, cam_id0, cam_id1);
//    }
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then try to initialize
    // TODO: Or if we are trying to reset the system, then do that here!
    if(!is_initialized_vio) {
        is_initialized_vio = try_to_initialize();
        if(!is_initialized_vio) return;
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);


    Eigen::Matrix3d rot_ItoG = quat_2_Rot(state->imu()->quat()).transpose();
    Eigen::Vector3d euler_ItoG = rot_ItoG.eulerAngles(2, 1, 0);
    euler_ItoG *= 180.0/M_PI;
    std::cout << "init euler: " << euler_ItoG.transpose() << "\n";
    ROS_INFO("\033[0;32m[run]: euler = %.4f, %.4f, %.4f\033[0m", euler_ItoG(0), euler_ItoG(1), euler_ItoG(2));
    ROS_INFO("\033[0;32m[run]: orientation = %.4f, %.4f, %.4f, %.4f\033[0m",state->imu()->quat()(0),state->imu()->quat()(1),state->imu()->quat()(2),state->imu()->quat()(3));
    ROS_INFO("\033[0;32m[run]: bias gyro = %.4f, %.4f, %.4f\033[0m",state->imu()->bias_g()(0),state->imu()->bias_g()(1),state->imu()->bias_g()(2));
    ROS_INFO("\033[0;32m[run]: velocity = %.4f, %.4f, %.4f\033[0m",state->imu()->vel()(0),state->imu()->vel()(1),state->imu()->vel()(2));
    ROS_INFO("\033[0;32m[run]: bias accel = %.4f, %.4f, %.4f\033[0m",state->imu()->bias_a()(0),state->imu()->bias_a()(1),state->imu()->bias_a()(2));
    ROS_INFO("\033[0;32m[run]: position = %.4f, %.4f, %.4f\033[0m",state->imu()->pos()(0),state->imu()->pos()(1),state->imu()->pos()(2));


}



void VioManager::feed_measurement_simulation(double timestamp, const std::vector<int> &camids, const std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> &feats) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Check if we actually have a simulated tracker
    TrackSIM *trackSIM = dynamic_cast<TrackSIM*>(trackFEATS);
    if(trackSIM == nullptr) {
        //delete trackFEATS; //(fix this error in the future)
        trackFEATS = new TrackSIM(state->options().max_aruco_features);
        trackFEATS->set_calibration(camera_calib, camera_fisheye);
        ROS_ERROR("[SIM]: casting our tracker to a TrackSIM object!");
    }

    // Cast the tracker to our simulation tracker
    trackSIM = dynamic_cast<TrackSIM*>(trackFEATS);
    trackSIM->set_width_height(camera_wh);

    // Feed our simulation tracker
    trackSIM->feed_measurement_simulation(timestamp, camids, feats);
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then return an error
    if(!is_initialized_vio) {
        ROS_ERROR("[SIM]: your vio system should already be initialized before simulating features!!!");
        ROS_ERROR("[SIM]: initialize your system first before calling feed_measurement_simulation()!!!!");
        std::exit(EXIT_FAILURE);
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);


}


bool VioManager::try_to_initialize() {

        // Returns from our initializer
        double time0;
        Eigen::Matrix<double, 4, 1> q_GtoI0;
        Eigen::Matrix<double, 3, 1> b_w0, v_I0inG, b_a0, p_I0inG;

        // Try to initialize the system
        bool success = initializer->initialize_with_imu(time0, q_GtoI0, b_w0, v_I0inG, b_a0, p_I0inG);

        // Return if it failed
        if (!success) {
            return false;
        }

        // Make big vector (q,p,v,bg,ba), and update our state
        // Note: start from zero position, as this is what our covariance is based off of
        Eigen::Matrix<double,16,1> imu_val;
        imu_val.block(0,0,4,1) = q_GtoI0;
        imu_val.block(4,0,3,1) << 0,0,0;
        imu_val.block(7,0,3,1) = v_I0inG;
        imu_val.block(10,0,3,1) = b_w0;
        imu_val.block(13,0,3,1) = b_a0;
        //imu_val.block(10,0,3,1) << 0,0,0;
        //imu_val.block(13,0,3,1) << 0,0,0;
        state->imu()->set_value(imu_val);
        // 这里的time0是newest_time - 0.5
        state->set_timestamp(time0);

        Eigen::Matrix3d rot_ItoG = quat_2_Rot(q_GtoI0).transpose();
        Eigen::Vector3d euler_ItoG = rot_ItoG.eulerAngles(2, 1, 0);
        euler_ItoG *= 180.0/M_PI;
        ROS_INFO("\033[0;32m[INIT]: euler = %.4f, %.4f, %.4f\033[0m",euler_ItoG(0), euler_ItoG(1), euler_ItoG(2));
        // Else we are good to go, print out our stats
        ROS_INFO("\033[0;32m[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\033[0m",state->imu()->quat()(0),state->imu()->quat()(1),state->imu()->quat()(2),state->imu()->quat()(3));
        ROS_INFO("\033[0;32m[INIT]: bias gyro = %.4f, %.4f, %.4f\033[0m",state->imu()->bias_g()(0),state->imu()->bias_g()(1),state->imu()->bias_g()(2));
        ROS_INFO("\033[0;32m[INIT]: velocity = %.4f, %.4f, %.4f\033[0m",state->imu()->vel()(0),state->imu()->vel()(1),state->imu()->vel()(2));
        ROS_INFO("\033[0;32m[INIT]: bias accel = %.4f, %.4f, %.4f\033[0m",state->imu()->bias_a()(0),state->imu()->bias_a()(1),state->imu()->bias_a()(2));
        ROS_INFO("\033[0;32m[INIT]: position = %.4f, %.4f, %.4f\033[0m",state->imu()->pos()(0),state->imu()->pos()(1),state->imu()->pos()(2));
        return true;

}

void VioManager::get_rs_feat_clonesCam(State* state, Feature* feat, std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>>& clones_cam) {
    for (auto const& pair : feat->timestamps) {
        std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
        for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++) {
            double timestamp = feat->timestamps.at(pair.first).at(m);
            int v = int(feat->uvs.at(pair.first).at(m)(1));
            std::vector<RsImuState>& rs_imu_states = state->get_clone_rs_imu_states().at(timestamp);
            assert(rs_imu_states.size() == camera_wh.at(0).second);
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

void VioManager::update_clones_rs_states(State* state) {
    const std::map<double, PoseJPL*>& clones_imu = state->get_clones();
    const std::map<double, Vec*>& clones_vel = state->get_clones_vel();
    std::map<double, std::vector<RsImuState>>& clones_rs_imu_states = state->get_clone_rs_imu_states();
    for (auto& clone : clones_imu) {
        double timestamp = clone.first;
        assert(clones_vel.find(timestamp) != clones_vel.end());
        assert(clone_rs_preintegs.find(timestamp) != clone_rs_preintegs.end());
        const std::vector<RsPreintegState>& rs_preintegs = clone_rs_preintegs[timestamp];
        assert(rs_preintegs.size() == camera_wh[0].second);
        RsImuState state0;
        state0.timestamp = timestamp;
        state0.p = clone.second->pos();
        state0.q = clone.second->quat();
        state0.v = clones_vel.at(timestamp)->value();
        state0.ba = state->imu()->bias_a();
        state0.bg = state->imu()->bias_g();
        std::vector<RsImuState> rs_imu_states;
        propagator->update_rs_imu_propogation(state0, rs_preintegs, rs_imu_states);
        clones_rs_imu_states[timestamp] = rs_imu_states;
        assert(rs_imu_states.size() == camera_wh[0].second);
    }
    // test
//    std::vector<RsImuState>& newest_clone_rs_imu_states = clones_rs_imu_states[state->timestamp()];
//    for (int i = 0; i < newest_clone_rs_imu_states.size(); i++) {
//        const RsImuState& rs_imu_state_new = newest_clone_rs_imu_states[i];
//        std::cout << std::setprecision(16);
//        std::cout << i << " newest_clone_rs_imu_state stamp(us): " << rs_imu_state_new.timestamp*1e6 << "\nnew_rs_imu prop q: " << rs_imu_state_new.q.transpose() << "\n";
//        std::cout << i << " newest_clone_rs_imu prop p: " << rs_imu_state_new.p.transpose() << "\n";
//        std::cout << i << " newest_clone_rs_imu prop v: " << rs_imu_state_new.v.transpose() << "\n";
//        std::cout << i << " newest_clone_rs_imu prop ba: " << rs_imu_state_new.ba.transpose() << "\n";
//        std::cout << i << " newest_clone_rs_imu prop bg: " << rs_imu_state_new.bg.transpose() << "\n";
//    }

}

void VioManager::do_feature_propagate_update(double timestamp) {


    //===================================================================================
    // State propagation, and clone augmentation
    //===================================================================================

    // 如果图像延迟较大，在初始化时是有可能的
    // Return if the camera measurement is out of order
    if(state->timestamp() >= timestamp) {
        ROS_WARN("image received out of order (prop dt = %3f)",(timestamp-state->timestamp()));
        return;
    }

    // If we have just started up, we should record this time as the current time
    if(startup_time == -1) {
        startup_time = timestamp;
    }

    // Propagate the state forward to the current update time
    // Also augment it with a new clone!
    propagator->propagate_and_clone(state, timestamp, {camera_wh[0].second, rs_row_tr, 0.0});
    clone_rs_preintegs[timestamp] = propagator->get_rs_preinteg_states();
    rT3 =  boost::posix_time::microsec_clock::local_time();

    // If we have not reached max clones, we should just return...
    // This isn't super ideal, but it keeps the logic after this easier...
    // We can start processing things when we have at least 5 clones since we can start triangulating things...
    if((int)state->n_clones() < std::min(state->options().max_clone_size,5)) {
        ROS_INFO("waiting for enough clone states (%d of %d) ....",(int)state->n_clones(),std::min(state->options().max_clone_size,5));
        return;
    }

    // Return if we where unable to propagate
    if(state->timestamp() != timestamp) {
        ROS_ERROR("[PROP]: Propagator unable to propagate the state forward in time!");
        ROS_ERROR("[PROP]: It has been %.3f since last time we propagated",timestamp-state->timestamp());
        return;
    }

    // 更新rs在每帧每行对应的位姿
    update_clones_rs_states(state);

    //===================================================================================
    // MSCKF features and KLT tracks that are SLAM features
    //===================================================================================


    // Now, lets get all features that should be used for an update that are lost in the newest frame
    std::vector<Feature*> feats_lost, feats_marg, feats_slam;
    feats_lost = trackFEATS->get_feature_database()->features_not_containing_newer(state->timestamp());

    // Don't need to get the oldest features untill we reach our max number of clones
    if((int)state->n_clones() > state->options().max_clone_size) {
        feats_marg = trackFEATS->get_feature_database()->features_containing(state->margtimestep());
//        if(trackARUCO != nullptr && timestamp-startup_time >= dt_statupdelay) {
//            feats_slam = trackARUCO->get_feature_database()->features_containing(state->margtimestep());
//        }
    }

    // We also need to make sure that the max tracks does not contain any lost features
    // This could happen if the feature was lost in the last frame, but has a measurement at the marg timestep
    auto it1 = feats_lost.begin();
    while(it1 != feats_lost.end()) {
        if(std::find(feats_marg.begin(),feats_marg.end(),(*it1)) != feats_marg.end()) {
//            ROS_WARN("FOUND FEATURE THAT WAS IN BOTH feats_lost and feats_marg!!!!!!");
            it1 = feats_lost.erase(it1);
        } else {
            it1++;
        }
    }

    // Find tracks that have reached max length, these can be made into SLAM features
    std::vector<Feature*> feats_maxtracks;
    auto it2 = feats_marg.begin();
    while(it2 != feats_marg.end()) {
        // See if any of our camera's reached max track
        bool reached_max = false;
        for (const auto &cams: (*it2)->timestamps){
            if ((int)cams.second.size() > state->options().max_clone_size){
                reached_max = true;
                break;
            }
        }
        // If max track, then add it to our possible slam feature list
        if(reached_max) {
            feats_maxtracks.push_back(*it2);
            it2 = feats_marg.erase(it2);
        } else {
            it2++;
        }
    }

    // Count how many aruco tags we have in our state
    int curr_aruco_tags = 0;
    auto it0 = state->features_SLAM().begin();
    while(it0 != state->features_SLAM().end()) {
        if ((int) (*it0).second->_featid <= state->options().max_aruco_features) curr_aruco_tags++;
        it0++;
    }

//    printf("feats_max count:%d\n", feats_maxtracks.size());
//    for (auto &feat : feats_maxtracks) {
//        printf("feats_max tm:%f size:%d id:%d last_tm:%f\n", timestamp, feat->timestamps[0].size(), feat->featid, feat->timestamps[0].at(feat->timestamps[0].size() - 1));
//    }
//    printf("state->features_SLAM ids:\n");
//    for (auto &feat : state->features_SLAM()) {
//        printf("feats_SLAM tm:%f id:%d \n", timestamp, feat.second->_featid);
//    }
    // Append a new SLAM feature if we have the room to do so
    // Also check that we have waited our delay amount (normally prevents bad first set of slam points)
    if(state->options().max_slam_features > 0 && timestamp-startup_time >= dt_statupdelay && (int)state->features_SLAM().size() < state->options().max_slam_features+curr_aruco_tags) {
        // Get the total amount to add, then the max amount that we can add given our marginalize feature array
        int amount_to_add = (state->options().max_slam_features+curr_aruco_tags)-(int)state->features_SLAM().size();
        int valid_amount = (amount_to_add > (int)feats_maxtracks.size())? (int)feats_maxtracks.size() : amount_to_add;
        // If we have at least 1 that we can add, lets add it!
        // Note: we remove them from the feat_marg array since we don't want to reuse information...
        if(valid_amount > 0) {
            feats_slam.insert(feats_slam.end(), feats_maxtracks.end()-valid_amount, feats_maxtracks.end());
            feats_maxtracks.erase(feats_maxtracks.end()-valid_amount, feats_maxtracks.end());
        }
    }
//    printf("feats_slam ids:\n");
//    for (auto &feat : feats_slam) {
//        printf("feats_slam tm:%f id:%d last_tm:%f\n", timestamp, feat->featid, feat->timestamps[0].at(feat->timestamps[0].size() - 1));
//    }

    // Loop through current SLAM features, we have tracks of them, grab them for this update!
    // Note: if we have a slam feature that has lost tracking, then we should marginalize it out
    // Note: if you do not use FEJ, these types of slam features *degrade* the estimator performance....

//    printf("tm:%f states's feature_SLAMS:%d\n", timestamp, state->features_SLAM().size());
    for (std::pair<const size_t, Landmark*> &landmark : state->features_SLAM()) {
//        if(trackARUCO != nullptr) {
//            Feature* feat1 = trackARUCO->get_feature_database()->get_feature(landmark.second->_featid);
//            if(feat1 != nullptr) feats_slam.push_back(feat1);
//        }
        Feature* feat2 = trackFEATS->get_feature_database()->get_feature(landmark.second->_featid);
        // slam feature还有观测
        if(feat2 != nullptr) feats_slam.push_back(feat2);
//        if(feat2 != nullptr) printf("slam feature %d add feat size:%d\n", landmark.first, feat2->timestamps[0].size());
        // 不再继续被看到的old slam feature需要被边缘化掉
        if(feat2 == nullptr) landmark.second->should_marg = true;
//        if (feat2 == nullptr) printf("slam feature %d should_marg:%d\n", landmark.first);
    }

//    printf("feats_lost:%d feats_mag:%d feats_max:%d feats_slam:%d\n", feats_lost.size(), feats_marg.size(), feats_maxtracks.size(), feats_slam.size());
    // Lets marginalize out all old SLAM features here
    // These are ones that where not successfully tracked into the current frame
    // We do *NOT* marginalize out our aruco tags
    StateHelper::marginalize_slam(state);

    // Separate our SLAM features into new ones, and old ones
    std::vector<Feature*> feats_slam_DELAYED, feats_slam_UPDATE;
    for(size_t i=0; i<feats_slam.size(); i++) {
        if(state->features_SLAM().find(feats_slam.at(i)->featid) != state->features_SLAM().end()) {
            feats_slam_UPDATE.push_back(feats_slam.at(i));
            //ROS_INFO("[UPDATE-SLAM]: found old feature %d (%d measurements)",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
        } else {
            feats_slam_DELAYED.push_back(feats_slam.at(i));
            //ROS_INFO("[UPDATE-SLAM]: new feature ready %d (%d measurements)",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
        }
    }

    // Concatenate our MSCKF feature arrays (i.e., ones not being used for slam updates)
//    printf("tm:%f feats_lost/marg/remained_maxtrack size:%d/%d/%d\n", timestamp, feats_lost.size(), feats_marg.size(), feats_maxtracks.size());
    std::vector<Feature*> featsup_MSCKF = feats_lost;
    featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(), feats_marg.end());
    featsup_MSCKF.insert(featsup_MSCKF.end(), feats_maxtracks.begin(), feats_maxtracks.end());


    //===================================================================================
    // Now that we have a list of features, lets do the EKF update for MSCKF and SLAM!
    //===================================================================================

    // Pass them to our MSCKF updater
    // We update first so that our SLAM initialization will be more accurate??
    updaterMSCKF->update(state, featsup_MSCKF);
    rT4 =  boost::posix_time::microsec_clock::local_time();

    // Perform SLAM delay init and update
//    printf("feats_slam_UPDATE size:%d\n", feats_slam_UPDATE.size());
//    printf("feats_slam_DELAYED size:%d\n", feats_slam_DELAYED.size());
    updaterSLAM->update(state, feats_slam_UPDATE);
    updaterSLAM->delayed_init(state, feats_slam_DELAYED);
    rT5 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    // Update our visualization feature set, and clean up the old features
    //===================================================================================


    // Save all the MSCKF features used in the update
    good_features_MSCKF.clear();
    for(Feature* feat : featsup_MSCKF) {
        good_features_MSCKF.push_back(feat->p_FinG);
        feat->to_delete = true;
    }

    // Remove features that where used for the update from our extractors at the last timestep
    // This allows for measurements to be used in the future if they failed to be used this time
    // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
    trackFEATS->get_feature_database()->cleanup();
//    if(trackARUCO != nullptr) {
//        trackARUCO->get_feature_database()->cleanup();
//    }

    //===================================================================================
    // Cleanup, marginalize out what we don't need any more...
    //===================================================================================

    // First do anchor change if we are about to lose an anchor pose
    updaterSLAM->change_anchors(state);

    // Marginalize the oldest clone of the state if we are at max length
    if((int)state->n_clones() > state->options().max_clone_size) {
        StateHelper::marginalize_old_clone(state);
    }

    // Finally if we are optimizing our intrinsics, update our trackers
    if(state->options().do_calib_camera_intrinsics) {
        // Get vectors arrays
        std::map<size_t, Eigen::VectorXd> cameranew_calib;
        std::map<size_t, bool> cameranew_fisheye;
        for(int i=0; i<state->options().num_cameras; i++) {
            Vec* calib = state->get_intrinsics_CAM(i);
            bool isfish = state->get_model_CAM(i);
            cameranew_calib.insert({i,calib->value()});
            cameranew_fisheye.insert({i,isfish});
        }
        // Update the trackers and their databases
        trackFEATS->set_calibration(cameranew_calib, cameranew_fisheye, true);
//        if(trackARUCO != nullptr) {
//            trackARUCO->set_calibration(cameranew_calib, cameranew_fisheye, true);
//        }
    }
    rT6 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    // Debug info, and stats tracking
    //===================================================================================


    // Timing information
    ROS_INFO("\u001b[34m[TIME]: %.4f seconds for tracking\u001b[0m",(rT2-rT1).total_microseconds() * 1e-6);
    ROS_INFO("\u001b[34m[TIME]: %.4f seconds for propagation\u001b[0m",(rT3-rT2).total_microseconds() * 1e-6);
    ROS_INFO("\u001b[34m[TIME]: %.4f seconds for MSCKF update (%d features)\u001b[0m",(rT4-rT3).total_microseconds() * 1e-6, (int)good_features_MSCKF.size());
    if(state->options().max_slam_features > 0)
        ROS_INFO("\u001b[34m[TIME]: %.4f seconds for SLAM update (%d delayed, %d update)\u001b[0m",(rT5-rT4).total_microseconds() * 1e-6, (int)feats_slam_DELAYED.size(), (int)feats_slam_UPDATE.size());
    ROS_INFO("\u001b[34m[TIME]: %.4f seconds for marginalization (%d clones in state)\u001b[0m",(rT6-rT5).total_microseconds() * 1e-6, (int)state->n_clones());
    ROS_INFO("\u001b[34m[TIME]: %.4f seconds for total\u001b[0m",(rT6-rT1).total_microseconds() * 1e-6);

    // Update our distance traveled
    if(timelastupdate != -1 && state->get_clones().find(timelastupdate) != state->get_clones().end()) {
        Eigen::Matrix<double,3,1> dx = state->imu()->pos() - state->get_clone(timelastupdate)->pos();
        distance += dx.norm();
    }
    timelastupdate = timestamp;

    // Debug, print our current state
    ROS_INFO("q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f | dist = %.2f (meters)",
            state->imu()->quat()(0),state->imu()->quat()(1),state->imu()->quat()(2),state->imu()->quat()(3),
            state->imu()->pos()(0),state->imu()->pos()(1),state->imu()->pos()(2),distance);
    ROS_INFO("bg = %.4f,%.4f,%.4f | ba = %.4f,%.4f,%.4f",
             state->imu()->bias_g()(0),state->imu()->bias_g()(1),state->imu()->bias_g()(2),
             state->imu()->bias_a()(0),state->imu()->bias_a()(1),state->imu()->bias_a()(2));


    // Debug for camera imu offset
    if(state->options().do_calib_camera_timeoffset) {
        ROS_INFO("camera-imu timeoffset = %.5f",state->calib_dt_CAMtoIMU()->value()(0));
    }

    // Debug for camera intrinsics
    if(state->options().do_calib_camera_intrinsics) {
        for(int i=0; i<state->options().num_cameras; i++) {
            Vec* calib = state->get_intrinsics_CAM(i);
            ROS_INFO("cam%d intrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f",(int)i,
                     calib->value()(0),calib->value()(1),calib->value()(2),calib->value()(3),
                     calib->value()(4),calib->value()(5),calib->value()(6),calib->value()(7));
        }
    }

    // Debug for camera extrinsics
    if(state->options().do_calib_camera_pose) {
        for(int i=0; i<state->options().num_cameras; i++) {
            PoseJPL* calib = state->get_calib_IMUtoCAM(i);
            ROS_INFO("cam%d extrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f",(int)i,
                     calib->quat()(0),calib->quat()(1),calib->quat()(2),calib->quat()(3),
                     calib->pos()(0),calib->pos()(1),calib->pos()(2));
        }
    }





}























