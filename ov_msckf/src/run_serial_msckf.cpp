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


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
//#include <cv_bridge/cv_bridge.h>

#include "core/VioManager.h"
#include "core/RosVisualizer.h"
#include "utils/dataset_reader.h"


using namespace ov_msckf;


VioManager* sys;
RosVisualizer* viz;


// Main function
int main(int argc, char** argv)
{

    // Launch our ros node
    ros::init(argc, argv, "run_serial_msckf");
    ros::NodeHandle nh("~");

    // Create our VIO system
    sys = new VioManager(nh);
    viz = new RosVisualizer(nh, sys);


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our camera topics (left and right stereo)
    std::string topic_imu;
    std::string topic_camera0;
    std::string topic_camera1;
    nh.param<std::string>("topic_imu", topic_imu, "/imu0");
    nh.param<std::string>("topic_camera0", topic_camera0, "/cam0/image_raw");
    nh.param<std::string>("topic_camera1", topic_camera1, "/cam1/image_raw");

    // Location of the ROS bag we want to read in
    std::string path_to_bag;
    //nhPrivate.param<std::string>("path_bag", path_to_bag, "/home/keck/catkin_ws/V1_01_easy.bag");
//    nh.param<std::string>("path_bag", path_to_bag, "/home/dji/Documents/datasets/EucRoc/V2_01_easy.bag");
    nh.param<std::string>("path_bag", path_to_bag, "/home/dji/Documents/datasets/rolling_shutter/calibrated/dataset-seq1.bag");
    ROS_INFO("ros bag path is: %s", path_to_bag.c_str());

    // Load groundtruth if we have it
    std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
    if (nh.hasParam("path_gt")) {
        std::string path_to_gt;
        nh.param<std::string>("path_gt", path_to_gt, "");
        DatasetReader::load_gt_file(path_to_gt, gt_states);
        ROS_INFO("gt file path is: %s", path_to_gt.c_str());
    }

    // Get our start location and how much of the bag we want to play
    // Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    ROS_INFO("bag start: %.1f",bag_start);
    ROS_INFO("bag duration: %.1f",bag_durr);

    // Read in what mode we should be processing in (1=mono, 2=stereo)
    int max_cameras;
    nh.param<int>("max_cameras", max_cameras, 1);


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);


    // We should load the bag as a view
    // Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    // Start a few seconds in from the full view time
    // If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish = (bag_durr < 0)? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO("time start = %.6f", time_init.toSec());
    ROS_INFO("time end   = %.6f", time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);

    // Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }


    // Buffer variables for our system (so we always have imu to use)
    bool has_left = false;
    bool has_right = false;
    cv::Mat img0, img1;
    cv::Mat img0_buffer, img1_buffer;
    double time = time_init.toSec();
    double time_buffer = time_init.toSec();
    double time0, time1;


    //===================================================================================
    //===================================================================================
    //===================================================================================

  cv::namedWindow("left_image");
  cv::namedWindow("right_image");
  cv::namedWindow("lr_rectify");
    cv::namedWindow("left_rectify");
    cv::namedWindow("right_rectify");

    // Step through the rosbag
    int msg_cnt = 0;
    for (const rosbag::MessageInstance& m : view) {

        // If ros is wants us to stop, break out
        if (!ros::ok())
            break;

        // Handle IMU measurement
        sensor_msgs::Imu::ConstPtr s2 = m.instantiate<sensor_msgs::Imu>();
        if (s2 != NULL && m.getTopic() == topic_imu) {
            // convert into correct format
            double timem = (*s2).header.stamp.toSec();
            Eigen::Matrix<double, 3, 1> wm, am;
            wm << (*s2).angular_velocity.x, (*s2).angular_velocity.y, (*s2).angular_velocity.z;
            am << (*s2).linear_acceleration.x, (*s2).linear_acceleration.y, (*s2).linear_acceleration.z;
            viz->pub_acc_gyr_m(am, wm);
            // send it to our VIO system
            // imu 数据塞到buffer里
            sys->feed_measurement_imu(timem, wm, am);
//            std::cout << std::setprecision(16);
//            std::cout << "msg_cnt: " << msg_cnt << " imu stamp: " << (*s2).header.stamp.toSec() << std::endl;
//            std:cout << "am: " << am.transpose() << "\n";
//            std::cout << "wm: " << wm.transpose() << "\n";
        }

        // Handle LEFT camera
        sensor_msgs::Image::ConstPtr s0 = m.instantiate<sensor_msgs::Image>();
        if (s0 != NULL && m.getTopic() == topic_camera0) {
            // Get the image
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(s0, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }
//            cv::Mat img_left(s0->height, s0->width, 0, const_cast<uint8_t *>(s0->data.data()));
            // Save to our temp variable
            has_left = true;
            img0 = cv_ptr->image.clone();
            time = cv_ptr->header.stamp.toSec();
//            img0 = img_left.clone();
//            time = s0->header.stamp.toSec();
            cv::imshow("left_image", img0);
            cv::waitKey(10);
//            std::cout << std::setprecision(16);
//            std::cout << "msg_cnt: " << msg_cnt << " left camera stamp: " << s0->header.stamp.toSec() << std::endl;
        }

        // Handle RIGHT camera
        sensor_msgs::Image::ConstPtr s1 = m.instantiate<sensor_msgs::Image>();
        if (s1 != NULL && m.getTopic() == topic_camera1) {
            // Get the image
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(s1, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }
//            cv::Mat img_right(s1->height, s1->width, 0, const_cast<uint8_t *>(s1->data.data()));
            // Save to our temp variable (use a right image that is near in time)
            // TODO: fix this logic as the left will still advance instead of waiting
            // TODO: should implement something like here:
            // TODO: https://github.com/rpng/MARS-VINS/blob/master/example_ros/ros_driver.cpp
            //if(std::abs(cv_ptr->header.stamp.toSec()-time) < 0.02) {
            has_right = true;
            img1 = cv_ptr->image.clone();
//            img1 = img_right.clone();
            time1 = cv_ptr->header.stamp.toSec();
            cv::imshow("right_image", img1);
            cv::waitKey(10);
//          std::cout << "image w/h: " << img1.cols << " / " << img1.rows << "\n";
//            cv::waitKey(0);
//            std::cout << std::setprecision(16);
//            std::cout << "msg_cnt: " << msg_cnt << " right camera stamp: " << s1->header.stamp.toSec() << std::endl;
            //}
        }
        msg_cnt++;


        // Fill our buffer if we have not
        if(has_left && img0_buffer.rows == 0) {
            // 第一帧图像不做处理
            has_left = false;
            time_buffer = time;
            img0_buffer = img0.clone();
        }

        // Fill our buffer if we have not
        if(has_right && img1_buffer.rows == 0) {
            has_right = false;
            img1_buffer = img1.clone();
        }


        // If we are in monocular mode, then we should process the left if we have it
        if(max_cameras==1 && has_left) {
            // process once we have initialized with the GT
            Eigen::Matrix<double, 17, 1> imustate;
            if(!gt_states.empty() && !sys->intialized() && DatasetReader::get_gt_state(time_buffer,imustate,gt_states)) {
                //biases are pretty bad normally, so zero them
                //imustate.block(11,0,6,1).setZero();
                sys->initialize_with_gt(imustate);
            } else if(gt_states.empty() || sys->intialized()) {
                sys->feed_measurement_monocular(time_buffer, img0_buffer, 0);
            }
            // visualize
            viz->visualize();
            // reset bools
            has_left = false;
            // move buffer forward
            time_buffer = time;
            img0_buffer = img0.clone();
        }


        // If we are in stereo mode and have both left and right, then process
        if(max_cameras==2 && has_left && has_right) {
            // process once we have initialized with the GT
            Eigen::Matrix<double, 17, 1> imustate;
            if(!gt_states.empty() && !sys->intialized() && DatasetReader::get_gt_state(time_buffer,imustate,gt_states)) {
                //biases are pretty bad normally, so zero them
                //imustate.block(11,0,6,1).setZero();
                sys->initialize_with_gt(imustate);
            } else if(gt_states.empty() || sys->intialized()) {
                // 每次都是处理上一帧
                sys->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1);
                std::cout << "img type: " << img0.type() << "\n";
                cv::remap(img0_buffer, sys->img01_rectify(cv::Rect(0,0,img0_buffer.cols, img0_buffer.rows)), sys->mapx_cam0, sys->mapy_cam0, cv::INTER_CUBIC);
                cv::remap(img1_buffer, sys->img01_rectify(cv::Rect(img0_buffer.cols,0,img1_buffer.cols, img1_buffer.rows)), sys->mapx_cam1, sys->mapy_cam1, cv::INTER_CUBIC);
//                cv::remap(img0_buffer, left_rectify, sys->mapx_cam0, sys->mapy_cam0, cv::INTER_CUBIC);
//                cv::remap(img1_buffer, right_rectify, sys->mapx_cam1, sys->mapy_cam1, cv::INTER_CUBIC);
                cv::Mat lr_rectify;
                lr_rectify.create(sys->img01_rectify.rows*2/3, sys->img01_rectify.cols*2/3, 0);
                cv::resize(sys->img01_rectify, lr_rectify, cv::Size(lr_rectify.cols, lr_rectify.rows));
                cv::imshow("lr_rectify", lr_rectify);
//                cv::imwrite("/home/dji/Documents/rolling_shutter/tum_rs_rectify/lr_rectify_"+std::to_string(int64_t(time_buffer*1e6))+".png", lr_rectify);
                std::cout << "img01 w/h: " << sys->img01_rectify.cols << " / " << sys->img01_rectify.rows << "\n";
                std::cout << "img01_resize w/h: " << lr_rectify.cols << " / " << lr_rectify.rows << "\n";
                cv::waitKey(10);
            }
            // visualize
            viz->visualize();
            // reset bools
            has_left = false;
            has_right = false;
            // move buffer forward
            time_buffer = time;
            img0_buffer = img0.clone();
            img1_buffer = img1.clone();
        }

    }

    // Final visualization
    viz->visualize_final();

    // Finally delete our system
    delete sys;
    delete viz;


    // Done!
    return EXIT_SUCCESS;

}


















