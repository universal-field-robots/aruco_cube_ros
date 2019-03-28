/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Universal Field Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its 
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Rhys McKercher, Joshua Owen
 *********************************************************************/
#ifndef __ARUCO_CUBE_ROS_NODELET_H__
#define __ARUCO_CUBE_ROS_NODELET_H__

//ros
#include <ros/ros.h>
#include <nodelet/nodelet.h>
//d-reconfigure
#include <dynamic_reconfigure/server.h>
#include <aruco_cube_ros/ArucoCubeConfig.h>
//tf
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
//messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <aruco_cube_ros/GetPose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//opencv includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
//stds
#include <algorithm>


namespace aruco_cube
{

class ArucoCubeNodelet : public nodelet::Nodelet
{
  public:
    ArucoCubeNodelet();
    ~ArucoCubeNodelet();

  private:
    ros::NodeHandle n_;
    ros::NodeHandle np_;
    ros::NodeHandle n_mt_;
    ros::NodeHandle np_mt_;

    ros::Publisher  pub_pose_;
    ros::Subscriber sub_image_;
    ros::Subscriber sub_info_;
    ros::ServiceServer srv_aruco_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    bool broadcast_transform_;
    bool publish_pose_;

    // Dynamic reconfigure
    boost::shared_ptr<dynamic_reconfigure::Server<aruco_cube::ArucoCubeConfig> > dsrv_;
    boost::recursive_mutex param_reconfigure_mutex_;
    dynamic_reconfigure::Server<aruco_cube::ArucoCubeConfig>::CallbackType param_reconfigure_callback_;
    bool dynamic_reconfigure_initialized_;
    aruco_cube::ArucoCubeConfig config_;

    std::string frame_id_, base_frame_id_;
    std::string parent_frame_id_, child_frame_id_;
    std::string pose_topic_;
    std::string image_topic_, info_topic_, srv_topic_;
    double marker_side_length_, cube_side_length_;
    std::vector <double> cube_offset_;
    int dictionary_id_;

    cv::Mat camMatrix_, distCoeffs_;
    std::vector <std::vector <cv::Point3f> > markers_;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::Board> arucoBoard_;
    std::vector <int> markerIds_;

    std::vector<geometry_msgs::Pose> circular_buffer_;
    std::vector<geometry_msgs::Pose>::iterator insert_it_;
    int window_size_;

    geometry_msgs::PoseWithCovarianceStamped pose_msg_;
    std::vector<double> covariance_;


    /** \brief Initaliser function called when ArucoCubeNodelet class is created.*/
    void onInit();

    /** \brief Initalise dynamic reconfigure variables.*/
    void initDynamicReconfigure(ros::NodeHandle& n);

    /** \brief Callback function for dynamic reconfigure.*/
    void reconfigureCB(aruco_cube::ArucoCubeConfig &config, uint32_t level);

    /** \brief Update dynamic reconfigure variables.*/
    void updateDynamicReconfigure(aruco_cube::ArucoCubeConfig config);

    /** \brief Callback for camera info topic.*/
    void infoCB(const sensor_msgs::CameraInfoConstPtr& info_msg);

    /** \brief Callback for camera image topic.*/
    void imageCB(const sensor_msgs::ImageConstPtr& image_msg);

    /** \brief Service callback which returns the estimate pose of the aruco cube.*/
    bool arucoServiceCB(aruco_cube::GetPoseRequest &req, aruco_cube::GetPoseResponse &res);

    /** \brief Finds aruco cube pose from a given image.*/
    bool findAruco(const sensor_msgs::ImageConstPtr& image_msg);

    /** \brief Gets the translation and rotation vector (rodrigues representation)
        for the location of the aruco cubes centre.*/
    bool getCubeCentre(cv::Mat &frame, cv::Vec3d &tvec, cv::Vec3d &rvec);

    /** \brief Adds pose to a circular buffer of size 'window_size_' and returns filtered pose.*/
    geometry_msgs::Pose filterPose(geometry_msgs::Pose pose_in);

    /** \brief Removes outliers in the buffer by guassian distrution and returns a point which position is selective
        by minimising square distances between all other positions in the pose buffer.*/
    geometry_msgs::Point getBestPosition(void);

    /** \brief Removes outliers in the buffer by guassian distrution and returns a quaternion which is selective
        by minimising the shortestAngle beteen all other poses in the buffer.*/
    geometry_msgs::Quaternion getBestRotation(void);

    /** \brief Returnsa vector of indices which corresponds to the elements in the input list that are within
        a single standard deviation of a guasisan distrubutuion.*/
    std::vector<int> getIndicesInsideStdDeviation(std::vector<double> element_list);

    /** \brief Pose estimation for a board of markers.*/
    int estimatePoseBoard(cv::InputArrayOfArrays _corners, cv::InputArray _ids, const cv::Ptr<cv::aruco::Board> &board,
        cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs, cv::OutputArray _rvec, cv::OutputArray _tvec, bool useExtrinsicGuess);

    /** \brief Given a board configuration and a set of detected markers, returns the corresponding
        image points and object points to call solvePnP.*/
    void getBoardObjectAndImagePoints(const cv::Ptr<cv::aruco::Board> &board, cv::InputArrayOfArrays detectedCorners,
        cv::InputArray detectedIds, cv::OutputArray objPoints, cv::OutputArray imgPoints);

};

}//namespace

#endif
