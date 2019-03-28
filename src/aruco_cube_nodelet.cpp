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

#include <pluginlib/class_list_macros.h>
#include <aruco_cube_ros/aruco_cube_nodelet.h>

namespace aruco_cube
{

ArucoCubeNodelet::ArucoCubeNodelet():
  tfListener_(tfBuffer_),
  covariance_(6),
  cube_offset_(3),
  marker_side_length_(0.0),
  cube_side_length_(0.0),
  dictionary_id_(0)
{
}


ArucoCubeNodelet::~ArucoCubeNodelet()
{
}

void ArucoCubeNodelet::onInit()
{

  //Node handles
  n_ = getNodeHandle();
  np_ = getPrivateNodeHandle();
  n_mt_ = getMTNodeHandle();
  np_mt_ = getMTPrivateNodeHandle();

  //Get params from server
  np_.param("pose_topic", pose_topic_, std::string("aruco_cube_pose"));
  np_.param("srv_topic", srv_topic_, std::string("get_aruco_pose"));
  np_.param("cam_info_topic", info_topic_, std::string("/camera/color/image_raw"));
  np_.param("image_topic", image_topic_, std::string("/camera/color/image_raw"));
  np_.param("parent_frame_id", parent_frame_id_, std::string("base_link"));
  np_.param("child_frame_id", child_frame_id_, std::string("aruco_cube_link"));

  np_.param("stream_image_topic", config_.stream_image, true);
  np_.param("display_opencv_window", config_.display_opencv_window, false);
  np_.param("broadcast_transform", config_.broadcast_transform, false);
  np_.param("publish_pose", config_.publish_pose, true);
  np_.param("filter_window_size", config_.window_size, 0);
  np_.param("do_corner_refinement", config_.do_corner_refinement, true);
  np_.param("corner_refinement_win_size", config_.corner_refinement_win_size, 2);
  np_.param("corner_refinement_max_iterations", config_.corner_refinement_max_iterations, 50);
  np_.param("corner_refinement_min_accuracy", config_.corner_refinement_min_accuracy, 0.9);
  np_.param("error_correction_rate", config_.error_correction_rate, 0.35);
  np_.param("max_erroneous_bits_in_border_rate", config_.max_erroneous_bits_in_border_rate, 0.6);

  np_.getParam("cube_offsets", cube_offset_);
  np_.getParam("covariance", covariance_);

  //Check if aruco specific paramters are set, shutdown if not.
  if(!(np_.getParam("marker_side_length", marker_side_length_) &&
      np_.getParam("cube_side_length", cube_side_length_) &&
      np_.getParam("aruco_dictionary_id", dictionary_id_) &&
      np_.getParam("marker_ids", markerIds_)))
  {
    ROS_ERROR("[ArucoCubeNodelet] Failed to fetch aruco specific parameters.");
    ros::shutdown();
  }

  //initalise dynamic reconfigure
  initDynamicReconfigure(np_);

  //Setup publishers/subscribers
  sub_info_ = n_.subscribe(info_topic_, 1, &ArucoCubeNodelet::infoCB, this);
  pub_pose_ =  n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_, 10);

  //This is the marker cube description 5 sides are used because the bottom is assumed to be a mount point
  markers_ = {
    {{(float)(-(0.5 * marker_side_length_)-cube_offset_[0]), (float)(-(0.5 * cube_side_length_)-cube_offset_[1]), (float)( (0.5 * marker_side_length_)-cube_offset_[2])},
     {(float)( (0.5 * marker_side_length_)-cube_offset_[0]), (float)(-(0.5 * cube_side_length_)-cube_offset_[1]), (float)( (0.5 * marker_side_length_)-cube_offset_[2])},
     {(float)( (0.5 * marker_side_length_)-cube_offset_[0]), (float)(-(0.5 * cube_side_length_)-cube_offset_[1]), (float)(-(0.5 * marker_side_length_)-cube_offset_[2])},
     {(float)(-(0.5 * marker_side_length_)-cube_offset_[0]), (float)(-(0.5 * cube_side_length_)-cube_offset_[1]), (float)(-(0.5 * marker_side_length_)-cube_offset_[2])}},

    {{(float)((0.5 * cube_side_length_)-cube_offset_[0]), (float)(-(0.5 * marker_side_length_)-cube_offset_[1]), (float)( (0.5 * marker_side_length_)-cube_offset_[2])},
     {(float)((0.5 * cube_side_length_)-cube_offset_[0]), (float)( (0.5 * marker_side_length_)-cube_offset_[1]), (float)( (0.5 * marker_side_length_)-cube_offset_[2])},
     {(float)((0.5 * cube_side_length_)-cube_offset_[0]), (float)( (0.5 * marker_side_length_)-cube_offset_[1]), (float)(-(0.5 * marker_side_length_)-cube_offset_[2])},
     {(float)((0.5 * cube_side_length_)-cube_offset_[0]), (float)(-(0.5 * marker_side_length_)-cube_offset_[1]), (float)(-(0.5 * marker_side_length_)-cube_offset_[2])}},

    {{(float)( (0.5 * marker_side_length_)-cube_offset_[0]), (float)((0.5 * cube_side_length_)-cube_offset_[1]), (float)( (0.5 * marker_side_length_)-cube_offset_[2])},
     {(float)(-(0.5 * marker_side_length_)-cube_offset_[0]), (float)((0.5 * cube_side_length_)-cube_offset_[1]), (float)( (0.5 * marker_side_length_)-cube_offset_[2])},
     {(float)(-(0.5 * marker_side_length_)-cube_offset_[0]), (float)((0.5 * cube_side_length_)-cube_offset_[1]), (float)(-(0.5 * marker_side_length_)-cube_offset_[2])},
     {(float)( (0.5 * marker_side_length_)-cube_offset_[0]), (float)((0.5 * cube_side_length_)-cube_offset_[1]), (float)(-(0.5 * marker_side_length_)-cube_offset_[2])}},

    {{(float)(-(0.5 * cube_side_length_)-cube_offset_[0]), (float)( (0.5 * marker_side_length_)-cube_offset_[1]), (float)( (0.5 * marker_side_length_)-cube_offset_[2])},
	   {(float)(-(0.5 * cube_side_length_)-cube_offset_[0]), (float)(-(0.5 * marker_side_length_)-cube_offset_[1]), (float)( (0.5 * marker_side_length_)-cube_offset_[2])},
     {(float)(-(0.5 * cube_side_length_)-cube_offset_[0]), (float)(-(0.5 * marker_side_length_)-cube_offset_[1]), (float)(-(0.5 * marker_side_length_)-cube_offset_[2])},
     {(float)(-(0.5 * cube_side_length_)-cube_offset_[0]), (float)( (0.5 * marker_side_length_)-cube_offset_[1]), (float)(-(0.5 * marker_side_length_)-cube_offset_[2])}},

    {{(float)(-(0.5 * marker_side_length_)-cube_offset_[0]), (float)( (0.5 * marker_side_length_)-cube_offset_[1]), (float)((0.5 * cube_side_length_)-cube_offset_[2])},
     {(float)( (0.5 * marker_side_length_)-cube_offset_[0]), (float)( (0.5 * marker_side_length_)-cube_offset_[1]), (float)((0.5 * cube_side_length_)-cube_offset_[2])},
     {(float)( (0.5 * marker_side_length_)-cube_offset_[0]), (float)(-(0.5 * marker_side_length_)-cube_offset_[1]), (float)((0.5 * cube_side_length_)-cube_offset_[2])},
     {(float)(-(0.5 * marker_side_length_)-cube_offset_[0]), (float)(-(0.5 * marker_side_length_)-cube_offset_[1]), (float)((0.5 * cube_side_length_)-cube_offset_[2])}}
  };

  detectorParams_ = cv::aruco::DetectorParameters::create();
	dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id_));
  arucoBoard_ = cv::aruco::Board::create(markers_, dictionary_, markerIds_);
}


void ArucoCubeNodelet::infoCB(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  cv::Mat K(3, 3, CV_64FC1, (void *) info_msg->K.data());
  camMatrix_ = K.clone();

  cv::Mat D(5, 1, CV_64FC1, (void *) info_msg->D.data());
  distCoeffs_ = D.clone();

  sub_info_.shutdown();

  if(config_.stream_image)
  {
    sub_image_ = n_.subscribe(image_topic_, 10, &ArucoCubeNodelet::imageCB, this);
  }

  srv_aruco_ = n_.advertiseService(srv_topic_, &ArucoCubeNodelet::arucoServiceCB, this);

  ROS_INFO("Camera info data collected");
}


bool ArucoCubeNodelet::arucoServiceCB(aruco_cube::GetPoseRequest &req, aruco_cube::GetPoseResponse &res)
{
  res.success = false;

  //reset buffer
  circular_buffer_.clear();

  ros::Time last_msg_time = ros::Time::now();

  param_reconfigure_mutex_.lock();
  //get image from topic and fill buffer
  while (circular_buffer_.size() < config_.window_size)
  {
    sensor_msgs::ImageConstPtr image_msg = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic_, ros::Duration(5));

    //check if we have already received this image. This avoids reprocessing the same image when reading from latched topics.
    if (image_msg->header.stamp == last_msg_time){
      continue;
    }
    last_msg_time = image_msg->header.stamp;

    if (image_msg == NULL)
    {
        ROS_WARN("[ArucoCube] No image was recieved from topic '%s'", image_topic_.c_str());
        return false;
    }

    if(!findAruco(image_msg))
    {
      ROS_WARN("[ArucoCube] Failed to find aruco cube.");
    }
  }
  param_reconfigure_mutex_.unlock();

  res.aruco_pose = pose_msg_;
  res.success = true;

  return true;
}


void ArucoCubeNodelet::imageCB(const sensor_msgs::ImageConstPtr& image_msg)
{

  param_reconfigure_mutex_.lock();
  findAruco(image_msg);
  param_reconfigure_mutex_.unlock();

  return;
}

bool ArucoCubeNodelet::findAruco(const sensor_msgs::ImageConstPtr& image_msg)
{

  frame_id_ = image_msg->header.frame_id;

  cv_bridge::CvImagePtr cvPtr;

  try
  {
    cvPtr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  cv::Vec3d rvec = {0.0, 0.0, 0.0}, tvec = {0.0, 0.0, 0.0};

  bool foundCube = false;

  if(!cvPtr->image.empty())
  {
		foundCube = getCubeCentre(cvPtr->image, tvec, rvec);
	}
	else
	{
		foundCube = false;
	}

	if(foundCube)
	{
  	// Make Quaternion
	  double angle = sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);
    double axis[] = {rvec[0]/angle, rvec[1]/angle, rvec[2]/angle};

    double sinAngle = sin(angle * 0.5);

    geometry_msgs::Pose pose_in;
    pose_in.orientation.w = cos(angle * 0.5);
    pose_in.orientation.x = axis[0] * sinAngle;
    pose_in.orientation.y = axis[1] * sinAngle;
    pose_in.orientation.z = axis[2] * sinAngle;

    pose_in.position.x = tvec[0];
    pose_in.position.y = tvec[1];
    pose_in.position.z = tvec[2];

    //apply filter if window size is greater then minimum
    if(config_.window_size > 2)
    {
      pose_in = filterPose(pose_in);
    }
    else if(config_.window_size != 0)
    {
      ROS_WARN_ONCE("[aruco_cube] Skipping filter, minimum window size is 3.");
    }

    // Publish pose if desired
    if(config_.publish_pose)
    {
      pose_msg_.header.stamp = image_msg->header.stamp;
      pose_msg_.header.frame_id = frame_id_;
      pose_msg_.pose.pose = pose_in;
      pose_msg_.pose.covariance[0] =  covariance_[0];
      pose_msg_.pose.covariance[7] =  covariance_[1];
      pose_msg_.pose.covariance[14] = covariance_[2];
      pose_msg_.pose.covariance[21] = covariance_[3];
      pose_msg_.pose.covariance[22] = covariance_[4];
      pose_msg_.pose.covariance[28] = covariance_[5];

      pub_pose_.publish(pose_msg_);
    }

    // Publish transform of aruco with repect to parent frame, if desired
    if(config_.broadcast_transform)
    {
      geometry_msgs::TransformStamped transformStamped;

      try
      {
        transformStamped = tfBuffer_.lookupTransform(parent_frame_id_, frame_id_, ros::Time(0));
      }
      catch (tf2::TransformException& e)
      {
        ROS_ERROR("[aruco_cube] %s, no pose or transform will be published for this message.", e.what());
        return false;
      }

      geometry_msgs::Pose pose_out;
      tf2::doTransform(pose_in, pose_out, transformStamped);

      // Send tf
      geometry_msgs::TransformStamped trans;
      trans.header.stamp = image_msg->header.stamp;
      trans.header.frame_id = parent_frame_id_;
      trans.child_frame_id = child_frame_id_;
      trans.transform.translation.x = pose_out.position.x;
      trans.transform.translation.y = pose_out.position.y;
      trans.transform.translation.z = pose_out.position.z;
      trans.transform.rotation = pose_out.orientation;

      static_broadcaster_.sendTransform(trans);
    }

    return true;
  }

  return false;
}


bool ArucoCubeNodelet::getCubeCentre(cv::Mat &frame, cv::Vec3d &tvec, cv::Vec3d &rvec)
{
	std::vector< int > ids;
	std::vector< std::vector<cv::Point2f> > corners, rejected;

  //set detection parameters
  detectorParams_->doCornerRefinement = config_.do_corner_refinement;
  detectorParams_->cornerRefinementWinSize = config_.corner_refinement_win_size;
  detectorParams_->cornerRefinementMaxIterations = config_.corner_refinement_max_iterations;
  detectorParams_->cornerRefinementMinAccuracy = config_.corner_refinement_min_accuracy;
  detectorParams_->errorCorrectionRate = config_.error_correction_rate;
  detectorParams_->maxErroneousBitsInBorderRate = config_.max_erroneous_bits_in_border_rate;

	// detect markers and estimate pose
	cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detectorParams_, rejected);
	cv::aruco::refineDetectedMarkers(frame, arucoBoard_, corners, ids, rejected, camMatrix_, distCoeffs_);

	if (ids.size() > 0)
	{
    if( config_.display_opencv_window)
    {
      cv::aruco::drawDetectedMarkers(frame, corners, ids);
      std::vector<cv::Vec3d> rrvecs, ttvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, marker_side_length_, camMatrix_, distCoeffs_, rrvecs, ttvecs);
      // draw axis for each marker
      for(int i=0; i<ids.size(); i++)
      {
         cv::aruco::drawAxis(frame, camMatrix_, distCoeffs_, rrvecs[i], ttvecs[i], 0.1);
      }
      cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
      cv::imshow("frame", frame);
      cv::waitKey(1);
    }

		if (estimatePoseBoard(corners, ids, arucoBoard_, camMatrix_, distCoeffs_, rvec, tvec, false))
		{
			return true;
		}
	}

	return false;
}


geometry_msgs::Pose ArucoCubeNodelet::filterPose(geometry_msgs::Pose pose_in)
{

  //if we have not got up to window size yet, add to buffer
  if (circular_buffer_.size() < config_.window_size)
  {
    circular_buffer_.push_back(pose_in);
    // if still not at buffer size, return the input pose.
    if (circular_buffer_.size() < config_.window_size)
    {
      return pose_in;
    }
    else
    {
      insert_it_ = circular_buffer_.begin(); //initalise start of buffer now that it is filled
    }
  }
  else
  {
    //if iterator has reach the end of the buffer reset to start.
    if (insert_it_ == circular_buffer_.end())
    {
      insert_it_ = circular_buffer_.begin();
    }
    //store data
    *insert_it_ = pose_in;
    //increament iterator
    ++insert_it_;
  }

  //estimate best pose based of poses in buffer.
  geometry_msgs::Pose best_pose;
  best_pose.position = getBestPosition();
  best_pose.orientation = getBestRotation();

  return best_pose;
}


geometry_msgs::Point ArucoCubeNodelet::getBestPosition(void)
{
  //determine the sum of squared distances between each pair of positions
  std::vector<double> distance_sums(config_.window_size, 0);
  tf2::Vector3 pos_i, pos_j;

  for(int i = 0; i < circular_buffer_.size(); ++i)
  {
    tf2::convert(circular_buffer_[i].position, pos_i);
    for(int j = 0; j < circular_buffer_.size(); ++j)
    {
      tf2::convert(circular_buffer_[j].position, pos_j);
      distance_sums[i] += static_cast<double>(tf2::tf2Distance2(pos_i,pos_j));
    }
  }

  //determine which elements are within a std deviations
  std::vector<int> index_list = getIndicesInsideStdDeviation(distance_sums);

  //find again the sum of squared distances for the positions with-in the std deviations
  std::vector<double> distance_sums_filtered(index_list.size(), 0);
  for(int i = 0; i < index_list.size(); ++i)
  {
    tf2::convert(circular_buffer_[index_list[i]].position , pos_i);
    for(int j = 0; j < index_list.size(); ++j)
    {
      tf2::convert(circular_buffer_[index_list[j]].position , pos_j);
      distance_sums_filtered[i] += static_cast<double>(tf2::tf2Distance2(pos_i,pos_j));
    }
  }

  //find interator to the smallest sum of distances of the filtered set
  std::vector<double>::iterator it_minimum = std::min_element(distance_sums_filtered.begin(), distance_sums_filtered.end());

  //get the  position in the index_list which corresponds to this smallest angle
  int index_smallest_sum = it_minimum - distance_sums_filtered.begin();

  // get index of corresponding best position in the buffer based of distance criteria
  int index_best_pos = index_list[index_smallest_sum];

  return circular_buffer_[index_best_pos].position;
}


geometry_msgs::Quaternion ArucoCubeNodelet::getBestRotation(void)
{

  //determine the sum of shorestangles between each pair of quaternions
  std::vector<double> shortest_angle_sums(config_.window_size, 0);
  tf2::Quaternion quat_i, quat_j;
  for(int i = 0; i < circular_buffer_.size(); ++i)
  {
    tf2::convert(circular_buffer_[i].orientation , quat_i);
    for(int j = 0; j < circular_buffer_.size(); ++j)
    {
      tf2::convert(circular_buffer_[j].orientation , quat_j);
      double angle_short = static_cast<double>(quat_i.angleShortestPath(quat_j));
      shortest_angle_sums[i] += angle_short;
    }
  }

  //determine which elements are within a std deviations
  std::vector<int> index_list = getIndicesInsideStdDeviation(shortest_angle_sums);

  //find again the sum of shorest angle for the quaternions with-in desired distrubtion (1 std deviation)
  std::vector<double> shortest_angle_sums_filtered(index_list.size(), 0);
  for(int i = 0; i < index_list.size(); ++i)
  {
    tf2::convert(circular_buffer_[index_list[i]].orientation , quat_i);
    for(int j = 0; j < index_list.size(); ++j)
    {
      tf2::convert(circular_buffer_[index_list[j]].orientation , quat_j);
      double angle_error = static_cast<double>(quat_i.angleShortestPath(quat_j));
      shortest_angle_sums_filtered[i] += angle_error;
    }
  }

  //find interator to the smallest sum of shortestAngles of the filtered set
  std::vector<double>::iterator it_minimum = std::min_element(shortest_angle_sums_filtered.begin(), shortest_angle_sums_filtered.end());

  //get the  position in the index_list which corresponds to this smallest angle
  int index_smallest_sum = it_minimum - shortest_angle_sums_filtered.begin();

  // get index of corresponding best rotation in the buffer based of smallest angle criteria
  int index_best_rot = index_list[index_smallest_sum];

  return circular_buffer_[index_best_rot].orientation;
}


std::vector<int> ArucoCubeNodelet::getIndicesInsideStdDeviation(std::vector<double> element_list)
{

  //determine standard deviation of sums of distances
  double mean;
  for(int i = 0; i < element_list.size(); ++i)
  {
    mean += element_list[i];
  }
  mean /= element_list.size();

  //determine variance [var = SUM{(X-u)^2} / N]
  double varience;
  for(int i = 0; i < element_list.size(); ++i)
  {
    varience += std::pow((element_list[i] - mean), 2.0);
  }
  varience /= element_list.size();

  //determine standard deviation
  double std_dev = std::sqrt(varience);

  std::vector<int> index_list;
  for(int i = 0; i < element_list.size(); ++i)
  {
    if ((element_list[i] > (mean - std_dev)) && (element_list[i] < (mean + std_dev)))
    {
      index_list.push_back(i);
    }
  }

  return index_list;
}


int ArucoCubeNodelet::estimatePoseBoard(cv::InputArrayOfArrays _corners, cv::InputArray _ids, const cv::Ptr<cv::aruco::Board> &board,
                      cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs, cv::OutputArray _rvec,
                      cv::OutputArray _tvec, bool useExtrinsicGuess) {

    CV_Assert(_corners.total() == _ids.total());

    // get object and image points for the solvePnP function
    cv::Mat objPoints, imgPoints;
    getBoardObjectAndImagePoints(board, _corners, _ids, objPoints, imgPoints);

    CV_Assert(imgPoints.total() == objPoints.total());

    if(objPoints.total() == 0) // 0 of the detected markers in board
        return 0;

    cv::solvePnP(objPoints, imgPoints, _cameraMatrix, _distCoeffs, _rvec, _tvec, useExtrinsicGuess);

    // divide by four since all the four corners are concatenated in the array for each marker
    return (int)objPoints.total() / 4;
}

void ArucoCubeNodelet::getBoardObjectAndImagePoints(const cv::Ptr<cv::aruco::Board> &board, cv::InputArrayOfArrays detectedCorners,
    cv::InputArray detectedIds, cv::OutputArray objPoints, cv::OutputArray imgPoints) {

    CV_Assert(board->ids.size() == board->objPoints.size());
    CV_Assert(detectedIds.total() == detectedCorners.total());

    size_t nDetectedMarkers = detectedIds.total();

    std::vector< cv::Point3f > objPnts;
    objPnts.reserve(nDetectedMarkers);

    std::vector< cv::Point2f > imgPnts;
    imgPnts.reserve(nDetectedMarkers);

    // look for detected markers that belong to the board and get their information
    for(unsigned int i = 0; i < nDetectedMarkers; i++) {
        int currentId = detectedIds.getMat().ptr< int >(0)[i];
        for(unsigned int j = 0; j < board->ids.size(); j++) {
            if(currentId == board->ids[j]) {
                for(int p = 0; p < 4; p++) {
                    objPnts.push_back(board->objPoints[j][p]);
                    imgPnts.push_back(detectedCorners.getMat(i).ptr< cv::Point2f >(0)[p]);
                }
            }
        }
    }

    // create output
    cv::Mat(objPnts).copyTo(objPoints);
    cv::Mat(imgPnts).copyTo(imgPoints);
}

void ArucoCubeNodelet::updateDynamicReconfigure(aruco_cube::ArucoCubeConfig config)
{
  if (!dynamic_reconfigure_initialized_)
  {
    return;
  }

  param_reconfigure_mutex_.lock();
  dsrv_->updateConfig(config);
  config_ = config;
  param_reconfigure_mutex_.unlock();
}


void ArucoCubeNodelet::initDynamicReconfigure(ros::NodeHandle& n)
{
  dsrv_.reset(new dynamic_reconfigure::Server<aruco_cube::ArucoCubeConfig>(param_reconfigure_mutex_, n));
  dynamic_reconfigure_initialized_ = true;

  updateDynamicReconfigure(config_);

  param_reconfigure_callback_ = boost::bind(&ArucoCubeNodelet::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(param_reconfigure_callback_);
}


void ArucoCubeNodelet::reconfigureCB(aruco_cube::ArucoCubeConfig& config, uint32_t level)
{
  updateDynamicReconfigure(config);
}

}//namespace


PLUGINLIB_EXPORT_CLASS(aruco_cube::ArucoCubeNodelet, nodelet::Nodelet);
