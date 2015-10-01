/*********************************************************************************************//**
* @file aruco_mapping.cpp
*
* Copyright (c)
* Smart Robotic Systems
* March 2015
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/* Author: Jan Bacik */

#ifndef ARUCO_MAPPING_CPP
#define ARUCO_MAPPING_CPP

#include <aruco_mapping.h>

namespace aruco_mapping
{

ArucoMapping::ArucoMapping(ros::NodeHandle *nh) :
  listener_ (new tf::TransformListener),  // Initialize TF Listener  
  num_of_markers_ (10),                   // Number of used markers
  marker_size_(0.1),                      // Marker size in m
  calib_filepath_("empty"),               // Calibration filepath
  space_type_ ("plane"),                  // Space type - 2D plane 
  roi_allowed_ (false),                   // ROI not allowed by default
  first_marker_detected_(false),          // First marker not detected by defualt
  marker_counter_(0),                     // Reset marker counter
  roi_x_(0),                              // ROI_X
  roi_y_(0),                              // ROI_Y
  roi_w_(0),                              // ROI_W
  roi_h_(0)                               // ROI_H
  
{
  double temp_marker_size;  
  
  //Parse params from launch file 
  nh->getParam("/aruco_mapping/calibration_file", calib_filepath_);
  nh->getParam("/aruco_maping/space_type", space_type_);
  nh->getParam("/aruco_mapping/marker_size", temp_marker_size);
  nh->getParam("/aruco_mapping/num_of_markers", num_of_markers_);
  nh->getParam("/aruco_mapping/roi_allowed",roi_allowed_);
  nh->getParam("/aruco_mapping/roi_x", roi_x_);
  nh->getParam("/aruco_mapping/roi_y", roi_y_);
  nh->getParam("/aruco_mapping/roi_w", roi_w_);
  nh->getParam("/aruco_mapping/roi_h", roi_h_);
     
  // Double to float conversion
  marker_size_ = float(temp_marker_size);
  
  if(calib_filepath_ == "empty")
  {
    ROS_WARN("Calibration filename empty! Check the launch file paths");
    throw std::exception();
  }
  else
  {
    ROS_INFO_STREAM("Calibration file path: " << calib_filepath_ );
    ROS_INFO_STREAM("Number of markers: " << num_of_markers_);
    ROS_INFO_STREAM("Marker Size: " << temp_marker_size);
    ROS_INFO_STREAM("Type of space: " << space_type_);
    ROS_INFO_STREAM("ROI allowed: " << roi_allowed_);
    ROS_INFO_STREAM("ROI x-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI y-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI width: "  << roi_w_);
    ROS_INFO_STREAM("ROI height: " << roi_h_);
  }
    
  //ROS publishers
  marker_msg_pub_           = nh->advertise<aruco_mapping::ArucoMarker>("aruco_poses",1);
  marker_visualization_pub_ = nh->advertise<visualization_msgs::Marker>("aruco_markers",1);
          
  //Load data from calibration file
  if(loadCalibrationFile(calib_filepath_) == false);
    throw std::exception();
    
  //Initialize OpenCV window
  cv::namedWindow("Mono8", CV_WINDOW_AUTOSIZE);       
      
  //Resize vector of markers 
  markers_.resize(num_of_markers_);
  
  // Default markers_ initialization
  for(size_t i = 0; i < num_of_markers_;i++)
  {
    markers_[i].active = false;
    markers_[i].marker_id = -1;
    markers_[i].previous_marker_id = -1;
  }
}

ArucoMapping::~ArucoMapping()
{
  delete listener_;
}

bool ArucoMapping::loadCalibrationFile(std::string calib_filename_)
{
  ROS_INFO_STREAM("Parsing data from : " << calib_filename_ );
  try
  {
    //Search for camera matrix and distortion coeffs in calibration textfile
    string camera_matrix_str("camera matrix");
    string distortion_str("distortion");
    string width_str("width");
    string height_str("height");

    // ifstream object
    ifstream file;
    file.open(calib_filename_.c_str());

    // Alocation of memory for calibration data
    cv::Mat  *intrinsics       = new(cv::Mat)(3, 3, CV_64F);
    cv::Mat  *distortion_coeff = new(cv::Mat)(5, 1, CV_64F);
    cv::Size *image_size       = new(cv::Size);

    //Read calibration .txt file line by line
    std::string line;
    int line_counter = 0;
    while(getline(file, line))
    {
      //Read image width and height
      if(line == width_str)
        file >> image_size->width;

      if(line == height_str)
        file >> image_size->height;

      // Read camera matrix 3x3
      if(line == camera_matrix_str)
      {
        for(size_t i = 0; i < 3; i++)
          for(size_t j = 0; j < 3; j++)
            file >> intrinsics->at<double>(i,j);
      }

      // Read distortion 5x1
      if(line == distortion_str)
      {
        for(size_t i = 0; i < 5; i++)
          file >> distortion_coeff->at<double>(i,0);
      }
      line_counter++;
    }

    ROS_DEBUG_STREAM("Image width: " << image_size->width);
    ROS_DEBUG_STREAM("Image height: " << image_size->height);
    ROS_DEBUG_STREAM("Intrinsics:" << std::endl << *intrinsics);
    ROS_DEBUG_STREAM("Distortion: " << *distortion_coeff);

    //Load parameters to aruco_calib_param_ for aruco detection
    aruco_calib_params_.setParams(*intrinsics, *distortion_coeff, *image_size);

    //Simple check if calibration data meets expected format
    if ((intrinsics->at<double>(2,2) == 1) && (distortion_coeff->at<double>(0,4) == 0))
    {
      ROS_INFO_STREAM("Calibration file loaded successfully");
      return true;
    }
    else
    {
      ROS_WARN("Wrong calibration data, check calibration file and filepath");
      return false;
    }
  }
  catch(int e)
  {
    ROS_ERROR("Not able to read data from calibration file!");
  }
}




void ArucoMapping::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
  //Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("Not able to convert sensor_msgs::Image to OpenCV::Mat format: " << e.what());
    return;
  }
  
  // sensor_msgs::Image to OpenCV::Mat structure
  cv::Mat  I;
  I = cv_ptr->image;
  
  // Apply region of interest
  if(roi_allowed_==true)
    I = cv_ptr->image(cv::Rect(roi_x_,roi_y_,roi_w_,roi_h_));

  //Marker detection
  processImage(I,I);
  
  // Show image
  cv::imshow("Mono8", I);
  cv::waitKey(CV_WAIT_KEY);
}

bool ArucoMapping::processImage(cv::Mat input_image,cv::Mat output_image)
{
  aruco::MarkerDetector detector;
  std::vector<aruco::Marker> temp_markers;

  // Set sign of visibility for all markers to false
  for(size_t i = 0; i < num_of_markers_; i++)
    markers_[i].active = false;

  // Save number of previously found markers before new processing
  marker_counter_previous_ = marker_counter_;

  // Call aruco marker detector
  detector.detect(input_image, temp_markers, aruco_calib_params_, marker_size_);

  //===================================================================
  // IF FIRST MARKER IS DETECTED
  //===================================================================
  if((temp_markers.size() > 0) && (first_marker_detected_ == false))
  {
    // Filter detected markers to find one with the lowest ID
    int lowest_marker_id = temp_markers[0].id;
    for(size_t i = 0; i < temp_markers.size(); i++)
    {
      if(temp_markers[i].id < lowest_marker_id)
        lowest_marker_id = temp_markers[i].id;
    }

    first_marker_detected_ = true;
    marker_counter_++;
    markers_[0].active = true;
    markers_[0].previous_marker_id = THIS_IS_FIRST_MARKER;

    //Make the lowest marker identical with world's origin
    identifyWithWorldOrigin(lowest_marker_id);
  }

  //==================================================================
  // FOR EVERY DETECTED MARKER DO
  //==================================================================
  for(size_t i = 0; i < temp_markers.size(); i++)
  {
    //Draw marker convex, ID, cube and axis
    temp_markers[i].draw(output_image, cv::Scalar(0,0,255), CV_WINDOW_MARKER_LINE_WIDTH);
    aruco::CvDrawingUtils::draw3dCube(output_image,temp_markers[i], aruco_calib_params_);
    aruco::CvDrawingUtils::draw3dAxis(output_image,temp_markers[i], aruco_calib_params_);

    // Find out if new or already known marker
    int current_marker_id = temp_markers[i].id;
    int index;
    int j = 0;
    bool already_known = false;

    //==================================================================

    //Is it existing marker?
    while((already_known == false) && (j < marker_counter_))
    {
      if(markers_[j].marker_id == current_marker_id)
      {
        index = j;
        already_known = true;
      }
      j++;
    }

    // For existing marker compute actual camera pose:
    if((index < marker_counter_) && (first_marker_detected_ == true))
    {
       markers_[index].current_camera_tf = arucoMarker2Tf(temp_markers[i]);
       computeCurrentCameraPose(index, true);
    }

    //===================================================================

    //Is it a new marker?
    if(already_known == false)
    {
      index = marker_counter_;
      markers_[index].marker_id = current_marker_id;
      ROS_INFO_STREAM("New marker found:" << current_marker_id);
    }

    // If new marker was found, sign of visibility must be changed
    for(size_t j = 0; j < marker_counter_; j++)
    {
      for(size_t k = 0; k < temp_markers.size(); k++)
      {
        if(markers_[j].marker_id == temp_markers[k].id)
           markers_[j].active = true;
      }
    }

    // For a new marker compute actual camera pose:
    if((index == marker_counter_) && (first_marker_detected_ == true))
    {
      markers_[index].current_camera_tf = arucoMarker2Tf(temp_markers[i]);
      computeCurrentCameraPose(index, false);

      //------------------------------------------------------------------

      // Test, if possible to compute position of a new marker with respect to the previous marker
      bool at_least_one_marker_visible = false;
      int last_marker_id;

      std::stringstream camera_tf_id_old;
      std::stringstream marker_tf_id_old;
      std::stringstream camera_tf_id;
      camera_tf_id << "camera_" << index;

      for(size_t j = 0; j < index; j++)
      {
        if((markers_[j].active == true) && (at_least_one_marker_visible == false))
        {
          if(markers_[j].previous_marker_id != -1)
          {
            at_least_one_marker_visible = true;
            camera_tf_id_old << "camera_" << j;
            marker_tf_id_old << "marker_" << j;
            markers_[index].previous_marker_id = j;
            last_marker_id = j;
          }
        }
      }

      //-----------------------------------------------------------------

      // If new position can be calculated
      if(at_least_one_marker_visible == true)
      {
        // Generate TFs for listener
        for(size_t j = 0; j < 10; j++)
        {
          // TF from previous marker to its camera
          broadcaster_.sendTransform(tf::StampedTransform(markers_[last_marker_id].current_camera_tf,ros::Time::now(),marker_tf_id_old.str(),camera_tf_id_old.str()));

          // TF from old camera to new camera
          broadcaster_.sendTransform(tf::StampedTransform(markers_[index].current_camera_tf,ros::Time::now(),camera_tf_id_old.str(),camera_tf_id.str()));
          ros::Duration(BROADCAST_WAIT_INTERVAL).sleep();
        }

        // Compute TF between two markers
        listener_->waitForTransform(marker_tf_id_old.str(),camera_tf_id.str(),ros::Time(0),ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
        try
        {
          broadcaster_.sendTransform(tf::StampedTransform(markers_[last_marker_id].current_camera_tf,ros::Time::now(),marker_tf_id_old.str(),camera_tf_id_old.str()));
          broadcaster_.sendTransform(tf::StampedTransform(markers_[index].current_camera_tf,ros::Time::now(),camera_tf_id_old.str(),camera_tf_id.str()));

          listener_->lookupTransform(marker_tf_id_old.str(),camera_tf_id.str(),ros::Time(0),markers_[index].transform_to_globe);
        }
        catch(tf::TransformException &e)
        {
          ROS_ERROR_STREAM("Not able to obtain TF Transform, an exception" << e.what());
        }

        // Save pose of computed TF
        tf::Vector3 marker_position = markers_[index].transform_to_previous.getOrigin();
        tf::Quaternion marker_quaternion = markers_[index].transform_to_previous.getRotation();

        // If plane "space_type" selected, we consider roll and pitch and z-axis zero
        if(space_type_ == "plane")
        {
          double roll, pitch, yaw;
          tf::Matrix3x3(marker_quaternion).getRPY(roll,pitch,yaw);
          roll  = 0;
          pitch = 0;
          marker_quaternion.setRPY(pitch,roll,yaw);
          marker_position.setZ(0);
        }
        markers_[index].transform_to_previous.setRotation(marker_quaternion);
        markers_[index].transform_to_previous.setOrigin(marker_position);

        marker_position = markers_[index].transform_to_previous.getOrigin();
        markers_[index].pose_to_previous.position.x = marker_position.getX();
        markers_[index].pose_to_previous.position.y = marker_position.getY();
        markers_[index].pose_to_previous.position.z = marker_position.getZ();

        marker_quaternion = markers_[index].transform_to_previous.getRotation();
        markers_[index].pose_to_previous.orientation.x = marker_quaternion.getX();
        markers_[index].pose_to_previous.orientation.y = marker_quaternion.getY();
        markers_[index].pose_to_previous.orientation.z = marker_quaternion.getZ();
        markers_[index].pose_to_previous.orientation.w = marker_quaternion.getW();

        // increasing count of markers
        marker_counter_++;

        // Compute camera position with respect to new marker
        computeCurrentCameraPose(index, true);

        // Publish all known TFs
        publishTfs(false);
      }
    }

    // Compute marker global position if relative position to previous is known
    if((marker_counter_previous_ < marker_counter_) && (first_marker_detected_ == true))
    {
      // Publish all known TFs five times for listener
      for(size_t j = 0; j < 5; j++)
        publishTfs(false);

      computeMarkerGlobalPosition(index);
    }
  }

  //===============================================================
  //COMPUTE GLOBAL CAMERA POSITION
  //===============================================================

  bool any_markers_visible = false;
  int num_of_visible_markers = 0;
  int closest_camera_index = 0;

  //Find the closest camera using the shortest distance
  if(first_marker_detected_ == true)
  {
    double min_size = INIT_MIN_SIZE_VALUE;
    for(size_t j = 0; j < num_of_markers_; j++)
    {
      double a;
      double b;
      double c;
      double size;

      //Calculate a camera distance for every active/visible marker
      if(markers_[j].active == true)
      {
        a = markers_[j].current_camera_pose.position.x;
        b = markers_[j].current_camera_pose.position.y;
        c = markers_[j].current_camera_pose.position.z;
        size =std::sqrt((a * a) + (b * b) + (c * c));

        if(size < min_size)
        {
          min_size = size;
          closest_camera_index = j;
        }

        any_markers_visible = true;
        num_of_visible_markers++;
      }
    }
  }

  // Publish all known TFs
  if(first_marker_detected_ == true)
    publishTfs(true);

  //====================================================================

  //Compute global camera pose from the closest marker position
  if((first_marker_detected_ == true) && (any_markers_visible == true))
  {
    std::stringstream closest_camera;
    closest_camera << "camera_" << closest_camera_index;

    listener_->waitForTransform("world",closest_camera.str(),ros::Time(0),ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
    try
    {
      listener_->lookupTransform("world",closest_camera.str(),ros::Time(0), world_position_transform_);
    }
    catch(tf::TransformException &e)
    {
      ROS_ERROR_STREAM("Not able to lookup transform:" << e.what());
    }

    // Save TF to Pose
    tf::Vector3 marker_position = world_position_transform_.getOrigin();
    world_position_geometry_msg_.position.x = marker_position.getX();
    world_position_geometry_msg_.position.y = marker_position.getY();
    world_position_geometry_msg_.position.z = marker_position.getZ();

    tf::Quaternion marker_quaternion = world_position_transform_.getRotation();
    world_position_geometry_msg_.orientation.x = marker_quaternion.getX();
    world_position_geometry_msg_.orientation.y = marker_quaternion.getY();
    world_position_geometry_msg_.orientation.z = marker_quaternion.getZ();
    world_position_geometry_msg_.orientation.w = marker_quaternion.getW();
  }

  //====================================================================

  // Publish all known markers
  if(first_marker_detected_ == true)
    publishTfs(true);

  //====================================================================

  //Publish ArucoMarker custom message to "aruco_poses" topic
  publishMarkerCustomMsg(any_markers_visible, num_of_visible_markers);

  return true;
}

void ArucoMapping::publishTfs(bool world_option)
{
  for(size_t i = 0; i < marker_counter_; i++)
  {
    // Actual Marker
    std::stringstream marker_tf_id;
    marker_tf_id << "marker_" << i;

    // Older marker
    std::stringstream marker_tf_id_old;
    if(i == 0)
      marker_tf_id_old << "world";
    else
      marker_tf_id_old << "marker_" << markers_[i].previous_marker_id;

    broadcaster_.sendTransform(tf::StampedTransform(markers_[i].transform_to_globe,ros::Time::now(),marker_tf_id_old.str(),marker_tf_id.str()));

    // Position of camera to its marker
    std::stringstream camera_tf_id;
    camera_tf_id << "camera_" << i;
    broadcaster_.sendTransform(tf::StampedTransform(markers_[i].current_camera_tf,ros::Time::now(),marker_tf_id.str(),camera_tf_id.str()));

    if(world_option == true)
    {
      // Global position of marker TF
      std::stringstream marker_globe;
      marker_globe << "marker_globe_" << i;
      broadcaster_.sendTransform(tf::StampedTransform(markers_[i].transform_to_globe,ros::Time::now(),"world", marker_globe.str()));
    }

    // Publish RVIZ Markers
    publishMarkerForVisualization(markers_[i].pose_to_globe,markers_[i].marker_id,i);
  }

  // Global Position of camera
  if(world_option == true)
    broadcaster_.sendTransform(tf::StampedTransform(world_position_transform_,ros::Time::now(),"world","camera_position"));
}

void ArucoMapping::publishMarkerForVisualization(geometry_msgs::Pose marker_pose, int marker_id, int index)
{
  visualization_msgs::Marker rviz_marker;

  if(index == 0)
    rviz_marker.header.frame_id = "world";
  else
  {
    std::stringstream marker_tf_id_old;
    marker_tf_id_old << "marker_" << markers_[index].previous_marker_id;
    rviz_marker.header.frame_id = marker_tf_id_old.str();
  }

    rviz_marker.header.stamp=ros::Time::now();
    rviz_marker.ns = "basic_shapes";
    rviz_marker.id = marker_id;
    rviz_marker.type = visualization_msgs::Marker::CUBE;
    rviz_marker.action = visualization_msgs::Marker::ADD;

    rviz_marker.pose = marker_pose;
    rviz_marker.scale.x = marker_size_;
    rviz_marker.scale.y = marker_size_;
    rviz_marker.scale.z = RVIZ_MARKER_HEIGHT;

    rviz_marker.color.r = RVIZ_MARKER_COLOR_R;
    rviz_marker.color.g = RVIZ_MARKER_COLOR_G;
    rviz_marker.color.b = RVIZ_MARKER_COLOR_B;
    rviz_marker.color.a = RVIZ_MARKER_COLOR_A;

    rviz_marker.lifetime = ros::Duration(RVIZ_MARKER_LIFETIME);

    marker_visualization_pub_.publish(rviz_marker);
}

void ArucoMapping::publishMarkerCustomMsg(bool any_markers_visible, int num_of_visible_markers)
{
  // Create ArucoMarker message instance
  aruco_mapping::ArucoMarker marker_msg;

  if((any_markers_visible == true))
  {
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = "world";
    marker_msg.numberOfMarkers = num_of_visible_markers;
    marker_msg.visibility = true;
    marker_msg.globalPose = world_position_geometry_msg_;
    marker_msg.markersID.clear();
    marker_msg.markersPose.clear();
    marker_msg.cameraPose.clear();
    for(int j = 0; j < marker_counter_; j++)
    {
      if(markers_[j].active == true)
      {
        marker_msg.markersID.push_back(markers_[j].marker_id);
        marker_msg.markersPose.push_back(markers_[j].pose_to_globe);
        marker_msg.cameraPose.push_back(markers_[j].current_camera_pose);
      }
    }
  }
  else
  {
     marker_msg.header.stamp = ros::Time::now();
     marker_msg.header.frame_id = "world";
     marker_msg.numberOfMarkers = num_of_visible_markers;
     marker_msg.visibility = false;
     marker_msg.markersID.clear();
     marker_msg.markersPose.clear();
  }

  // Publish
  marker_msg_pub_.publish(marker_msg);
}

void ArucoMapping::identifyWithWorldOrigin(int lowest_marker_id)
{
  markers_[0].marker_id = lowest_marker_id;
  markers_[0].pose_to_previous.position.x = 0;
  markers_[0].pose_to_previous.position.y = 0;
  markers_[0].pose_to_previous.position.z = 0;

  markers_[0].pose_to_previous.orientation.x = 0;
  markers_[0].pose_to_previous.orientation.y = 0;
  markers_[0].pose_to_previous.orientation.z = 0;
  markers_[0].pose_to_previous.orientation.w = 1;

  markers_[0].pose_to_globe.position.x = 0;
  markers_[0].pose_to_globe.position.y = 0;
  markers_[0].pose_to_globe.position.z = 0;

  markers_[0].pose_to_globe.orientation.x = 0;
  markers_[0].pose_to_globe.orientation.y = 0;
  markers_[0].pose_to_globe.orientation.z = 0;
  markers_[0].pose_to_globe.orientation.w = 1;

  // Convert pose information to TF
  tf::Vector3 position;
  position.setX(0);
  position.setY(0);
  position.setZ(0);

  tf::Quaternion quaternion;
  quaternion.setX(0);
  quaternion.setY(0);
  quaternion.setZ(0);
  quaternion.setW(1);

  markers_[0].transform_to_previous.setOrigin(position);
  markers_[0].transform_to_previous.setRotation(quaternion);

  // Relative and global pose of the first marker is the same
  markers_[0].transform_to_globe = markers_[0].transform_to_previous;

  ROS_INFO_STREAM("Marker ID: " << lowest_marker_id << " is set as world's origin");
}

void ArucoMapping::computeCurrentCameraPose(int index, bool invert)
{
  if(invert == true)
    markers_[index].current_camera_tf = markers_[index].current_camera_tf.inverse();

  tf::Vector3 marker_position = markers_[index].current_camera_tf.getOrigin();
  markers_[index].current_camera_pose.position.x = marker_position.getX();
  markers_[index].current_camera_pose.position.y = marker_position.getY();
  markers_[index].current_camera_pose.position.z = marker_position.getZ();

  tf::Quaternion marker_quaternion = markers_[index].current_camera_tf.getRotation();
  markers_[index].current_camera_pose.orientation.x = marker_quaternion.getX();
  markers_[index].current_camera_pose.orientation.y = marker_quaternion.getY();
  markers_[index].current_camera_pose.orientation.z = marker_quaternion.getZ();
  markers_[index].current_camera_pose.orientation.w = marker_quaternion.getW();

}

void ArucoMapping::computeMarkerGlobalPosition(int index)
{
  std::stringstream marker_globe;
  marker_globe << "marker_" << index;

  listener_->waitForTransform("world",marker_globe.str(),ros::Time(0),ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
  try
  {
    listener_->lookupTransform("world",marker_globe.str(),ros::Time(0),markers_[index].transform_to_globe);
  }
  catch(tf::TransformException &e)
  {
    ROS_ERROR_STREAM("Not able to lookup transform: " << e.what());
  }

  // Saving TF to Pose
  tf::Vector3 marker_position = markers_[index].transform_to_globe.getOrigin();
  markers_[index].pose_to_globe.position.x = marker_position.getX();
  markers_[index].pose_to_globe.position.y = marker_position.getY();
  markers_[index].pose_to_globe.position.z = marker_position.getZ();

  tf::Quaternion marker_quaternion = markers_[index].transform_to_globe.getRotation();
  markers_[index].pose_to_globe.orientation.x = marker_quaternion.getX();
  markers_[index].pose_to_globe.orientation.y = marker_quaternion.getY();
  markers_[index].pose_to_globe.orientation.z = marker_quaternion.getZ();
  markers_[index].pose_to_globe.orientation.w = marker_quaternion.getW();
}

tf::Transform ArucoMapping::arucoMarker2Tf(const aruco::Marker &marker)
{
  cv::Mat marker_rotation(3,3, CV_32FC1);
  cv::Rodrigues(marker.Rvec, marker_rotation);
  cv::Mat marker_translation = marker.Tvec;

  cv::Mat rotate_to_ros(3,3,CV_32FC1);
  rotate_to_ros.at<float>(0,0) = -1.0;
  rotate_to_ros.at<float>(0,1) = 0;
  rotate_to_ros.at<float>(0,2) = 0;
  rotate_to_ros.at<float>(1,0) = 0;
  rotate_to_ros.at<float>(1,1) = 0;
  rotate_to_ros.at<float>(1,2) = 1.0;
  rotate_to_ros.at<float>(2,0) = 0.0;
  rotate_to_ros.at<float>(2,1) = 1.0;
  rotate_to_ros.at<float>(2,2) = 0.0;

  marker_rotation = marker_rotation * rotate_to_ros.t();

  // Origin solution
  tf::Matrix3x3 marker_tf_rot(marker_rotation.at<float>(0,0),marker_rotation.at<float>(0,1),marker_rotation.at<float>(0,2),
                              marker_rotation.at<float>(1,0),marker_rotation.at<float>(1,1),marker_rotation.at<float>(1,2),
                              marker_rotation.at<float>(2,0),marker_rotation.at<float>(2,1),marker_rotation.at<float>(2,2));

  tf::Vector3 marker_tf_tran(marker_translation.at<float>(0,0),
                             marker_translation.at<float>(1,0),
                             marker_translation.at<float>(2,0));

  return tf::Transform(marker_tf_rot, marker_tf_tran);
}

}  //aruco_mapping namespace

#endif  //ESTIMATOR_CPP
