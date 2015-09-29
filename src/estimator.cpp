/*********************************************************************************************//**
* @file estimator.cpp
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

#ifndef ESTIMATOR_CPP
#define ESTIMATOR_CPP

#include <estimator.h>

namespace aruco_mapping
{

Estimator::Estimator(ros::NodeHandle *nh) :
  listener_ (new tf::TransformListener),  // Initialize TF Listener  
  num_of_markers_ (10),                   // Number of used markers
  marker_size_(0.1),                      // Marker size in m
  calib_filename_("empty"),               // Calibration filepath
  space_type_ ("plane"),                  // Space type - 2D plane 
  roi_allowed_ (false),                   // ROI not allowed by default
  first_marker_detected_(false),          // First marker not detected by defualt
  lowest_marker_id_(-1),                  // Lowest marker ID
  marker_counter_(0),                     // Reset marker counter
  closest_camera_index_(0),               // Reset closest camera index
  roi_x_(0),                              // ROI_X
  roi_y_(0),                              // ROI_Y
  roi_w_(0),                              // ROI_W
  roi_h_(0)                               // ROI_H
  
{
  double temp_marker_size;  
  
  //Parse params from launch file 
  nh->getParam("/aruco_mapping/calibration_file", calib_filename_);
  nh->getParam("/aruco_mapping/marker_size", temp_marker_size); 
  nh->getParam("/aruco_mapping/num_of_markers", num_of_markers_);
  nh->getParam("/aruco_maping/space_type",space_type_);
  nh->getParam("/aruco_mapping/roi_allowed",roi_allowed_);
  nh->getParam("/aruco_mapping/roi_x",roi_x_);
  nh->getParam("/aruco_mapping/roi_y",roi_y_);
  nh->getParam("/aruco_mapping/roi_w",roi_w_);
  nh->getParam("/aruco_mapping/roi_h",roi_h_);
     
  // Double to float conversion
  marker_size_ = float(temp_marker_size);
  
  if(calib_filename_ == "empty")
    ROS_WARN("Calibration filename empty! Check the launch file paths");
  else
  {
    ROS_INFO_STREAM("Calibration file path: " << calib_filename_ );
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
  marker_msg_pub_           = nh->advertise<aruco_mapping::ArUcoMarkers>("aruco_poses",1);
  marker_visualization_pub_ = nh->advertise<visualization_msgs::Marker>("aruco_markers",1);
          
  //Load data from calibration file
  loadCalibrationFile(calib_filename_);
    
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





Estimator::~Estimator()
{
  delete listener_;
  delete intrinsics_;
  delete distortion_coeff_;
  delete image_size_;
}




bool
Estimator::loadCalibrationFile(std::string calib_filename_)
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
    intrinsics_       = new(cv::Mat)(3, 3, CV_64F);
    distortion_coeff_ = new(cv::Mat)(5, 1, CV_64F);
    image_size_       = new(cv::Size);

    //Read calibration .txt file line by line
    std::string line;
    int line_counter = 0;
    while(getline(file, line))
    {
      //Read image width
      if(line == width_str)
      {
        file >> image_size_->width;
        ROS_DEBUG_STREAM("Image width: " << image_size_->width);
      }

      //Read image height
      if(line == height_str)
      {
        file >> image_size_->height;
        ROS_DEBUG_STREAM("Image height: " << image_size_->height);
      }

      // Read camera matrix 3x3
      if(line == camera_matrix_str)
      {
      for(size_t i = 0; i < 3; i++)
        for(size_t j = 0; j < 3; j++)
          file >> intrinsics_->at<double>(i,j);
          ROS_DEBUG_STREAM("Intrinsics:" << std::endl << *intrinsics_);
      }

      // Read distortion 5x1
      if(line == distortion_str)
      {
        for(size_t i = 0; i < 5; i++)
          file >> distortion_coeff_->at<double>(i,0);
          ROS_DEBUG_STREAM("Distortion: " << *distortion_coeff_);
      }
      line_counter++;
    }

    //Set parameters for aruco detection
    aruco_calib_params_.setParams(*intrinsics_, *distortion_coeff_, *image_size_);

    //Check if calibration data meets expected format
    if ((intrinsics_->at<double>(2,2) == 1) && (distortion_coeff_->at<double>(0,4) == 0))
      ROS_INFO_STREAM("Calibration file loaded successfully");
    else
      ROS_WARN("Wrong calibration data, check calibration file and filepath");
  }
  catch(int e)
  {
    ROS_ERROR_STREAM("Not able to read data from calibration file, an exception n." << e << " occured");
  }
}




void
Estimator::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
  //Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
    return;
  }
  
  // sensor_msgs::Image to OpenCV::Mat structure
  I_=cv_ptr->image;
  
  // Apply region of interest
  if(roi_allowed_==true)
    I_=cv_ptr->image(cv::Rect(roi_x_,roi_y_,roi_w_,roi_h_));

  //Marker detection
  markersFindPattern(I_,I_);
  
  // Show image
  cv::imshow("Mono8", I_);
  cv::waitKey(10);  
}




bool
Estimator::markersFindPattern(cv::Mat input_image,cv::Mat output_image)
{
  aruco::MarkerDetector detector;
  std::vector<aruco::Marker> temp_markers;

  // Set sign of visibility for all markers to false
  for(size_t i = 0; i < num_of_markers_; i++)
    markers_[i].active = false;

  // Save previous count of found markers before new processing
  marker_counter_previous_ = marker_counter_;

  // Call aruco marker detector
  detector.detect(input_image,temp_markers,aruco_calib_params_,marker_size_);

  //--------------------------------------------------------------------
  // First marker is detected

  if((temp_markers.size() > 0) && (first_marker_detected_ == false))
  {
    size_t low_ID;

    // Filter detected markers to find one with the lowest ID
    lowest_marker_id_ = temp_markers[0].id;
    low_ID = 0;
    for(size_t i = 0; i < temp_markers.size(); i++)
    {
      if(temp_markers[i].id < lowest_marker_id_)
      {
        lowest_marker_id_ = temp_markers[i].id;
        low_ID = i;
      }
    }

    ROS_INFO_STREAM("Starting marker ID: " << lowest_marker_id_ );

    //Make the lowest ID maker identical to world's origin
    markers_[0].marker_id = lowest_marker_id_;
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

    // Increase count of actual markers
    marker_counter_++;

    // Set sign of visibility for first marker
    first_marker_detected_ = true;

    ROS_INFO_STREAM("Marker ID: " << lowest_marker_id_ << " is set as world's origin");

    // First marker does not have any previous marker
    markers_[0].previous_marker_id = -2;

    // Set visibility sign
    markers_[0].active = true;
  }

  //--------------------------------------------------------------------
  // Add new markers and create a map

  // Markers are always sorted in ascending order
  for(size_t i = 0; i < temp_markers.size(); i++)
  {
    //Draw marker convex, ID, cube and axis
    temp_markers[i].draw(output_image, cv::Scalar(0,0,255),2);
    aruco::CvDrawingUtils::draw3dCube(output_image,temp_markers[i], aruco_calib_params_);
    aruco::CvDrawingUtils::draw3dAxis(output_image,temp_markers[i], aruco_calib_params_);

    // New or already known marker?
    int current_marker_id = temp_markers[i].id;
    int marker_array_id;

    bool already_known = false;
    int j = 0;
    while((already_known == false) && (j < marker_counter_))
    {
      if(markers_[j].marker_id == current_marker_id)
      {
        marker_array_id = j;
        already_known = true;
        ROS_INFO_STREAM("Existing ID: " << marker_array_id << " was assigned");
      }
      j++;
    }

    if(already_known == false)
    {
      marker_array_id = marker_counter_;
      markers_[marker_array_id].marker_id = current_marker_id;
      already_known = true;
      ROS_INFO_STREAM("New marker");
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

    //------------------------------------------------------
    // Existing marker was found

    if((marker_array_id < marker_counter_) && (first_marker_detected_==true))
    {
      markers_[marker_array_id].current_camera_tf = arucoMarker2Tf(temp_markers[i]);
      markers_[marker_array_id].current_camera_tf = markers_[marker_array_id].current_camera_tf.inverse();

      const tf::Vector3 marker_position = markers_[marker_array_id].current_camera_tf.getOrigin();
      markers_[marker_array_id].current_camera_pose.position.x = marker_position.getX();
      markers_[marker_array_id].current_camera_pose.position.y = marker_position.getY();
      markers_[marker_array_id].current_camera_pose.position.z = marker_position.getZ();

      const tf::Quaternion marker_quaternion = markers_[marker_array_id].current_camera_tf.getRotation();
      markers_[marker_array_id].current_camera_pose.orientation.x = marker_quaternion.getX();
      markers_[marker_array_id].current_camera_pose.orientation.y = marker_quaternion.getY();
      markers_[marker_array_id].current_camera_pose.orientation.z = marker_quaternion.getZ();
      markers_[marker_array_id].current_camera_pose.orientation.w = marker_quaternion.getW();
    }

    //------------------------------------------------------
    // New marker was found

    if((marker_array_id == marker_counter_) && (first_marker_detected_==true))
    {
      markers_[marker_array_id].current_camera_tf = arucoMarker2Tf(temp_markers[i]);

      tf::Vector3 marker_position = markers_[marker_array_id].current_camera_tf.getOrigin();
      markers_[marker_array_id].current_camera_pose.position.x = marker_position.getX();
      markers_[marker_array_id].current_camera_pose.position.y = marker_position.getY();
      markers_[marker_array_id].current_camera_pose.position.z = marker_position.getZ();

      tf::Quaternion marker_quaternion = markers_[marker_array_id].current_camera_tf.getRotation();
      markers_[marker_array_id].current_camera_pose.orientation.x = marker_quaternion.getX();
      markers_[marker_array_id].current_camera_pose.orientation.y = marker_quaternion.getY();
      markers_[marker_array_id].current_camera_pose.orientation.z = marker_quaternion.getZ();
      markers_[marker_array_id].current_camera_pose.orientation.w = marker_quaternion.getW();

      // Prepare variables for naming TFs
      std::stringstream camera_tf_id_old;
      std::stringstream marker_tf_id_old;
      std::stringstream camera_tf_id;
      camera_tf_id << "camera_" << marker_array_id;

      // Flag if any known marker is visible
      bool at_least_one_marker_visible = false;

      // Array ID of markers, which position of new marker is calculated
      int last_marker_id;

      // Test, if it is possible calculate position of a new marker with respect to the old marker
      for(size_t j = 0; j < marker_array_id; j++)
      {
        if((markers_[j].active == true) && (at_least_one_marker_visible == false))
        {
          if(markers_[j].previous_marker_id != -1)
          {
            at_least_one_marker_visible = true;
            camera_tf_id_old << "camera_" << j;
            marker_tf_id_old << "marker_" << j;
            markers_[marker_array_id].previous_marker_id = j;
            last_marker_id = j;
          }
        }
      }

      // New position can be calculated
      if(at_least_one_marker_visible == true)
      {
        // Generating TFs for listener
        for(size_t j = 0; j < 10; j++)
        {
          // TF from previous marker and its camera
          broadcaster_.sendTransform(tf::StampedTransform(markers_[last_marker_id].current_camera_tf,ros::Time::now(),marker_tf_id_old.str(),camera_tf_id_old.str()));

          // TF from old camera to new camera
          broadcaster_.sendTransform(tf::StampedTransform(markers_[marker_array_id].current_camera_tf,ros::Time::now(),camera_tf_id_old.str(),camera_tf_id.str()));
          usleep(100);
        }

        // Calculate TF between two markers
        listener_->waitForTransform(marker_tf_id_old.str(),camera_tf_id.str(),ros::Time(0),ros::Duration(2.0));
        try
        {
          broadcaster_.sendTransform(tf::StampedTransform(markers_[last_marker_id].current_camera_tf,ros::Time::now(),marker_tf_id_old.str(),camera_tf_id_old.str()));
          broadcaster_.sendTransform(tf::StampedTransform(markers_[marker_array_id].current_camera_tf,ros::Time::now(),camera_tf_id_old.str(),camera_tf_id.str()));

          listener_->lookupTransform(marker_tf_id_old.str(),camera_tf_id.str(),ros::Time(0),markers_[marker_array_id].transform_to_globe);
        }
        catch(tf::TransformException &e)
        {
          ROS_ERROR("Not able to obtain TF Transform");
          ros::Duration(2.0).sleep();
        }

        // Save quaternion and origin of calculated TF to pose
        marker_quaternion = markers_[marker_array_id].transform_to_previous.getRotation();

        // If all markers are in the plane, for better accuracy, we known that roll and pitch are zero
        if(space_type_ == "plane")
        {
          double roll, pitch, yaw;
          tf::Matrix3x3(marker_quaternion).getRPY(roll,pitch,yaw);
          roll  = 0;
          pitch = 0;
          marker_quaternion.setRPY(pitch,roll,yaw);
        }
        markers_[marker_array_id].transform_to_previous.setRotation(marker_quaternion);

        marker_position = markers_[marker_array_id].transform_to_previous.getOrigin();

        // If all markers are in the plane, for better accuracy, we known that Z is zero
        if(space_type_=="plane")
          marker_position.setZ(0);

        markers_[marker_array_id].transform_to_previous.setOrigin(marker_position);

        marker_position = markers_[marker_array_id].transform_to_previous.getOrigin();
        markers_[marker_array_id].pose_to_previous.position.x = marker_position.getX();
        markers_[marker_array_id].pose_to_previous.position.y = marker_position.getY();
        markers_[marker_array_id].pose_to_previous.position.z = marker_position.getZ();

        marker_quaternion = markers_[marker_array_id].transform_to_previous.getRotation();
        markers_[marker_array_id].pose_to_previous.orientation.x = marker_quaternion.getX();
        markers_[marker_array_id].pose_to_previous.orientation.y = marker_quaternion.getY();
        markers_[marker_array_id].pose_to_previous.orientation.z = marker_quaternion.getZ();
        markers_[marker_array_id].pose_to_previous.orientation.w = marker_quaternion.getW();

        // increasing count of markers
        marker_counter_++;

        //--------------------------------------
        // Position of new marker have to be inversed, because i need for calculating of a next new marker position inverse TF of old markers
        //--------------------------------------
        markers_[marker_array_id].current_camera_tf = markers_[marker_array_id].current_camera_tf.inverse();

        marker_position = markers_[marker_array_id].current_camera_tf.getOrigin();
        markers_[marker_array_id].current_camera_pose.position.x = marker_position.getX();
        markers_[marker_array_id].current_camera_pose.position.y = marker_position.getY();
        markers_[marker_array_id].current_camera_pose.position.z = marker_position.getZ();

        marker_quaternion = markers_[marker_array_id].current_camera_tf.getRotation();
        markers_[marker_array_id].current_camera_pose.orientation.x = marker_quaternion.getX();
        markers_[marker_array_id].current_camera_pose.orientation.y = marker_quaternion.getY();
        markers_[marker_array_id].current_camera_pose.orientation.z = marker_quaternion.getZ();
        markers_[marker_array_id].current_camera_pose.orientation.w = marker_quaternion.getW();

        // Publishing of all TFs and markers
        publishTfs(false);
      }
  }

  //------------------------------------------------------
  // New Marker was found, its global position is calculated
  //------------------------------------------------------
  if((marker_counter_previous_ < marker_counter_) && (first_marker_detected_ == true))
  {
    // Publish all TF for five times for listener
    for(size_t j = 0; j < 5; j++)
       publishTfs(false);

    std::stringstream marker_globe;
    marker_globe << "marker_" << marker_array_id;

    listener_->waitForTransform("world",marker_globe.str(),ros::Time(0),ros::Duration(2.0));
    try
    {
      listener_->lookupTransform("world",marker_globe.str(),ros::Time(0),markers_[marker_array_id].transform_to_globe);
    }

    catch(tf::TransformException &e)
    {
      ROS_ERROR("Not able to lookup transform: ");
      ros::Duration(2.0).sleep();
    }

    // Saving TF to Pose
    const tf::Vector3 marker_position = markers_[marker_array_id].transform_to_globe.getOrigin();
    markers_[marker_array_id].pose_to_globe.position.x = marker_position.getX();
    markers_[marker_array_id].pose_to_globe.position.y = marker_position.getY();
    markers_[marker_array_id].pose_to_globe.position.z = marker_position.getZ();

    tf::Quaternion marker_quaternion = markers_[marker_array_id].transform_to_globe.getRotation();
    markers_[marker_array_id].pose_to_globe.orientation.x = marker_quaternion.getX();
    markers_[marker_array_id].pose_to_globe.orientation.y = marker_quaternion.getY();
    markers_[marker_array_id].pose_to_globe.orientation.z = marker_quaternion.getZ();
    markers_[marker_array_id].pose_to_globe.orientation.w = marker_quaternion.getW();
  }
}
  //------------------------------------------------------
  // Calculating the shortest camera marker distance
  // Camera with the shortest distance is used as reference of global position of object (camera)
  //------------------------------------------------------

  bool any_markers_visible = false;
  int num_of_visible_markers = 0;

  if(first_marker_detected_==true)
  {
    double min_size = 999999;
    for(size_t j = 0; j < num_of_markers_; j++)
    {
      double a;
      double b;
      double c;
      double size;

      // If marker is active, distance is calculated
      if(markers_[j].active==true)
      {
        a = markers_[j].current_camera_pose.position.x;
        b = markers_[j].current_camera_pose.position.y;
        c = markers_[j].current_camera_pose.position.z;
        size =std::sqrt((a*a)+(b*b)+(c*c));

        if(size < min_size)
        {
          min_size = size;
          closest_camera_index_=j;
        }

        any_markers_visible = true;
        num_of_visible_markers++;
      }
    }
  }

  // Publish all known markers
  if(first_marker_detected_ == true)
    publishTfs(true);

  //------------------------------------------------------
  // Calculating TF od closer camera
  // Camera with the shortest distance is used as reference of global position of object (camera)
  //------------------------------------------------------
  if((first_marker_detected_ == true) && (any_markers_visible == true))
  {
    std::stringstream closest_camera;
    closest_camera << "camera_" << closest_camera_index_;

    listener_->waitForTransform("world",closest_camera.str(),ros::Time(0),ros::Duration(2.0));
    try
    {
      listener_->lookupTransform("world",closest_camera.str(),ros::Time(0), world_position_transform_);
    }
    catch(tf::TransformException &e)
    {
      ROS_ERROR("Not able to lookup transform:");
    }

    // Save TF to Pose
    const tf::Vector3 marker_position = world_position_transform_.getOrigin();

    world_position_geometry_msg_.position.x = marker_position.getX();
    world_position_geometry_msg_.position.y = marker_position.getY();
    world_position_geometry_msg_.position.z = marker_position.getZ();

    tf::Quaternion marker_quaternion = world_position_transform_.getRotation();
    world_position_geometry_msg_.orientation.x = marker_quaternion.getX();
    world_position_geometry_msg_.orientation.y = marker_quaternion.getY();
    world_position_geometry_msg_.orientation.z = marker_quaternion.getZ();
    world_position_geometry_msg_.orientation.w = marker_quaternion.getW();
  }

  // Publish all known markers
  if(first_marker_detected_ == true)
    publishTfs(true);

  // Publisging ArUcoMarkersPose message
  if((any_markers_visible == true))
  {
    ArUcoMarkersMsgs.header.stamp = ros::Time::now();
    ArUcoMarkersMsgs.header.frame_id = "world";
    ArUcoMarkersMsgs.numberOfMarkers = num_of_visible_markers;
    ArUcoMarkersMsgs.visibility = true;
    ArUcoMarkersMsgs.globalPose = world_position_geometry_msg_;
    ArUcoMarkersMsgs.markersID.clear();
    ArUcoMarkersMsgs.markersPose.clear();
    ArUcoMarkersMsgs.cameraPose.clear();
    for(int j = 0; j < marker_counter_; j++)
    {
      if(markers_[j].active == true)
      {
        ArUcoMarkersMsgs.markersID.push_back(markers_[j].marker_id);
        ArUcoMarkersMsgs.markersPose.push_back(markers_[j].pose_to_globe);
        ArUcoMarkersMsgs.cameraPose.push_back(markers_[j].current_camera_pose);
      }
    }
  }
  else
  {
     ArUcoMarkersMsgs.header.stamp = ros::Time::now();
     ArUcoMarkersMsgs.header.frame_id = "world";
     ArUcoMarkersMsgs.numberOfMarkers = num_of_visible_markers;
     ArUcoMarkersMsgs.visibility = false;
     ArUcoMarkersMsgs.markersID.clear();
     ArUcoMarkersMsgs.markersPose.clear();
  }

  // Publish
  marker_msg_pub_.publish(ArUcoMarkersMsgs);

  return true;
}





void
Estimator::publishTfs(bool world_option)
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
    publishMarker(markers_[i].pose_to_globe,markers_[i].marker_id,i);
  }

  // Global Position of camera
  if(world_option==true)
    broadcaster_.sendTransform(tf::StampedTransform(world_position_transform_,ros::Time::now(),"world","camera_position"));
}





void
Estimator::publishMarker(geometry_msgs::Pose marker_pose, int marker_id, int index)
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
    rviz_marker.scale.z = 0.01;

    rviz_marker.color.r = 1.0f;
    rviz_marker.color.g = 1.0f;
    rviz_marker.color.b = 1.0f;
    rviz_marker.color.a = 1.0f;

    rviz_marker.lifetime = ros::Duration(0.2);

    marker_visualization_pub_.publish(rviz_marker);
}





tf::Transform
Estimator::arucoMarker2Tf(const aruco::Marker &marker)
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





}  //aruco_mapping

#endif  //ESTIMATOR_CPP
