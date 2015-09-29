/*********************************************************************************************//**
* @file estimator.h
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

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

// Standard ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>

// Standarc C++ libraries
#include <iostream>
#include <fstream>
#include <cmath>

// Aruco libraries
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/arucofidmarkers.h>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom message
#include <aruco_mapping/ArUcoMarkers.h>

/** \brief Aruco mapping namespace */
namespace aruco_mapping
{

/** \brief Client class for Aruco mapping */  
class Estimator
{
public:
  
  typedef struct MarkerInfo
  {
    bool active;                                     // Sign of marker acitivity (visibility)
    int marker_id;                                   // Marker ID
    int previous_marker_id;                          // Used for chaining markers
    geometry_msgs::Pose pose_to_previous;            // Position of marker to another marker
    geometry_msgs::Pose pose_to_globe;               // Position of marker to WORLD [0,0,0]
    tf::StampedTransform transform_to_previous;      // Transformation of Marker to another marker
    tf::StampedTransform transform_to_globe;         // Transformation of marker to WORLD [0,0,0]
    geometry_msgs::Pose current_camera_pose;         // Actual position of camera over marker
    tf::Transform current_camera_tf;                 // Actual Transformation of camera over marker
   
   } MarkerInfo;

public:
  
  /** \brief Construct a client for EZN64 USB control*/  
  Estimator(ros::NodeHandle *myNode);
    
  ~Estimator();
    
  void imageCallback(const sensor_msgs::ImageConstPtr &original_image);
  
  inline void waitForStart(const std_msgs::EmptyPtr& message)
  {
    start_now_ = true;
  }

private:
  
  tf::Transform arucoMarker2Tf(const aruco::Marker &marker);
  void publishTfs(bool world_option);
  void publishMarker(geometry_msgs::Pose marker_pose, int marker_id, int index);
  bool loadCalibrationFile(std::string filename);
  bool markersFindPattern(cv::Mat input_image,cv::Mat output_image);
  
  cv::Mat  I_;
  cv::Mat  *intrinsics_;                            
  cv::Mat  *distortion_coeff_;                      
  cv::Size *image_size_;                           
  
  ros::Publisher marker_visualization_pub_;       
  ros::Publisher marker_msg_pub_;                 // publisher of my message
   
  aruco_mapping::ArUcoMarkers ArUcoMarkersMsgs;
  aruco::CameraParameters aruco_calib_params_;     
    
  std::string calib_filename_;                    
  std::string space_type_;                        
    
  float marker_size_;                            
    
  int num_of_markers_;                    
  int lowest_marker_id_;                          
  int marker_counter_;                            
  int marker_counter_previous_;                   // counter of actual markers before image processing
  int closest_camera_index_;                     // actual camera, which is closer to some marker
  
  bool start_now_;
  bool first_marker_detected_;                    
  
  bool roi_allowed_;                              
  int  roi_x_;                                      
  int  roi_y_;                                      
  int  roi_w_;                                     
  int  roi_h_;     
  
  std::vector<MarkerInfo> markers_;              // pole pre poziciu kazdeho markera - markre pevne na zemi
  
  tf::TransformListener *listener_;              // listener for TF
  tf::TransformBroadcaster broadcaster_;         // broadcaster
  tf::StampedTransform world_position_transform_;           // global position to World TF
  
  geometry_msgs::Pose world_position_geometry_msg_;        // global position to World
 
  
};

}  //aruco_mapping

#endif //ESTIMATOR_H
