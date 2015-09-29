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
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Standard C++ libraries
#include <fstream>

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
#include <aruco_mapping/ArucoMarker.h>


/** \brief Aruco mapping namespace */
namespace aruco_mapping
{

/** \brief Client class for Aruco mapping */  
class Estimator
{
public:
  
  /** \brief Struct to keep marker information */
  struct MarkerInfo
  {
    bool active;                                     // Flag of marker visibility (active)
    int marker_id;                                   // Marker ID
    int previous_marker_id;                          // Used for chaining markers
    geometry_msgs::Pose pose_to_previous;            // Position with respect to previous marker
    geometry_msgs::Pose pose_to_globe;               // Position with respect to world's origin
    tf::StampedTransform transform_to_previous;      // TF with respect to previous marker
    tf::StampedTransform transform_to_globe;         // TF with respect to world's origin
    geometry_msgs::Pose current_camera_pose;         // Position of camera with respect to the marker
    tf::Transform current_camera_tf;                 // TF of camera with respect to the marker
   };

public:
  
  /** \brief Construct client for Aruco mapping*/
  Estimator(ros::NodeHandle *nh);
    
  ~Estimator();

  /** \brief Callback function to handle image processing*/
  void imageCallback(const sensor_msgs::ImageConstPtr &original_image);

private:
  
  /** \brief Function to parse data from calibration file*/
  bool loadCalibrationFile(std::string filename);

  /** \brief Function to process input image, detect markers and its poses*/
  bool processImage(cv::Mat input_image,cv::Mat output_image);

  /** \brief Function to publish all known TFs*/
  void publishTfs(bool world_option);

  /** \brief Function to publish all known markers for visualization purposes*/
  void publishMarker(geometry_msgs::Pose marker_pose, int marker_id, int index);

  /** \brief Compute TF from marker detector result*/
  tf::Transform arucoMarker2Tf(const aruco::Marker &marker);

  /** \brief Publisher of visualization_msgs::Marker message*/
  ros::Publisher marker_visualization_pub_;

  /** \brief Publisher of aruco_mapping::ArucoMarker custom message*/
  ros::Publisher marker_msg_pub_;

  //Launch file parameters
  std::string calib_filepath_;
  std::string space_type_;                        
  float marker_size_;
  int num_of_markers_;                    
  bool roi_allowed_;
  int  roi_x_;                                      
  int  roi_y_;                                      
  int  roi_w_;                                     
  int  roi_h_;

  /** \brief Container holding MarkerInfo data about all detected markers */
  std::vector<MarkerInfo> markers_;

  /** \brief Actual TF of camera with respect to world's origin */
  tf::StampedTransform world_position_transform_;

  /** \brief Actual Pose of camera with respect to world's origin */
  geometry_msgs::Pose world_position_geometry_msg_;

  aruco::CameraParameters aruco_calib_params_;

  int marker_counter_;
  int marker_counter_previous_;
  int lowest_marker_id_;

  bool first_marker_detected_;

  tf::TransformListener *listener_;
  tf::TransformBroadcaster broadcaster_;
   
};

}  //aruco_mapping

#endif //ESTIMATOR_H
