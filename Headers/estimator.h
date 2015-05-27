/*********************************************************************************************//**
* @file estimator.h
*
* ArUco Positioning System header file
*
* Copyright (c)
* Jan Bacik
* Smart Robotic Systems
* www.smartroboticsys.eu
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

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

////////////////////////////////////////////////////////////////////////////////////////////////

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

// my message
#include "aruco_positioning_system//ArUcoMarkers.h"

////////////////////////////////////////////////////////////////////////////////////////////////


class ViewPoint_Estimator
{
public:
    enum Pattern {NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, MARKERS};

    typedef struct MarkerInfo
    {
            // Marker ID
            int markerID;
            // Position of marker to another marker
            geometry_msgs::Pose AllMarkersPose;
            // Transformation of Marker to another marker
            tf::StampedTransform AllMarkersTransform;
            // Transformation of marker to WORLD [0,0,0]
            tf::StampedTransform AllMarkersTransformGlobe;
            // Position of marker to WORLD [0,0,0]
            geometry_msgs::Pose AllMarkersPoseGlobe;
            // Actual position of camera over marker
            geometry_msgs::Pose CurrentCameraPose;
            // Actuak Transformation of camera over marker
            tf::Transform CurrentCameraTf;
            // Sign of acitivity (visibility) of marker
            bool active;
            // Realated marker ID, (define ID of which marker is relative to actual marker)
            int relatedMarkerID;

    } MarkerInfo;

public:
    explicit ViewPoint_Estimator(ros::NodeHandle *myNode, float paramMakerSize);
    ~ViewPoint_Estimator();
    tf::Transform arucoMarker2Tf(const aruco::Marker &marker);
    void image_callback(const sensor_msgs::ImageConstPtr &original_image);
    void publish_tfs(bool world_option);
    void publish_marker(geometry_msgs::Pose markerPose, int MarkerID, int rank);
    bool load_calibration_file(std::string filename);
    bool markers_find_pattern(cv::Mat input_image,cv::Mat output_image);

    inline void wait_for_start(const std_msgs::EmptyPtr& message)
    {
        StartNow=true;
    }

private:
    cv::Mat I;
    ros::Publisher my_markers_pub;                  // publisher of my message
    aruco_positioning_system::ArUcoMarkers ArUcoMarkersMsgs;
    ros::Publisher pose3D_pub;                      // 3D pose publisher
    ros::Publisher pose_3D_array;                   // 3D pose array
    ros::Publisher marker_pub;                      // marker visualization
    std::string filename;                           // calibration file path
    std::string type_of_space;                      // plane or 3D space
    cv::Mat *intrinsics;                            // camera intrinsics
    cv::Mat *distortion_coeff;                      // camera distortion coeffs
    cv::Size *image_size;                           // image_size
    Pattern calibration_pattern;                    // type of calibration pattern
    float markerSize;                               // marker geometry
    aruco::CameraParameters arucoCalibParams;       // camera parameters for aruco lib
    int numberOfAllMarkers;                         // size of dynamical array
    bool regionOfInterest;                          // ROI allow
    int ROIx;                                       // ROI X
    int ROIy;                                       // ROI Y
    int ROIw;                                       // ROI WIDTH
    int ROIh;                                       // ROI HEIGHT
    bool lookingForFirst;                           // control, if the first marker is already found
    int lowestIDMarker;                             // ID of the first marker
    MarkerInfo *AllMarkers;                         // pole pre poziciu kazdeho markera - markre pevne na zemi
    int markersCounter;                             // counter of actuals markers
    int markerCounter_before;                       // counter of actual markers before image processing
    tf::TransformListener *myListener;              // listener for TF
    tf::TransformBroadcaster myBroadcaster;         // broadcaster
    int indexActualCamera;                          // actual camera, which is closer to some marker
    tf::StampedTransform myWorldPosition;           // global position to World TF
    geometry_msgs::Pose myWorldPositionPose;        // global position to World
    bool StartNow;                                  // information about start image processing after start of program
    bool StartNowFromParameter;                     // or start after revieving starting message
};

////////////////////////////////////////////////////////////////////////////////////////////////

#endif //ESTIMATOR_H
