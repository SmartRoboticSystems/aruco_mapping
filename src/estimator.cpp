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

#ifndef ESTIMATOR_CPP
#define ESTIMATOR_CPP


#include <estimator.h>

namespace aruco_mapping
{

Estimator::Estimator(ros::NodeHandle *myNode) :
    marker_size_(0.1),                      // Marker size in m
    calib_filename_("empty"),               // Initial filenames
    listener_ (new tf::TransformListener), // Listener of TFs
    start_now_(false),
    roi_allowed_ (false),               // switiching ROI
    num_of_markers_to_find_ (35),                // Number of used markers
    space_type_ ("plane")                   // default space - plane
{
     // Parameter - marker size [m]
    
    double temp_marker_size;  
    myNode->getParam("MarkerSize",temp_marker_size);  
    marker_size_ = float(temp_marker_size);
    
    // Path to calibration file of camera
    //--------------------------------------------------
    myNode->getParam("calibration_file",calib_filename_);
    std::cout << "Calibration file path: " << calib_filename_ << std::endl;
    //--------------------------------------------------
    // Parameter - number of all markers
    myNode->getParam("markers_number",num_of_markers_to_find_);
    //--------------------------------------------------
    
    //Start Immediately?
    myNode->getParam("StartImmediately", start_now_);
    
    
    // Parameter - region of interest
    myNode->getParam("region_of_interest",roi_allowed_);
    //--------------------------------------------------
    
    //--------------------------------------------------
    // Parameter - type of space, plane or 3D space
    myNode->getParam("type_of_markers_space",space_type_);
    //--------------------------------------------------
    // Parameter - region of interest - parameters
    //--------------------------------------------------
    roi_x_ = 0;
    roi_y_ = 0;
    roi_w_ = 10;
    roi_h_ = 5;
    
    myNode->getParam("region_of_interest_x",roi_x_);
    myNode->getParam("region_of_interest_y",roi_y_);
    myNode->getParam("region_of_interest_width",roi_w_);
    myNode->getParam("region_of_interest_height",roi_h_);
    //--------------------------------------------------

    // Publishers
    marker_msg_pub_=myNode->advertise<aruco_mapping::ArUcoMarkers>("ArUcoMarkersPose",1);
    marker_visualization_pub_=myNode->advertise<visualization_msgs::Marker>("aruco_marker",1);
    //--------------------------------------------------

    // Loading calibration parameters
    //--------------------------------------------------
    loadCalibrationFile(calib_filename_);
    //--------------------------------------------------

    // Inicialization of marker
    //--------------------------------------------------
    cv::namedWindow("Mono8", CV_WINDOW_AUTOSIZE);
    //--------------------------------------------------


    // Inicialization of variables
    //--------------------------------------------------
    first_marker_detected_=false;
    lowest_marker_id_=-1;
    marker_counter_=0;
    closest_camera_index_=0;
    // Dynamics array of markers
    markers_=new MarkerInfo[num_of_markers_to_find_];

    // Inicialization of array - all informations about each marker
    for(int j=0;j<num_of_markers_to_find_;j++)
    {
        markers_[j].relatedMarkerID=-1;
        markers_[j].active=false;
        markers_[j].markerID=-1;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////

Estimator::~Estimator()
{
    delete intrinsics_;
    delete distortion_coeff_;
    delete image_size_;
    delete listener_;
    delete [] markers_;
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
Estimator::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
    // ROS Image to Mat structure
    //--------------------------------------------------
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Error open image %s", e.what());
        return;
    }
    I_=cv_ptr->image;
    //--------------------------------------------------

    // Region Of Interest
    if(roi_allowed_==true)
        I_=cv_ptr->image(cv::Rect(roi_x_,roi_y_,roi_w_,roi_h_));

    // Marker detection
    //--------------------------------------------------
    if(start_now_ == true)
      bool found=markersFindPattern(I_,I_);
    //--------------------------------------------------

    // Show image
    cv::imshow("Mono8", I_);
    cv::waitKey(10);
}


////////////////////////////////////////////////////////////////////////////////////////////////

bool
Estimator::markersFindPattern(cv::Mat input_image,cv::Mat output_image)
{
    aruco::MarkerDetector MDetector;
    std::vector<aruco::Marker> markers;

    // Initialization, all markers sign of visibility is set to false
    for(int j=0;j<num_of_markers_to_find_;j++)
        markers_[j].active=false;

    // Actual count of found markers before new image processing
    marker_counter_previous_=marker_counter_;

    // Actual ID of marker, which is processed
    int MarrkerArrayID;

    // Markers Detector, if return marker.size() 0, it dint finf any marker in image
    MDetector.detect(input_image,markers,aruco_calib_params_,marker_size_);

    // Any marker wasnt find
    if(markers.size()==0)
        std::cout << "Any marker is not in the actual image!" << std::endl;

    //------------------------------------------------------
    // Initialization of First Marker - begining of the path
    //------------------------------------------------------
    // First marker is detected
    if((markers.size()>0)&&(first_marker_detected_==false))
    {
        size_t low_ID;

        // Marker with lowest ID is my begining of the path
        lowest_marker_id_=markers[0].id;
        low_ID=0;
        for(size_t i=0;i<markers.size();i++)
        {
            if(markers[i].id<lowest_marker_id_)
            {
                lowest_marker_id_=markers[i].id;
                low_ID=i;
            }
        }
        std::cout << "The lowest Id marker " << lowest_marker_id_ << std::endl;

        // Position of my beginning - origin [0,0,0]
        markers_[0].markerID=lowest_marker_id_;
        markers_[0].AllMarkersPose.position.x=0;
        markers_[0].AllMarkersPose.position.y=0;
        markers_[0].AllMarkersPose.position.z=0;
        // And orientation
        markers_[0].AllMarkersPose.orientation.x=0;
        markers_[0].AllMarkersPose.orientation.y=0;
        markers_[0].AllMarkersPose.orientation.z=0;
        markers_[0].AllMarkersPose.orientation.w=1;

        // Relative position and Global position of Origin is same
        markers_[0].AllMarkersPoseGlobe.position.x=0;
        markers_[0].AllMarkersPoseGlobe.position.y=0;
        markers_[0].AllMarkersPoseGlobe.position.z=0;
        markers_[0].AllMarkersPoseGlobe.orientation.x=0;
        markers_[0].AllMarkersPoseGlobe.orientation.y=0;
        markers_[0].AllMarkersPoseGlobe.orientation.z=0;
        markers_[0].AllMarkersPoseGlobe.orientation.w=1;

        // Transformation Pose to TF

        tf::Vector3 origin;
        origin.setX(0);
        origin.setY(0);
        origin.setZ(0);

        tf::Quaternion quat;
        quat.setX(0);
        quat.setY(0);
        quat.setZ(0);
        quat.setW(1);

        markers_[0].AllMarkersTransform.setOrigin(origin);
        markers_[0].AllMarkersTransform.setRotation(quat);
        // Relative position and Global position of first marker - Origin is same
        markers_[0].AllMarkersTransformGlobe=markers_[0].AllMarkersTransform;

        // Increasing count of actual markers
        marker_counter_++;

        // Sign of visibility of first marker
        first_marker_detected_=true;
        std::cout << "First marker [origin] was found" << std::endl;

        // Position of origin is relative to global position, no relative position to any marker
        markers_[0].relatedMarkerID=-2;
        // Sign of visibilitys
        markers_[0].active=true;
    }
    //------------------------------------------------------

    //------------------------------------------------------
    // Addings new markers and mapping
    //------------------------------------------------------
    // Markers are always sorted in ascending
    for(size_t i=0;i<markers.size();i++)
    {
        int currentMarkerID=markers[i].id;

        // Use only marker with modulo 10
        if(currentMarkerID%10==0)
        {
            //------------------------------------------------------
            //Draw marker convex, ID, cube and axis
            //------------------------------------------------------
            markers[i].draw(output_image, cv::Scalar(0,0,255),2);
            aruco::CvDrawingUtils::draw3dCube(output_image,markers[i],aruco_calib_params_);
            aruco::CvDrawingUtils::draw3dAxis(output_image,markers[i],aruco_calib_params_);

            //------------------------------------------------------
            // Check, if it see new marker or it has already known
            //------------------------------------------------------
            bool find=false;
            int m_j=0;
            while((find==false)&&(m_j<marker_counter_))
            {
                if(markers_[m_j].markerID==currentMarkerID)
                {
                    MarrkerArrayID=m_j;
                    find=true;
                    std::cout << "Existing ID was assigned" << std::endl;
                }
                m_j++;
            }
            if(find==false)
            {
                MarrkerArrayID=marker_counter_;
                markers_[MarrkerArrayID].markerID=currentMarkerID;
                find=true;
                std::cout << "New marker" << std::endl;
            }

            //------------------------------------------------------
            std::cout << "Actuak marker " << MarrkerArrayID << " and its ID " << markers_[MarrkerArrayID].markerID << std::endl;
            //------------------------------------------------------

            //------------------------------------------------------
            // Sign of visivility of marker
            // If new marker is found, sign of visibility must be changed
            //------------------------------------------------------
            for(int j=0;j<marker_counter_;j++)
            {
                for(size_t im=0;im<markers.size();im++)
                {
                    if(markers_[j].markerID==markers[im].id)
                        markers_[j].active=true;
                }
            }
            //------------------------------------------------------

            //------------------------------------------------------
            // Old marker was found in the image
            //------------------------------------------------------
            if((MarrkerArrayID<marker_counter_)&&(first_marker_detected_==true))
            {
                markers_[MarrkerArrayID].CurrentCameraTf=arucoMarker2Tf(markers[i]);
                markers_[MarrkerArrayID].CurrentCameraTf=markers_[MarrkerArrayID].CurrentCameraTf.inverse();

                const tf::Vector3 marker_origin=markers_[MarrkerArrayID].CurrentCameraTf.getOrigin();
                markers_[MarrkerArrayID].CurrentCameraPose.position.x=marker_origin.getX();
                markers_[MarrkerArrayID].CurrentCameraPose.position.y=marker_origin.getY();
                markers_[MarrkerArrayID].CurrentCameraPose.position.z=marker_origin.getZ();

                const tf::Quaternion marker_quaternion=markers_[MarrkerArrayID].CurrentCameraTf.getRotation();
                markers_[MarrkerArrayID].CurrentCameraPose.orientation.x=marker_quaternion.getX();
                markers_[MarrkerArrayID].CurrentCameraPose.orientation.y=marker_quaternion.getY();
                markers_[MarrkerArrayID].CurrentCameraPose.orientation.z=marker_quaternion.getZ();
                markers_[MarrkerArrayID].CurrentCameraPose.orientation.w=marker_quaternion.getW();
            }

            //------------------------------------------------------
            // New marker was found
            // Global and relative position must be calculated
            //------------------------------------------------------
            if((MarrkerArrayID==marker_counter_)&&(first_marker_detected_==true))
            {
                markers_[MarrkerArrayID].CurrentCameraTf=arucoMarker2Tf(markers[i]);

                tf::Vector3 marker_origin=markers_[MarrkerArrayID].CurrentCameraTf.getOrigin();
                markers_[MarrkerArrayID].CurrentCameraPose.position.x=marker_origin.getX();
                markers_[MarrkerArrayID].CurrentCameraPose.position.y=marker_origin.getY();
                markers_[MarrkerArrayID].CurrentCameraPose.position.z=marker_origin.getZ();

                tf::Quaternion marker_quaternion=markers_[MarrkerArrayID].CurrentCameraTf.getRotation();
                markers_[MarrkerArrayID].CurrentCameraPose.orientation.x=marker_quaternion.getX();
                markers_[MarrkerArrayID].CurrentCameraPose.orientation.y=marker_quaternion.getY();
                markers_[MarrkerArrayID].CurrentCameraPose.orientation.z=marker_quaternion.getZ();
                markers_[MarrkerArrayID].CurrentCameraPose.orientation.w=marker_quaternion.getW();

                // Naming - TFs
                std::stringstream cameraTFID;
                cameraTFID << "camera_" << MarrkerArrayID;
                std::stringstream cameraTFID_old;
                std::stringstream markerTFID_old;

                // Sing if any known marker is visible
                bool anyMarker=false;
                // Array ID of markers, which position of new marker is calculated
                int lastMarlerID;

                // Testing, if is possible calculate position of a new marker to old known marker
                for(int k=0;k<MarrkerArrayID;k++)
                {
                    if((markers_[k].active==true)&&(anyMarker==false))
                    {
                        if(markers_[k].relatedMarkerID!=-1)
                        {
                            anyMarker=true;
                            cameraTFID_old << "camera_" << k;
                            markerTFID_old << "marker_" << k;
                            markers_[MarrkerArrayID].relatedMarkerID=k;
                            lastMarlerID=k;
                        }
                    }
                }

                // New position can be calculated
                if(anyMarker==true)
                {
                    // Generating TFs for listener
                    for(char k=0;k<10;k++)
                    {
                        // TF from old marker and its camera
                        broadcaster_.sendTransform(tf::StampedTransform(markers_[lastMarlerID].CurrentCameraTf,ros::Time::now(),markerTFID_old.str(),cameraTFID_old.str()));
                          // TF from old camera to new camera
                        broadcaster_.sendTransform(tf::StampedTransform(markers_[MarrkerArrayID].CurrentCameraTf,ros::Time::now(),cameraTFID_old.str(),cameraTFID.str()));
                        usleep(100);
                    }

                    // Calculate TF between two markers
                    listener_->waitForTransform(markerTFID_old.str(),cameraTFID.str(),ros::Time(0),ros::Duration(2.0));
                    try
                    {
                        broadcaster_.sendTransform(tf::StampedTransform(markers_[lastMarlerID].CurrentCameraTf,ros::Time::now(),markerTFID_old.str(),cameraTFID_old.str()));
                        broadcaster_.sendTransform(tf::StampedTransform(markers_[MarrkerArrayID].CurrentCameraTf,ros::Time::now(),cameraTFID_old.str(),cameraTFID.str()));

                        listener_->lookupTransform(markerTFID_old.str(),cameraTFID.str(),ros::Time(0),markers_[MarrkerArrayID].AllMarkersTransform);
                    }
                    catch(tf::TransformException &ex)
                    {
                        ROS_ERROR("Error - 1\n");
                        ros::Duration(2.0).sleep();
                    }

                    // Saving quaternion and origin of calculated TF to pose
                    marker_quaternion=markers_[MarrkerArrayID].AllMarkersTransform.getRotation();
                    // If all markers are in the plane, for better accuracy, we known that roll and pitch are zero
                    if(space_type_=="plane")
                    {
                        double roll,pitch,yaw;
                        tf::Matrix3x3(marker_quaternion).getRPY(roll,pitch,yaw);
                        roll=0;
                        pitch=0;
                        marker_quaternion.setRPY(pitch,roll,yaw);
                    }
                    markers_[MarrkerArrayID].AllMarkersTransform.setRotation(marker_quaternion);

                    marker_origin=markers_[MarrkerArrayID].AllMarkersTransform.getOrigin();
                    // If all markers are in the plane, for better accuracy, we known that Z is zero
                    if(space_type_=="plane")
                        marker_origin.setZ(0);
                    markers_[MarrkerArrayID].AllMarkersTransform.setOrigin(marker_origin);

                    marker_origin=markers_[MarrkerArrayID].AllMarkersTransform.getOrigin();
                    markers_[MarrkerArrayID].AllMarkersPose.position.x=marker_origin.getX();
                    markers_[MarrkerArrayID].AllMarkersPose.position.y=marker_origin.getY();
                    markers_[MarrkerArrayID].AllMarkersPose.position.z=marker_origin.getZ();

                    marker_quaternion=markers_[MarrkerArrayID].AllMarkersTransform.getRotation();
                    markers_[MarrkerArrayID].AllMarkersPose.orientation.x=marker_quaternion.getX();
                    markers_[MarrkerArrayID].AllMarkersPose.orientation.y=marker_quaternion.getY();
                    markers_[MarrkerArrayID].AllMarkersPose.orientation.z=marker_quaternion.getZ();
                    markers_[MarrkerArrayID].AllMarkersPose.orientation.w=marker_quaternion.getW();

                    // increasing count of markers
                    marker_counter_++;

                    //--------------------------------------
                    // Position of new marker have to be inversed, because i need for calculating of a next new marker position inverse TF of old markers
                    //--------------------------------------
                    markers_[MarrkerArrayID].CurrentCameraTf=markers_[MarrkerArrayID].CurrentCameraTf.inverse();
                    marker_origin=markers_[MarrkerArrayID].CurrentCameraTf.getOrigin();
                    markers_[MarrkerArrayID].CurrentCameraPose.position.x=marker_origin.getX();
                    markers_[MarrkerArrayID].CurrentCameraPose.position.y=marker_origin.getY();
                    markers_[MarrkerArrayID].CurrentCameraPose.position.z=marker_origin.getZ();
                    marker_quaternion=markers_[MarrkerArrayID].CurrentCameraTf.getRotation();
                    markers_[MarrkerArrayID].CurrentCameraPose.orientation.x=marker_quaternion.getX();
                    markers_[MarrkerArrayID].CurrentCameraPose.orientation.y=marker_quaternion.getY();
                    markers_[MarrkerArrayID].CurrentCameraPose.orientation.z=marker_quaternion.getZ();
                    markers_[MarrkerArrayID].CurrentCameraPose.orientation.w=marker_quaternion.getW();

                    // Publishing of all TFs and markers
                    publishTfs(false);
                }
            }
        }


        //------------------------------------------------------
        // New Marker was found, its global position is calculated
        //------------------------------------------------------
        if((marker_counter_previous_<marker_counter_)&&(first_marker_detected_==true))
        {
            // Publishing all TF for five times for listener
            for(char k=0;k<5;k++)
                publishTfs(false);

            std::stringstream markerGlobe;
            markerGlobe << "marker_" << MarrkerArrayID;

            listener_->waitForTransform("world",markerGlobe.str(),ros::Time(0),ros::Duration(2.0));
            try
            {
                listener_->lookupTransform("world",markerGlobe.str(),ros::Time(0),markers_[MarrkerArrayID].AllMarkersTransformGlobe);
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("Error - 2 Global position of marker\n");
                ros::Duration(2.0).sleep();
            }

            // Saving TF to Pose
            const tf::Vector3 marker_origin=markers_[MarrkerArrayID].AllMarkersTransformGlobe.getOrigin();
            markers_[MarrkerArrayID].AllMarkersPoseGlobe.position.x=marker_origin.getX();
            markers_[MarrkerArrayID].AllMarkersPoseGlobe.position.y=marker_origin.getY();
            markers_[MarrkerArrayID].AllMarkersPoseGlobe.position.z=marker_origin.getZ();
            tf::Quaternion marker_quaternion=markers_[MarrkerArrayID].AllMarkersTransformGlobe.getRotation();
            markers_[MarrkerArrayID].AllMarkersPoseGlobe.orientation.x=marker_quaternion.getX();
            markers_[MarrkerArrayID].AllMarkersPoseGlobe.orientation.y=marker_quaternion.getY();
            markers_[MarrkerArrayID].AllMarkersPoseGlobe.orientation.z=marker_quaternion.getZ();
            markers_[MarrkerArrayID].AllMarkersPoseGlobe.orientation.w=marker_quaternion.getW();
        }
    }
    //------------------------------------------------------


    //------------------------------------------------------
    // Calculating ot the shortest camera distance to some marker
    // Camera with the shortest distance is used as reference of global position of object (camera)
    //------------------------------------------------------

    bool someMarkersAreVisible=false;
    int numberOfVisibleMarkers=0;

    if(first_marker_detected_==true)
    {
        double minSize=999999;
        for(int k=0;k<num_of_markers_to_find_;k++)
        {
            double a;
            double b;
            double c;
            double size;
            // If marker is active, distance is calculated
            if(markers_[k].active==true)
            {
                a=markers_[k].CurrentCameraPose.position.x;
                b=markers_[k].CurrentCameraPose.position.y;
                c=markers_[k].CurrentCameraPose.position.z;
                size=std::sqrt((a*a)+(b*b)+(c*c));
                if(size<minSize)
                {
                    minSize=size;
                    closest_camera_index_=k;
                }

                someMarkersAreVisible=true;
                numberOfVisibleMarkers++;
            }
        }
    }
    //------------------------------------------------------

    //------------------------------------------------------
    // Publish all known markers
    //------------------------------------------------------
    if(first_marker_detected_==true)
        publishTfs(true);
    //------------------------------------------------------

    //------------------------------------------------------
    // Calculating TF od closer camera
    // Camera with the shortest distance is used as reference of global position of object (camera)
    //------------------------------------------------------
    //---
    if((first_marker_detected_==true)&&(someMarkersAreVisible==true))
    {
        std::stringstream closestCamera;
        closestCamera << "camera_" << closest_camera_index_;

        listener_->waitForTransform("world",closestCamera.str(),ros::Time(0),ros::Duration(2.0));
        try
        {
            listener_->lookupTransform("world",closestCamera.str(),ros::Time(0),world_position_transform_);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("Error - 3\n");
        }

        // Saving TF to Pose
        const tf::Vector3 marker_origin=world_position_transform_.getOrigin();
        world_position_geometry_msg_.position.x=marker_origin.getX();
        world_position_geometry_msg_.position.y=marker_origin.getY();
        world_position_geometry_msg_.position.z=marker_origin.getZ();
        tf::Quaternion marker_quaternion=world_position_transform_.getRotation();
        world_position_geometry_msg_.orientation.x=marker_quaternion.getX();
        world_position_geometry_msg_.orientation.y=marker_quaternion.getY();
        world_position_geometry_msg_.orientation.z=marker_quaternion.getZ();
        world_position_geometry_msg_.orientation.w=marker_quaternion.getW();
    }
    //------------------------------------------------------

    //------------------------------------------------------
    // Publish all known markers
    //------------------------------------------------------
    if(first_marker_detected_==true)
        publishTfs(true);
    //------------------------------------------------------

    //------------------------------------------------------
    // Publisging ArUcoMarkersPose message
    //------------------------------------------------------
    if((someMarkersAreVisible==true))
    {
        ArUcoMarkersMsgs.header.stamp=ros::Time::now();
        ArUcoMarkersMsgs.header.frame_id="world";
        ArUcoMarkersMsgs.numberOfMarkers=numberOfVisibleMarkers;
        ArUcoMarkersMsgs.visibility=true;
        ArUcoMarkersMsgs.globalPose=world_position_geometry_msg_;
        ArUcoMarkersMsgs.markersID.clear();
        ArUcoMarkersMsgs.markersPose.clear();
        ArUcoMarkersMsgs.cameraPose.clear();
        for(int j=0;j<marker_counter_;j++)
        {
            if(markers_[j].active==true)
            {
                ArUcoMarkersMsgs.markersID.push_back(markers_[j].markerID);
                ArUcoMarkersMsgs.markersPose.push_back(markers_[j].AllMarkersPoseGlobe);
                ArUcoMarkersMsgs.cameraPose.push_back(markers_[j].CurrentCameraPose);
            }
        }
    }
    else
    {
        ArUcoMarkersMsgs.header.stamp=ros::Time::now();
        ArUcoMarkersMsgs.header.frame_id="world";
        ArUcoMarkersMsgs.numberOfMarkers=numberOfVisibleMarkers;
        ArUcoMarkersMsgs.visibility=false;
        ArUcoMarkersMsgs.markersID.clear();
        ArUcoMarkersMsgs.markersPose.clear();
    }
    //------------------------------------------------------

    // Publish
    marker_msg_pub_.publish(ArUcoMarkersMsgs);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
Estimator::publishTfs(bool world_option)
{
    for(int j=0;j<marker_counter_;j++)
    {
        // Actual Marker
        std::stringstream markerTFID;
        markerTFID << "marker_" << j;
        // Older marker - or World
        std::stringstream markerTFID_old;
        if(j==0)
            markerTFID_old << "world";
        else
            markerTFID_old << "marker_" << markers_[j].relatedMarkerID;
        broadcaster_.sendTransform(tf::StampedTransform(markers_[j].AllMarkersTransform,ros::Time::now(),markerTFID_old.str(),markerTFID.str()));

        // Position of camera to its marker
        std::stringstream cameraTFID;
        cameraTFID << "camera_" << j;
        broadcaster_.sendTransform(tf::StampedTransform(markers_[j].CurrentCameraTf,ros::Time::now(),markerTFID.str(),cameraTFID.str()));

        if(world_option==true)
        {
            // Global position of marker TF
            std::stringstream markerGlobe;
            markerGlobe << "marker_globe_" << j;
            broadcaster_.sendTransform(tf::StampedTransform(markers_[j].AllMarkersTransformGlobe,ros::Time::now(),"world",markerGlobe.str()));
        }

        // Cubes for RVIZ - markers
        publishMarker(markers_[j].AllMarkersPose,markers_[j].markerID,j);
    }

    // Global Position of object
    if(world_option==true)
        broadcaster_.sendTransform(tf::StampedTransform(world_position_transform_,ros::Time::now(),"world","myGlobalPosition"));
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
Estimator::publishMarker(geometry_msgs::Pose markerPose, int MarkerID, int rank)
{
    visualization_msgs::Marker myMarker;

    if(rank==0)
        myMarker.header.frame_id="world";
    else
    {
        std::stringstream markerTFID_old;
        markerTFID_old << "marker_" << markers_[rank].relatedMarkerID;
        myMarker.header.frame_id=markerTFID_old.str();
    }

    myMarker.header.stamp=ros::Time::now();
    myMarker.ns="basic_shapes";
    myMarker.id=MarkerID;
    myMarker.type=visualization_msgs::Marker::CUBE;
    myMarker.action=visualization_msgs::Marker::ADD;

    myMarker.pose=markerPose;
    myMarker.scale.x=marker_size_;
    myMarker.scale.y=marker_size_;
    myMarker.scale.z=0.01;

    myMarker.color.r=1.0f;
    myMarker.color.g=1.0f;
    myMarker.color.b=1.0f;
    myMarker.color.a=1.0f;

    myMarker.lifetime=ros::Duration(0.2);

    marker_visualization_pub_.publish(myMarker);
}

////////////////////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

bool
Estimator::loadCalibrationFile(std::string calib_filename_)
{
    std::cout << "Reading calibration file from: " << calib_filename_ << std::endl;
    try
    {
        //Searching camera matrix and distortion in calibration textfile
        //# oST version 5.0 parameters
        string camera_matrix_str("camera matrix");
        string distortion_str("distortion");

        // Object of reading file
        ifstream file;
        file.open(calib_filename_.c_str());

        // Alocation of memory
        intrinsics_=new(cv::Mat)(3,3,CV_64F);
        distortion_coeff_=new(cv::Mat)(5,1,CV_64F);
        image_size_=new(cv::Size);

        //  Reading of calibration file lines
        std::string line;
        int line_counter = 0;
        while(getline(file, line))
        {
            // Camera matrix 3x3
            if(line==camera_matrix_str)
            {
                for(size_t i=0;i<3;i++)
                    for(size_t j=0;j<3;j++)
                        file >> intrinsics_->at<double>(i,j);
                std::cout << "Intrinsics:" << std::endl << *intrinsics_ << std::endl;
            }
            // Distortion 5x1
            if(line==distortion_str)
            {
                for(size_t i=0; i<5;i++)
                file >> distortion_coeff_->at<double>(i,0);
                std::cout << "Distortion: " << *distortion_coeff_ << std::endl;
            }
            line_counter++;
        }
        aruco_calib_params_.setParams(*intrinsics_, *distortion_coeff_, *image_size_);
        if ((intrinsics_->at<double>(2,2)==1)&&(distortion_coeff_->at<double>(0,4)==0))
            ROS_INFO_STREAM("Calibration file loaded successfully");
        else
            ROS_WARN("WARNING: Suspicious calibration data");
    }
    catch(int e)
    {
        std::cout << "An exception n." << e << "occured";
    }
}

}  //aruco_mapping

#endif  //ESTIMATOR_CPP