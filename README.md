# aruco_positioning_system

[ROS](http://ros.org) package

Positioning system based on ArUco markers

* Video - 3D space: https://www.youtube.com/watch?v=Hv1Bs1w5HlU
* Video - 2D space: https://www.youtube.com/watch?v=IBsmotN35Us
* ROS wiki page: 
* Autors: [Jan Bacik] (http://www.smartroboticsys.eu/?page_id=895&lang=en), [Smart Robotic Systems] (http://www.smartroboticsys.eu)

## Topics:

#### Input Image: 

/camera/image_raw

#### Input Topics:

/arucoPositioningSystem/startArUco

* EmptyMessage for starting mapping

/ArUcoMarkersPose

Variable   | Comment|
-----------|-----------|
header     | header of message|
visibility | sign of visibility of any marker |
globalPose| my global position|
numberOfMarkers| number of visible markers|
markersID| array of visible markers IDs |
markersPose| array of visible markers poses |
cameraPose| array of actual cameras poses to theirs relatieve markers |

/aruco_marker

* visualization of markers in R-Viz

## Parameters:

Name          | Type         | Default value       | Comment                  |
------------- | -------------| --------------------| -------------------------|
calibration_file | string | - | Path to calibration file |
MarkerSize | int | 0.1 | Size of ArUco marker |
markers_number | int | 35 | Number of used markers for mapping |
type_of_markers_space | string | plane | Plane for 2D space or Cube for 3D space |
start_now | bool | true | switching | Switch off starting by empty message |
region_of_interest | bool | false | Switch off Region of Interest of input image |
region_of_interest_x | int | 0 | Starting pixel of ROI |
region_of_interest_y | int | 0 | Starting pixel of ROI |
region_of_interest_widht | int | 10 | Width of ROI in pixels |
region_of_interest_height | int | 5 | Height of ROI in pixels |

## Performance improvement:

* for better detection of ArUco markers use our [image filter] (https://github.com/SmartRoboticSystems/image_filtering_for_aruco.git)
* performance depends on calibration of your camera
* better accuracy is achieved when all markers are in the plane
