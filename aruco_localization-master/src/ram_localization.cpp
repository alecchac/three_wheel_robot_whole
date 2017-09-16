//Katherine Liu
//UCSD, Dynamics Systems and Controls
//2014

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Int32.h"
#include "nav_msgs/Odometry.h"
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/markerdetector.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>

//INITIALIZE VARIABLES

//Define robot markers
static const int _BOTICELLI = 6;
static const int _LEONARDO = 20;
static const int _DONATELLO = 4;
static const int _RAPHAEL = 5;
static const int _TITIAN = 7;
static const int _MASACCIO = 3;
static const int _MICHELANGELO = 15;
static const int _GHIBERTI = 80;
static const int _BERNINI = 10;
static const int _BELLINI = 2;
static const int _PICASSO= 21;

//AARON WAS HERE
int ghost[100];
bool active[100];


//Try maps
std::map<int, std::string> robot_name_map;
std::map<int, ros::Publisher> odom_pub_map;
std::map<int, ros::Publisher> poly_map;
std::map<int, ros::Publisher> camPose_pub_map;
std::map<int, ros::Publisher> particleCloudPub_map;

//Create the publishers for the location data, for each robot
image_transport::Publisher pub;
ros::Publisher odom_pub_boti;
ros::Publisher camPose_pub_boti;
ros::Publisher particleCloudPub_boti;
ros::Publisher poly_boti;


ros::Publisher odom_pub_titian;
ros::Publisher camPose_pub_titian;
ros::Publisher particleCloudPub_titian;
ros::Publisher poly_titian;


ros::Publisher odom_pub_raphael;
ros::Publisher camPose_pub_raphael;
ros::Publisher particleCloudPub_raphael;
ros::Publisher poly_raphael;

ros::Publisher odom_pub_donatello;
ros::Publisher camPose_pub_donatello;
ros::Publisher particleCloudPub_donatello;
ros::Publisher poly_donatello;

ros::Publisher odom_pub_leonardo;
ros::Publisher camPose_pub_leonardo;
ros::Publisher particleCloudPub_leonardo;
ros::Publisher poly_leonardo;

ros::Publisher odom_pub_masaccio;
ros::Publisher camPose_pub_masaccio;
ros::Publisher particleCloudPub_masaccio;
ros::Publisher poly_masaccio;

ros::Publisher odom_pub_michelangelo;
ros::Publisher camPose_pub_michelangelo;
ros::Publisher particleCloudPub_michelangelo;
ros::Publisher poly_michelangelo;

ros::Publisher odom_pub_ghiberti;
ros::Publisher camPose_pub_ghiberti;
ros::Publisher particleCloudPub_ghiberti;
ros::Publisher poly_ghiberti;

ros::Publisher camPose_pub_bernini;
ros::Publisher particleCloudPub_bernini;
ros::Publisher poly_bernini;

ros::Publisher odom_pub_bellini;
ros::Publisher camPose_pub_bellini;
ros::Publisher particleCloudPub_bellini;
ros::Publisher poly_bellini;

ros::Publisher odom_pub_picasso;
ros::Publisher camPose_pub_picasso;
ros::Publisher particleCloudPub_picasso;
ros::Publisher poly_picasso;

tf::TransformBroadcaster *odom_broadcaster = NULL;

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Set up the Aruco detection variables
aruco::MarkerDetector MDetector;
vector<aruco::Marker> Markers;
aruco::CameraParameters CParam;

//Scaling factoring 
const double PIXEL_X = 640;
const double PIXEL_Y = 480;
//const double SCALING_FACTOR_X = 3.35/PIXEL_X;
//const double SCALING_FACTOR_Y = 2.44/PIXEL_Y;
const double SCALING_FACTOR_X = 1.0;
const double SCALING_FACTOR_Y = 1.0;
const double PI = 3.14159265;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "MULTIAGENT LOCALIZATION";

//initalize the messages that will be published
nav_msgs::Odometry odom;
geometry_msgs::PoseWithCovarianceStamped camPose;
geometry_msgs::PoseWithCovarianceStamped poly_pose;
geometry_msgs::Quaternion odom_quat;
geometry_msgs::TransformStamped odom_trans;
geometry_msgs::PoseArray particleCloud;
ros::Time current_time;

std::string intToString(int number){
    std::stringstream ss;
    ss << number;
    return ss.str();
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    current_time = ros::Time::now();
    
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
 
    //Call to Aruco to identify markers
    MDetector.detect(cv_ptr->image, Markers,CParam,.025);
    if (CParam.isValid()) {
	     for (unsigned int i=0;i<Markers.size();i++) {
		//A different option for drawing                	
		//aruco::CvDrawingUtils::draw3dCube(cv_ptr->image,Markers[i],CParam);
               	//aruco::CvDrawingUtils::draw3dAxis(cv_ptr->image,Markers[i],CParam);
		Markers[i].draw(cv_ptr->image,cv::Scalar(0,0,255),2);

		//Get coordinates of the center
		cv::Point2f marker_centroid = Markers[i].getCenter();
		camPose.pose.pose.position.x = marker_centroid.x*SCALING_FACTOR_X;
		camPose.pose.pose.position.y = (420-marker_centroid.y)*SCALING_FACTOR_Y-0; //SWITCH TO -500 FOR 2CAM

		//calculate the heading
		double myHeading;
  		float size = Markers[i].ssize*3;
    		cv::Mat objectPoints (4,3,CV_32FC1);
    		objectPoints.at<float>(0,0)=0;
    		objectPoints.at<float>(0,1)=0;
    		objectPoints.at<float>(0,2)=0;
    		objectPoints.at<float>(1,0)=size;
    		objectPoints.at<float>(1,1)=0;
    		objectPoints.at<float>(1,2)=0;
    		objectPoints.at<float>(2,0)=0;
    		objectPoints.at<float>(2,1)=size;
    		objectPoints.at<float>(2,2)=0;
    		objectPoints.at<float>(3,0)=0;
    		objectPoints.at<float>(3,1)=0;
    		objectPoints.at<float>(3,2)=size;
   		vector<cv::Point2f> axisPoints;
    		cv::projectPoints(objectPoints, Markers[i].Rvec, Markers[i].Tvec, CParam.CameraMatrix, CParam.Distorsion, axisPoints);
		myHeading = atan2(-(axisPoints[1].y-axisPoints[0].y),(axisPoints[1].x-axisPoints[0].x));
		//draw the heading
		cv::line(cv_ptr->image,axisPoints[0],axisPoints[1], cv::Scalar(0,0,255,255),1,CV_AA);
		
		//Construct the odom message
		odom_quat = tf::createQuaternionMsgFromYaw(myHeading);	
		odom_quat = tf::createQuaternionMsgFromYaw(myHeading);	
		ros::Time current_time;
		current_time = ros::Time::now();	
		odom.header.stamp = current_time;
		camPose.header.stamp = current_time;
		odom.pose.pose.position.x = 0.0;
		odom.pose.pose.position.y = 0.0;
		odom.pose.pose.position.z = 0.0;
		//TODO: not sure what this is
		odom.pose.covariance[0]  = .1237;
		odom.pose.covariance[7]  = .1237;
		odom.pose.covariance[14] = 1.24069;
		odom.pose.covariance[21] = .1237;
		odom.pose.covariance[28] = .124085;
		odom.pose.covariance[35] = .1237;
		
		camPose.pose.pose.position.z = 0.0;
		camPose.pose.pose.orientation = odom_quat;
		//for tf
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "/map";
		odom_trans.transform.translation.x = marker_centroid.x*SCALING_FACTOR_X;
		odom_trans.transform.translation.y = (420-marker_centroid.y)*SCALING_FACTOR_Y;
		//odom_trans.transform.translation.y = (420-marker_centroid.y)*SCALING_FACTOR_Y;
		//For debug		
		//odom_trans.transform.translation.x = marker_centroid.x;
		//odom_trans.transform.translation.y = marker_centroid.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		particleCloud.header = camPose.header;
		particleCloud.poses[0] = camPose.pose.pose;

		odom.header.frame_id = robot_name_map[Markers[i].id]+"/odom";
		camPose.header.frame_id = robot_name_map[Markers[i].id]+"/pose"; 
		odom_trans.child_frame_id = robot_name_map[Markers[i].id]+"/odom";

if (Markers[i].id == _BOTICELLI || Markers[i].id == _LEONARDO || Markers[i].id == _DONATELLO || Markers[i].id == _RAPHAEL  || Markers[i].id == _TITIAN || Markers[i].id == _MASACCIO || Markers[i].id == _MICHELANGELO || Markers[i].id == _GHIBERTI || Markers[i].id == _BERNINI || Markers[i].id == _BELLINI || Markers[i].id ==_PICASSO){


		ghost[Markers[i].id]=ghost[Markers[i].id]+1;
			if(ghost[Markers[i].id]>100){
				active[Markers[i].id]=true;
			}
			
			if (active[Markers[i].id]==true){				
				odom_pub_map[Markers[i].id].publish(odom);
				camPose_pub_map[Markers[i].id].publish(camPose);
				cout << "REACHED";	
				particleCloudPub_map[Markers[i].id].publish(particleCloud); 
				poly_pose.pose.pose.position.x=camPose.pose.pose.position.y;
				poly_pose.pose.pose.position.y=camPose.pose.pose.position.x;

				poly_map[Markers[i].id].publish(poly_pose);
			}
}

		cv::putText(cv_ptr->image,robot_name_map[Markers[i].id],cv::Point(marker_centroid.x,marker_centroid.y+20),1,1, cv::Scalar(0,255,0));
             }
	}
    //Display the tracked robots
    cv::imshow(WINDOW, cv_ptr->image);

    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);

    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    //pub.publish(cv_ptr->toImageMsg());
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_processor");

	odom_broadcaster = new(tf::TransformBroadcaster);
	
	ros::NodeHandle nh;

        image_transport::ImageTransport it(nh);

        cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE); //another option is: CV_WINDOW_NORMAL
 
        image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);

	robot_name_map[_BOTICELLI]="boticelli";
	robot_name_map[_LEONARDO]="leonardo";
	robot_name_map[_RAPHAEL]="raphael";
	robot_name_map[_DONATELLO]="donatello";
	robot_name_map[_TITIAN]="titian";
	robot_name_map[_MASACCIO]="masaccio"; 
	robot_name_map[_MICHELANGELO]="michelangelo";   
	robot_name_map[_BERNINI]="bernini";   
	robot_name_map[_BELLINI]="bellini"; 
	robot_name_map[_PICASSO]="picasso";

	odom_pub_map[_BOTICELLI] = nh.advertise<nav_msgs::Odometry>("boticelli/odom", 10);
	camPose_pub_map[_BOTICELLI] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("boticelli/amcl_pose", 1, true);
	particleCloudPub_map[_BOTICELLI] = nh.advertise<geometry_msgs::PoseArray>("boticelli/particlecloud",1,true);
	poly_map[_BOTICELLI] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("boticelli/poly_demo",1,true);

	particleCloud.poses.resize(1);

	odom_pub_map[_TITIAN] = nh.advertise<nav_msgs::Odometry>("titian/odom", 10);
	camPose_pub_map[_TITIAN] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("titian/amcl_pose", 1, true);
	particleCloudPub_map[_TITIAN] = nh.advertise<geometry_msgs::PoseArray>("titian/particlecloud",1,true);
	poly_map[_TITIAN] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("titian/poly_demo",1,true);

	odom_pub_map[_RAPHAEL] = nh.advertise<nav_msgs::Odometry>("raphael/odom", 10);
	camPose_pub_map[_RAPHAEL] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("raphael/amcl_pose", 1, true);
	particleCloudPub_map[_RAPHAEL] = nh.advertise<geometry_msgs::PoseArray>("raphael/particlecloud",1,true);
	poly_map[_RAPHAEL] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("raphael/poly_demo",1,true);

	odom_pub_map[_DONATELLO] = nh.advertise<nav_msgs::Odometry>("donatello/odom", 10);
	camPose_pub_map[_DONATELLO] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("donatello/amcl_pose", 1, true);
	particleCloudPub_map[_DONATELLO] = nh.advertise<geometry_msgs::PoseArray>("donatello/particlecloud",1,true);
	poly_map[_DONATELLO] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("donatello/poly_demo",1,true);

	odom_pub_map[_LEONARDO] = nh.advertise<nav_msgs::Odometry>("leonardo/odom", 10);
	camPose_pub_map[_LEONARDO] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("leonardo/amcl_pose", 1, true);
	particleCloudPub_map[_LEONARDO] = nh.advertise<geometry_msgs::PoseArray>("leonardo/particlecloud",1,true);
	poly_map[_LEONARDO] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("leonardo/poly_demo",1,true);

	odom_pub_map[_MASACCIO] = nh.advertise<nav_msgs::Odometry>("masaccio/odom", 10);
	camPose_pub_map[_MASACCIO] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("masaccio/amcl_pose", 1, true);
	particleCloudPub_map[_MASACCIO] = nh.advertise<geometry_msgs::PoseArray>("masaccio/particlecloud",1,true);
	poly_map[_MASACCIO] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("masaccio/poly_demo",1,true);

	odom_pub_map[_MICHELANGELO] = nh.advertise<nav_msgs::Odometry>("michelangelo/odom", 10);
	camPose_pub_map[_MICHELANGELO] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("michelangelo/amcl_pose", 1, true);
	particleCloudPub_map[_MICHELANGELO] = nh.advertise<geometry_msgs::PoseArray>("michelangelo/particlecloud",1,true);  
	poly_map[_MICHELANGELO] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("michelangelo/poly_demo",1,true); 

	odom_pub_map[_GHIBERTI] = nh.advertise<nav_msgs::Odometry>("ghiberti/odom", 10);
	camPose_pub_map[_GHIBERTI] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ram/amcl_pose", 1, true);
	particleCloudPub_map[_GHIBERTI] = nh.advertise<geometry_msgs::PoseArray>("ghiberti/particlecloud",1,true);  
	poly_map[_GHIBERTI] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ghiberti/poly_demo",1,true); 

	odom_pub_map[_BERNINI] = nh.advertise<nav_msgs::Odometry>("bernini/odom", 2);
	camPose_pub_map[_BERNINI] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("bernini/amcl_pose", 1, true);
	particleCloudPub_map[_BERNINI] = nh.advertise<geometry_msgs::PoseArray>("bernini/particlecloud",1,true);  
	poly_map[_BERNINI] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("bernini/poly_demo",1,true); 
	
	odom_pub_map[_BELLINI] = nh.advertise<nav_msgs::Odometry>("bellini/odom", 2);
	camPose_pub_map[_BELLINI] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("bellini/amcl_pose", 1, true);
	particleCloudPub_map[_BELLINI] = nh.advertise<geometry_msgs::PoseArray>("bellini/particlecloud",1,true);  
	poly_map[_BELLINI] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("bellini/poly_demo",1,true); 

	odom_pub_map[_PICASSO] = nh.advertise<nav_msgs::Odometry>("picasso/odom", 2);
	camPose_pub_map[_PICASSO] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("chip/amcl_pose", 1, true);
	
	particleCloudPub_map[_PICASSO] = nh.advertise<geometry_msgs::PoseArray>("picasso/particlecloud",1,true);  
	poly_map[_PICASSO] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("picasso/poly_demo",1,true); 

        pub = it.advertise("/camera_2/image_processed", 1);

	//CParam.readFromXMLFile("/home/kliu/aruco-1.2.4/build/utils/camera_old.yml");

	CParam.readFromXMLFile("/home/alec/catkin_ws/src/aruco_localization-master/camera_old.yml");
	//CParam.readFromXMLFile("/home/aaron/catkin_ws/src/aruco/build/utils/camera_old.yml");
	MDetector.setThresholdParams(7,7);
        
	ros::spin();
   
    	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
	cv::destroyWindow(WINDOW);
 }

