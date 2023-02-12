/**
* \file detect_marker.cpp
* \brief noed for controlling the robot arm with the camera at the top of it in order to detect all the seven markers * \author Daniele Martino Parisi
* \version 0.1
* \date 12/02/2023

* Subscribes to: <BR>
* ° /robot/camera1/image_raw *
* Publishes to: <BR>
* ° /robot/joint1_position_controller/command
*
* ° /robot/joint2_position_controller/command
*
* ° /robot/joint3_position_controller/command
*
* ° /info/rooms
*
* Services : <BR>
* ° /RoomInformation *
* Description : *
* This node make the robot detecting all the seven markers associating to the seven locations
* by controlling the robot in order to make two 360 degrees loops: the first to detect the 4
* markers at the bottom and the second to detect the other 3 at the top. The node then publishes
* the informations about the locations on the topic /info/rooms.
*
*/

#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <stdlib.h>
#include "std_msgs/Float64.h"
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <map>
#include <vector>

#include <assignment2/RoomInformation.h>

#include <assignment2/RoomConnection.h>

#include <assignment2/InfoRoom.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>


// poses store
std_msgs::Float64 msg;

ros::Publisher joint_1_pose_pub; //! publisher to the first joint topic
ros::Publisher joint_2_pose_pub; //! publisher to the second joint topic
ros::Publisher joint_3_pose_pub; //! publisher to the third joint topic

std::map<int, bool> detected_marker_map;   //! map to store if a marker has been detected or not
std::vector<int> id_marker_list;           //! vector to store the ids of the detected markers only one time

ros::ServiceClient marker_info_client;  //! Service client to retrieve room information embedded in the marker

int counter = 0; //! counter to count the seconds for temporizing the arm motion

/*!
 * \class MyArucoMarkerPublisher
 *
 * \brief A class for detecting markers with the camera and for publishing informations about rooms.
 *
 * This class detects the Aruco markers with the camera, asks for room informations embedded in them through
 * the `/info_room` service (provided by `marker_server`) and publish those informations in the `info/room` topic.
 *
 */
class MyArucoMarkerPublisher
{
private:
  // ArUco stuff
  aruco::MarkerDetector mDetector_; //! Aruco library's marker detector object
  std::vector<aruco::Marker> markers_; //! vector to store the markers

  aruco::CameraParameters camParam_;

  // node params
  double marker_size_;
  bool useCamInfo_;

  // ROS pub-sub
  ros::NodeHandle nh_; //! ROS Node handler
  image_transport::ImageTransport it_; //! Image transport to receive the camera image
  image_transport::Subscriber image_sub_; //! ROS image subscriber
  image_transport::Publisher debug_pub_;

  // ROS pub
  ros::Publisher room_information_pub; //! publisher to the /info/rooms topic

  int markers_max = 7; //! maximum number of markers

  cv::Mat inImage_;

public:
  MyArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  {

    image_sub_ = it_.subscribe("/robot/camera1/image_raw", 1, &MyArucoMarkerPublisher::image_callback, this); //! subscriber to /robot/camera1/image_raw topic
    debug_pub_ = it_.advertise("debug", 1); //! publisher to debug topic

    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();

    room_information_pub = nh_.advertise<assignment2::InfoRoom>("/info/rooms",1); //! create a publisher
    while (room_information_pub.getNumSubscribers() < 1) //! while the subscriber in the other node (assignmen_fsm) are not ready do nothing
          ros::Duration(0.1).sleep();
  }

  /**
     * @brief Callback function called whenever a message is published to the `/robot/camera/image_raw` topic
     * @param msg Image message
     *
     * This function detects markers in the received image, retrieves room informations
     * embedded in the marker and send them to `/info/rooms topic. When it found all the
     * 7 markers it sends an empy message indicating that the transmission is over.
     */
  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      // it converts the ros image to an openCV pointer
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage_ = cv_ptr->image;

      // clear out previous detection results
      markers_.clear();

      // ok, let's detect the openCV image by using the 'detect' function of the MarkerDetector class
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false); // it returns a list of the all detected markers


        for (std::size_t i = 0; i < markers_.size(); ++i)
        {
          int marker_id;
          // check if is detected a feasible token
          if(markers_[i].id > 10 && markers_[i].id < 18)
                {
                  marker_id = markers_[i].id;
                }
          if(!detected_marker_map[marker_id])
          {
          	detected_marker_map[marker_id] = true;

          	std::cout << "The id of the detected marker detected is: ";
          	std::cout << marker_id << " ";
          	std::cout << std::endl;

                assignment2::RoomInformation srv;
                srv.request.id = marker_id;

                if (marker_info_client.call(srv))
                {
                	id_marker_list.push_back(marker_id);
                	// Print room informations of the marker
                    	std::cout << "Room Name = " << srv.response.room.c_str() << "\nRoom coordinates = [ " ;
                    	std::cout << srv.response.x << ", " << srv.response.y << " ]" << std::endl;

                      for (std::size_t j = 0; j < srv.response.connections.size(); ++j)
                      {
                        std::cout << srv.response.connections[j] << " ";
                      }

                      // Publish room informations
                    assignment2::InfoRoom room_information_msg;
                    room_information_msg.room = srv.response.room;
                    room_information_msg.x = srv.response.x;
                    room_information_msg.y = srv.response.y;
                    for (std::size_t j = 0; j < srv.response.connections.size(); j++){
                        room_information_msg.connections.push_back(srv.response.connections.at(j));
                    }
                    room_information_pub.publish(room_information_msg);

                }
                else
                {
                  ROS_WARN("Couldn't get room informations for this marker");
                }
          }

          // If all markers have been found, send an empty message to know the end of transmission
          if(id_marker_list.size() == markers_max)
          {
              // Publish an empty msg to comunicate the end of the transmission
              assignment2::InfoRoom empty_msg;
              room_information_pub.publish(empty_msg);

              // Stop execution
              printf("\n All 7 markers detected");
              ros::Duration(5).sleep();
              id_marker_list.clear();
          }
       }
    }


    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};


int main(int argc,char **argv)
{

  ros::init(argc, argv, "detect_marker");

  MyArucoMarkerPublisher node;

  ros::NodeHandle nh;

  ROS_INFO("main: instantiating an object of type MarkerDetectClass");

  joint_1_pose_pub = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command",100);
  joint_2_pose_pub = nh.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command",100);
  joint_3_pose_pub = nh.advertise<std_msgs::Float64>("/robot/joint3_position_controller/command",100);

  marker_info_client = nh.serviceClient<assignment2::RoomInformation>("/room_info");

  marker_info_client.waitForExistence();

  ros::Rate loop_rate(1);

  while(ros::ok())
  {

  while(counter != -1)
	{

		if(counter < 20)
		{

			msg.data = 6.28;
			joint_1_pose_pub.publish(msg);
			counter++;
		}
		else if( counter >= 20 && counter < 25)
		{
			msg.data = -1.3;
			joint_2_pose_pub.publish(msg);
			counter++;
		}
		else if( counter >= 25 && counter < 35)
		{
			msg.data = -1.0;
			joint_1_pose_pub.publish(msg);
			counter++;
		}

		else if (counter >= 35)
		{
			counter = -1;
			ROS_INFO("first loop done");
		}

		//loop_rate.sleep();
		ros::Duration(1).sleep();
		ros::spinOnce();
	}
  ros::spinOnce();
  }



  return 0;
}
