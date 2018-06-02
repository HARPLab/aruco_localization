/*
 * ArucoDetectionNode.cpp
 *
 *  Created on: Jun 2, 2018
 *      Author: reubena
 */



#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <aruco_detection/Boards.h>

struct ArucoBoard {
	std::string name;
	cv::aruco::Board board;
};

struct ArucoDetectionNode {
	private:
		bool useRectifiedImages;
		bool draw_markers;
		bool draw_markers_cube;
		bool draw_markers_axis;
		ros::Subscriber cam_info_sub;
		bool cam_info_received;
		image_transport::Publisher image_pub;
		ros::Publisher detection_pub;
		std::string boards_config;
		std::string boards_directory;
		std::vector<ArucoBoard> boards;

		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;


	public:
		ArucoDetectionNode()
			: cam_info_received(false),
			nh("~"),
			it(nh) {
				this->image_sub = this->it.subscribe("/image", 1, &ArucoDetectionNode::image_callback, this);
				this->cam_info_sub = this->nh.subscribe("/camera_info", 1, &ArucoDetectionNode::cam_info_callback, this);

				this->image_pub = this->it.advertise("result", 1);
				this->detection_pub = this->nh.advertise<aruco_detection::Boards>("detections", 100);

				this->nh.param<std::string>("boards_config", this->boards_config, "boardsConfiguration.yml");
				this->nh.param<std::string>("boards_directory", this->boards_directory, "./data");
				this->nh.param<bool>("image_is_rectified", this->useRectifiedImages, true);
				this->nh.param<bool>("draw_markers", this->draw_markers, false);
				this->nh.param<bool>("draw_markers_cube", this->draw_markers_cube, false);
				this->nh.param<bool>("draw_markers_axis", this->draw_markers_axis, false);

				this->readFromFile(this->boards_config);
				ROS_INFO("ArSys node started with boards configuration: %s", boards_config.c_str());
		}

		void readFromFile ( std::string const & sfile ) throw ( cv::Exception ) {
			try {
				cv::FileStorage fs ( sfile.c_str(), cv::FileStorage::READ );
				readFromFile ( fs );
			}
			catch (std::exception &ex)
			{
				throw	cv::Exception ( 81818,"ArSysMultiBoards::readFromFile",ex.what()+string(" file=)")+sfile ,__FILE__,__LINE__ );
			}
		}


		void readFromFile ( cv::FileStorage &fs ) throw ( cv::Exception )
		{
			//look for the ar_sys_boards
			if (fs["ar_sys_boards"].name() != "ar_sys_boards")
				throw cv::Exception ( 81818,"ArSysMultiBoards::readFromFile","invalid file type" ,__FILE__,__LINE__ );

			cv::FileNode FnBoards=fs["ar_sys_boards"];
			for (cv::FileNodeIterator it = FnBoards.begin(); it != FnBoards.end(); ++it)
			{
				board_t board;

				board.uid = boards.size();
				board.name = (std::string)(*it)["name"];
				board.marker_size = (double)(*it)["marker_size"];

				std::string path(boards_directory);
				path.append("/");
				path.append((std::string)(*it)["path"]);
				board.config.readFromFile(path);

				boards.push_back(board);
			}

			ROS_ASSERT(boards.size() > 0);
		}

		void image_callback(const sensor_msgs::ImageConstPtr& msg)
		{
			if (!cam_info_received) {
				return;
			}

			cv_bridge::CvImagePtr cv_ptr;
			try {
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				cv::Mat const & inImage = cv_ptr->image;
				cv::Mat resultImg = cv_ptr->image.clone();

				bool const build_marked_image = (this->image_pub.getNumSubscribers() > 0);

				aruco_detection::Boards boards;

				std::transform( this->boards.begin(), this->boards.end(), std::back_inserter(boards.boards), [&] (ArucoBoard const & board) {

					// Detect raw markers
					std::vector< std::vector< cv::Point2f > > marker_corners;
					std::vector< int > marker_ids;

					float probDetect = the_board_detector.detect(markers, boards[board_index].config, board_detected, camParam, boards[board_index].marker_size);
					if (probDetect > 0.0)
					{
						tf::Transform transform = ar_sys::getTf(board_detected.Rvec, board_detected.Tvec);

						tf::StampedTransform stampedTransform(transform, msg->header.stamp, msg->header.frame_id, boards[board_index].name);

						geometry_msgs::PoseStamped poseMsg;
						tf::poseTFToMsg(transform, poseMsg.pose);
						poseMsg.header.frame_id = msg->header.frame_id;
						poseMsg.header.stamp = msg->header.stamp;
						pose_pub.publish(poseMsg);

						geometry_msgs::TransformStamped transformMsg;
						tf::transformStampedTFToMsg(stampedTransform, transformMsg);
						transform_pub.publish(transformMsg);

						geometry_msgs::Vector3Stamped positionMsg;
						positionMsg.header = transformMsg.header;
						positionMsg.vector = transformMsg.transform.translation;
						position_pub.publish(positionMsg);

						if(camParam.isValid())
						{
							//draw board axis
							CvDrawingUtils::draw3dAxis(resultImg, board_detected, camParam);
						}
					}
				} );

				//for each marker, draw info and its boundaries in the image
				for(size_t i=0; draw_markers && i < markers.size(); ++i)
				{
					markers[i].draw(resultImg,cv::Scalar(0,0,255),2);
				}

				if(camParam.isValid())
				{
					//draw a 3d cube in each marker if there is 3d info
					for(size_t i=0; i<markers.size(); ++i)
					{
						if (draw_markers_cube) CvDrawingUtils::draw3dCube(resultImg, markers[i], camParam);
						if (draw_markers_axis) CvDrawingUtils::draw3dAxis(resultImg, markers[i], camParam);
					}
				}

				if(image_pub.getNumSubscribers() > 0)
				{
					//show input with augmented information
					cv_bridge::CvImage out_msg;
					out_msg.header.frame_id = msg->header.frame_id;
					out_msg.header.stamp = msg->header.stamp;
					out_msg.encoding = sensor_msgs::image_encodings::RGB8;
					out_msg.image = resultImg;
					image_pub.publish(out_msg.toImageMsg());
				}
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
		}

		// wait for one camerainfo, then shut down that subscriber
		void cam_info_callback(const sensor_msgs::CameraInfo &msg) {
			camParam = ar_sys::getCamParams(msg, useRectifiedImages);
			cam_info_received = true;
			cam_info_sub.shutdown();
		}
};


int main(int argc,char **argv)
{
	ros::init(argc, argv, "aruco_detection_node");

	ArucoDetectionNode node;

	ros::spin();
}

