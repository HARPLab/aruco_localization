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
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <aruco_detection/Boards.h>

struct ArucoBoard {
	std::string name;
	cv::Ptr<cv::aruco::Board> board;
};


void convert_cv_to_ros( cv::Mat const & rvec, cv::Mat const & tvec, geometry_msgs::Pose & pose)
{
	cv::Mat rot;
	cv::Rodrigues(rvec, rot);

	cv::Matx33f rotate_to_sys( 1.0, 0.0, 0.0,
			0.0, -1.0, 0.0,
			0.0, 0.0, -1.0);

	rot = rot*rotate_to_sys.t();

	pose.position.x = tvec.at(0);
	pose.position.y = tvec.at(1);
	pose.position.z = tvec.at(2);

	tf2::Matrix3x3 const mat(rot.at(0), rot.at(1), rot.at(2),
			rot.at(3), rot.at(4), rot.at(5),
			rot.at(6), row.at(7), row.at(8));
	tf2::Quaternion q;
	mat.getRotation(q);

	tf2::convert(q, pose.quaternion);
}

struct ArucoDetectionNode {
	private:
		bool useRectifiedImages;
		bool draw_markers;
		bool draw_markers_cube;
		bool draw_markers_axis;

		ros::Subscriber cam_info_sub;
		bool cam_info_received;
		cv::Mat cam_matrix;
		cv::Mat dist_coeffs;


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

				// todo: read this from config, maybe even board-specific?
				cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::aruco::DetectorParameters::create();

				bool const build_marked_image = (this->image_pub.getNumSubscribers() > 0);

				aruco_detection::Boards boards;
				boards.header.stamp = msg->header.stamp;
				boards.header.frame_id = msg->header.frame_id;

				std::transform( this->boards.begin(), this->boards.end(), std::back_inserter(boards.boards), [&] (ArucoBoard const & board) {

					// Detect raw markers
					std::vector< std::vector< cv::Point2f > > marker_corners;
					std::vector< int > marker_ids;
					std::vector< std::vector< cv::Point2f > > rejected_corners;


					cv::aruco::detectMarkers(inImage, board.board->dictionary, marker_corners, marker_ids, detector_params, rejected_corners,
							this->cam_matrix, this->dist_coeffs);
					cv::aruco::refineDetectedMarkers(inImage, board.board, marker_corners, marker_ids, rejected_corners, this->cam_matrix,
							this->dist_coeffs);

					aruco_detection::Board board_msg;
					board_msg.board_name = board.name;
					board_msg.num_detections = marker_ids.size();

					if (build_marked_image) {
						cv::aruco::drawDetectedMarkers(resultImg, marker_corners, marker_ids);
						// TODO: different color per board?
					}

					if (marker_ids.size() > 3) { // at least 3 detected points
						cv::Mat rvec, tvec; // TODO: cache last position
						int pts_used = cv::aruco::estimatePoseBoard(marker_corners, marker_ids, board.board, this->cam_matrix, this->dist_coeffs,
								rvec, tvec);

						convert_cv_to_ros(rvec, tvec, board_msg.pose.pose);
						board_msg.pose.covariance = std::vector<float>( {
							0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
							0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
							0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
							0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
							0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
							0.0, 0.0, 0.0, 0.0, 0.0, 0.001,
						}); // TODO: something smarter with this


						if (build_marked_image) {
							cv::aruco::drawAxis(resultImg, this->cam_matrix, this->dist_coeffs, rvec, tvec, 0.1);
							// TODO: different color per board?
						}

					}
				} );

				if (build_marked_image) {
					//show input with augmented information
					cv_bridge::CvImage out_msg;
					out_msg.header.frame_id = msg->header.frame_id;
					out_msg.header.stamp = msg->header.stamp;
					out_msg.encoding = sensor_msgs::image_encodings::RGB8;
					out_msg.image = resultImg;
					image_pub.publish(out_msg.toImageMsg());
				}

				this->detection_pub.publish(boards);
			}
			catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
		}

		// wait for one camera info, then shut down that subscriber
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

