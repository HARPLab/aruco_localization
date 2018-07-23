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
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcValue.h>

#include <boost/filesystem.hpp>


#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <aruco_detection/Boards.h>

#include "../include/aruco_detection/ArucoBoard.hpp"

namespace aruco {

void convert_cv_to_ros( cv::Mat const & rvec, cv::Mat const & tvec, geometry_msgs::Pose & pose)
{
	cv::Mat rot;
	cv::Rodrigues(rvec, rot);

	pose.position.x = tvec.at<double>(0);
	pose.position.y = tvec.at<double>(1);
	pose.position.z = tvec.at<double>(2);

	tf2::Matrix3x3 const mat(rot.at<double>(0), rot.at<double>(1), rot.at<double>(2),
			rot.at<double>(3), rot.at<double>(4), rot.at<double>(5),
			rot.at<double>(6), rot.at<double>(7), rot.at<double>(8));
	tf2::Quaternion q;
	mat.getRotation(q);

	pose.orientation.w = q.w();
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
}



struct ArucoDetectionNode {
	private:
		bool useRectifiedImages;

		ros::Subscriber cam_info_sub;
		bool cam_info_received;
		cv::Mat cam_matrix;
		cv::Mat dist_coeffs;

		image_transport::Publisher image_pub;
		ros::Publisher detection_pub;
		std::vector<ArucoBoard> boards;

		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;

		std::map<std::string, ros::Publisher> pose_publishers;


	public:
		ArucoDetectionNode()
			: cam_info_received(false),
			nh("~"),
			it(nh) {

				std::string const topic = this->nh.resolveName("/image");
				ROS_INFO_STREAM("Subscribing to image topic " << topic);

				this->image_sub = this->it.subscribe(topic + "/image_raw", 1, &ArucoDetectionNode::image_callback, this);
				this->cam_info_sub = this->nh.subscribe(topic + "/camera_info", 1, &ArucoDetectionNode::cam_info_callback, this);

				this->image_pub = this->it.advertise("result", 1);
				this->detection_pub = this->nh.advertise<aruco_detection::Boards>("detections", 100);
				this->nh.param<bool>("image_is_rectified", this->useRectifiedImages, true);

				this->boards = ArucoBoard::load_from_params(this->nh);
				ROS_INFO("Aruco board detector node started with boards: ");
				std::for_each(this->boards.begin(), this->boards.end(),
						[] (ArucoBoard const & board) {
					ROS_INFO_STREAM("\t" << board.name);
				});

				std::transform(this->boards.begin(), this->boards.end(), std::inserter(this->pose_publishers, this->pose_publishers.end()), [this] (ArucoBoard const & board) {
					return std::make_pair(board.name, this->nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(std::string("boards/" + board.name), 1));
				});
		}

		void image_callback(const sensor_msgs::ImageConstPtr& msg)
		{
			if (!cam_info_received) {
				return;
			}

			ROS_DEBUG_STREAM("Image info:\n"
					<< "\tWidth: " << msg->width << "\n"
					<< "\tHeight: " << msg->height << "\n"
					<< "\tEncoding: " << msg->encoding << "\n"
					<< "\tSize: " << msg->data.size()
					);

			cv_bridge::CvImagePtr cv_ptr;
			try	{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			} catch (cv_bridge::Exception & e) {
				ROS_ERROR_STREAM("Failed to convert ptr: " << e.what());
				return;
			}

			cv::Mat const & inImage = cv_ptr->image;
			cv::Mat resultImg = cv_ptr->image.clone();

			// todo: read this from config, maybe even board-specific?
#ifdef CV_ARUCO_ADV
			cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::aruco::DetectorParameters::create();
#else // CV_ARUCO_ADV
			cv::aruco::DetectorParameters detector_params;
#endif // CV_ARUCO_ADV

			bool const build_marked_image = (this->image_pub.getNumSubscribers() > 0);


			aruco_detection::Boards boards;
			boards.header.stamp = msg->header.stamp;
			boards.header.frame_id = msg->header.frame_id;

			std::transform( this->boards.begin(), this->boards.end(), std::back_inserter(boards.boards), [&] (ArucoBoard const & board) {

				// Detect raw markers
				std::vector< std::vector< cv::Point2f > > marker_corners;
				std::vector< int > marker_ids;
				std::vector< std::vector< cv::Point2f > > rejected_corners;


				cv::aruco::detectMarkers(inImage, board.get_dict(), marker_corners, marker_ids, detector_params, rejected_corners,
						this->cam_matrix, this->dist_coeffs);
				cv::aruco::refineDetectedMarkers(inImage, board.board, marker_corners, marker_ids, rejected_corners, this->cam_matrix,
						this->dist_coeffs);

				aruco_detection::Board board_msg;
				board_msg.board_name = board.name;
				board_msg.num_detections = marker_ids.size();
				board_msg.num_inliers = 0;

				if (build_marked_image) {
					cv::aruco::drawDetectedMarkers(resultImg, marker_corners, marker_ids);
					// TODO: different color per board?
				}

				if (marker_ids.size() > 3) { // at least 3 detected points
					cv::Mat rvec, tvec; // TODO: cache last position
					board_msg.num_inliers = cv::aruco::estimatePoseBoard(marker_corners, marker_ids, board.board, this->cam_matrix, this->dist_coeffs,
							rvec, tvec);

					if (board_msg.num_inliers > 0) {
						convert_cv_to_ros(rvec, tvec, board_msg.pose.pose);
						board_msg.pose.covariance = boost::array<float, 36>{
							0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
							0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
							0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
							0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
							0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
							0.0, 0.0, 0.0, 0.0, 0.0, 0.001,
						}; // TODO: something smarter with this

						// publish this board
						ros::Publisher & pub = this->pose_publishers[board.name];
						if (pub.getNumSubscribers() > 0) {
							// Convert it into a frame that we actually care about

							geometry_msgs::PoseWithCovarianceStamped pose_stamped;
							pose_stamped.pose = board_msg.pose;
							pose_stamped.header = boards.header;
							// We want the vector from the board to the image frame, not from the image frame to the board
							// So invert this pose and fix the frame id
							tf2::Transform t;
							tf2::fromMsg(pose_stamped.pose.pose, t);
							tf2::toMsg(t.inverse(), pose_stamped.pose.pose);

							pose_stamped.header.frame_id = board.name;
							try {
								pub.publish(pose_stamped);
							} catch (tf2::TransformException & ex) {
								ROS_WARN_STREAM("Failed to transform message to frame: " << ex.what());
							}
						}


						if (build_marked_image) {
							cv::aruco::drawAxis(resultImg, this->cam_matrix, this->dist_coeffs, rvec, tvec, 0.1);
							// TODO: different color per board?
						}
					}

				}

				return board_msg;
			} );

			if (build_marked_image) {
				//show input with augmented information
				cv::aruco::drawAxis(resultImg, this->cam_matrix, this->dist_coeffs, cv::Mat::zeros(1, 3, CV_64F), cv::Mat::zeros(1, 3, CV_64F), 0.1);
				cv_bridge::CvImage out_msg;
				out_msg.header.frame_id = msg->header.frame_id;
				out_msg.header.stamp = msg->header.stamp;
				out_msg.encoding = sensor_msgs::image_encodings::BGR8;
				out_msg.image = resultImg;
				image_pub.publish(out_msg.toImageMsg());
			}

			this->detection_pub.publish(boards);
		}

		// wait for one camera info, then shut down that subscriber
		void cam_info_callback(const sensor_msgs::CameraInfo & cam_info) {
			if ( this->useRectifiedImages ) {
				this->cam_matrix = (cv::Mat_<double>(3,3) <<
						cam_info.P[0], cam_info.P[1], cam_info.P[2],
						cam_info.P[4], cam_info.P[5], cam_info.P[6],
						cam_info.P[8], cam_info.P[9], cam_info.P[10]);

				this->dist_coeffs = (cv::Mat_<double>(4,1) << 0., 0., 0., 0.);
			}
			else {
				this->cam_matrix = (cv::Mat_<double>(3,3) <<
						cam_info.K[0], cam_info.K[1], cam_info.K[2],
						cam_info.K[3], cam_info.K[4], cam_info.K[5],
						cam_info.K[6], cam_info.K[7], cam_info.K[8]);

				this->dist_coeffs = cv::Mat(cam_info.D, true);
			}

			ROS_INFO_STREAM("Received camera info:\nK=" << this->cam_matrix << "\nD=" << this->dist_coeffs);
			cam_info_received = true;
			cam_info_sub.shutdown();
		}
};

} // namespace aruco


int main(int argc,char **argv)
{
	ros::init(argc, argv, "aruco_detection_node");

	aruco::ArucoDetectionNode node;

	ros::spin();
}

