/*
 * ArucoDetectionNode.cpp
 *
 *  Created on: Jun 2, 2018
 *      Author: reubena
 */

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <XmlRpcValue.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <aruco_detection/Boards.h>

#include "aruco_detection/ArucoDetector.hpp"

template<class T>
T getParam(ros::NodeHandle & nh, std::string const & param_name,
		T const & def) {
	T t;
	nh.param<T>(param_name, t, def);
	return t;
}

namespace aruco {

struct ArucoDetectionNode {
private:

	ros::Subscriber cam_info_sub;

	image_transport::Publisher image_pub;
	ros::Publisher detection_pub;

	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;

	std::map<std::string, ros::Publisher> pose_publishers;

	ArucoDetector detector;

public:
	ArucoDetectionNode() :
			nh("~"), it(nh), detector(ArucoBoard::load_from_params(this->nh),
					getParam<bool>(this->nh, "image_is_rectified", true)) {

		std::string const topic = this->nh.resolveName("/image");
		ROS_INFO_STREAM("Subscribing to image topic " << topic);

		this->image_sub = this->it.subscribe(topic + "/image_raw", 1,
				&ArucoDetectionNode::image_callback, this);
		this->cam_info_sub = this->nh.subscribe(topic + "/camera_info", 1,
				&ArucoDetectionNode::cam_info_callback, this);

		this->image_pub = this->it.advertise("result", 1);
		this->detection_pub = this->nh.advertise<aruco_detection::Boards>(
				"detections", 100);

		std::transform(this->detector.get_boards().begin(), this->detector.get_boards().end(),
				std::inserter(this->pose_publishers,
						this->pose_publishers.end()),
				[this] (ArucoBoard const & board) {
					ROS_INFO_STREAM("Loaded board: " << board.name);
					return std::make_pair(board.name, this->nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(std::string("boards/" + board.name), 1));
				});
	}

	void image_callback(const sensor_msgs::ImageConstPtr& msg) {
		ros::Time start_tm = ros::Time::now();

		if (!this->detector.is_initialized()) {
			ROS_DEBUG("Still waiting for camera info.");
			return;
		}
		ROS_DEBUG("Received image");

		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception & e) {
			ROS_ERROR_STREAM("Failed to convert ptr: " << e.what());
			return;
		}
		bool const build_marked_image =
				(this->image_pub.getNumSubscribers() > 0);

		DetectionResult const result = this->detector.detect(cv_ptr->image, build_marked_image);

		ros::Time dxn_tm = ros::Time::now();
		aruco_detection::Boards boards_msg = result.asBoards();
		boards_msg.header.stamp = msg->header.stamp;
		boards_msg.header.frame_id = msg->header.frame_id;

		std::for_each(result.detections.begin(), result.detections.end(), [&msg, this] (DetectionResult::BoardDetection const & dxn) {
			ros::Publisher & pub = this->pose_publishers[dxn.name];
			if (pub.getNumSubscribers() > 0) {
				// Convert it into a frame that we actually care about

				geometry_msgs::PoseWithCovarianceStamped pose_stamped;
				pose_stamped.pose = dxn.asPose(true);
				pose_stamped.header.stamp = msg->header.stamp;
				pose_stamped.header.frame_id = dxn.name;
				pub.publish(pose_stamped);
			}
		});


		if (build_marked_image) {
			//show input with augmented information
			cv_bridge::CvImage out_msg;
			out_msg.header.frame_id = msg->header.frame_id;
			out_msg.header.stamp = msg->header.stamp;
			out_msg.encoding = sensor_msgs::image_encodings::BGR8;
			out_msg.image = result.debugImage;
			image_pub.publish(out_msg.toImageMsg());
		}

		this->detection_pub.publish(boards_msg);
		ros::Time end_tm = ros::Time::now();
		ROS_DEBUG_STREAM("Detected in " << (dxn_tm - start_tm) << " s, published in " << (end_tm - dxn_tm) << " s");
	}

	// wait for one camera info, then shut down that subscriber
	void cam_info_callback(const sensor_msgs::CameraInfo & cam_info) {
		ROS_INFO_STREAM("Got camera info");
		this->detector.update_camera_info(cam_info);
		cam_info_sub.shutdown();
	}
};

} // namespace aruco

int main(int argc, char **argv) {
	ros::init(argc, argv, "aruco_detection_node");

	aruco::ArucoDetectionNode node;

	ros::spin();
}

