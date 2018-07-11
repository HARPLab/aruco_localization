/*
 * ArucoLocalizationNode.cpp
 *
 *  Created on: Jul 11, 2018
 *      Author: reubena
 */

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <aruco_detection/Boards.h>

struct ArucoLocalizationParams {

	ArucoLocalizationParams(ros::NodeHandle const & nh) {
		nh.param<std::string>("camera_name", this->camera_name, "camera");
		nh.param<std::string>("ref_frame", this->ref_frame, "world");
		nh.param<std::string>("aruco_detection_topic",
				this->aruco_detection_topic,
				"/aruco_detection_node/detections");
	}

	std::string camera_name;
	std::string ref_frame;
	std::string aruco_detection_topic;
};

struct ArucoLocalizationNode {
private:

	ros::NodeHandle nh;
	ros::Subscriber aruco_detection_sub;

	ArucoLocalizationParams const params;

	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	tf2_ros::TransformBroadcaster tf_broadcaster;

public:
	ArucoLocalizationNode() :
			nh("~"), params(nh), tf_buffer(), tf_listener(tf_buffer), tf_broadcaster() {

		this->aruco_detection_sub = nh.subscribe(
				this->params.aruco_detection_topic, 1,
				&ArucoLocalizationNode::detection_callback, this);
	}

	void detection_callback(aruco_detection::BoardsConstPtr const & boards) {
		std::vector<tf2::Transform> transforms;

		std::for_each(boards->boards.begin(), boards->boards.end(), [&] (aruco_detection::Board const & board) {
			try {
				// first see if we know the transform for that board
				geometry_msgs::TransformStamped base_tf_msg = this->tf_buffer.lookupTransform(board.board_name, this->params.ref_frame, boards->header.stamp);

				// now apply the transformation and add it to our list
				tf2::Transform base_tf, board_tf;
				tf2::fromMsg(base_tf_msg.transform, base_tf);
				tf2::fromMsg(board.pose.pose, board_tf);

				tf2::Transform full_tf;
				full_tf.mult(board_tf, base_tf);

			} catch (tf2::TransformException & ex) {
				ROS_WARN_STREAM("Failed to look up frame " << board.board_name << " to " << this->params.ref_frame << ": " << ex.what());
			}
		});

		tf2::Transform final_tf;

		// now average all the tfs together
		if (transforms.empty()) {
			ROS_WARN("No valid transforms found.");
			return;
		} else if (transforms.size() == 1) {
			final_tf = transforms[0];
		} else {
			// TODO: fancy averaging
			final_tf = transforms[0];
		}

		geometry_msgs::TransformStamped final_tf_msg;
		final_tf_msg.header.stamp = boards->header.stamp;
		final_tf_msg.header.frame_id = this->params.ref_frame;
		final_tf_msg.transform = tf2::toMsg(final_tf);

		this->tf_broadcaster.sendTransform(final_tf_msg);

	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "aruco_localization_node");

	ArucoLocalizationNode node;

	ros::spin();
}

