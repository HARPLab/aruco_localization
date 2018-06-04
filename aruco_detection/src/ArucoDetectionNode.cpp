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
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcValue.h>

#include <boost/filesystem.hpp>

#include <aruco_detection/Boards.h>

const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> aruco_dict_lookup {
	    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
	    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
	    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
	    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
	    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
	    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
	    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
	    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
	    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
	    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
	    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
	    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
	    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
	    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
	    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
	    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
	    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL} };

struct ArucoBoard {
	std::string name;
	cv::Ptr<cv::aruco::Board> board;



	static cv::Ptr<cv::aruco::Dictionary> getPredefinedDictionaryFromName(std::string const & name) {
		try {
			return cv::aruco::getPredefinedDictionary(aruco_dict_lookup.at(name));
		} catch (std::out_of_range & e) {
			throw std::out_of_range(std::string("No such dictionary name: ") + name);
		}
	}

	static ArucoBoard load_from_file(std::string const & filename) {
		ROS_INFO_STREAM("Reading from file " << filename);

		ArucoBoard board;

		cv::FileStorage fs(cv::String(filename.c_str()), cv::FileStorage::READ );
		ROS_INFO_STREAM("File opened " << filename.c_str());

		cv::FileNode root = fs.getFirstTopLevelNode();
		for (auto it = root.begin(); it != root.end(); ++it) {
			ROS_INFO_STREAM( "node: " << (*it).name() << ": " << (*it).type());
		}

		std::string const dict = fs["dictionary"];
		ROS_INFO_STREAM("Read dictionary " << dict);
		int const marker_x = fs["marker_x"];
		int const marker_y = fs["marker_y"];
		double const marker_len = fs["marker_length"];
		double const marker_sep = fs["marker_separation"];

		cv::String name;
		cv::read(fs["name"], name, boost::filesystem::path(filename).stem().native());

		board.name = cv::String(fs["name"]).c_str();
		board.board = cv::aruco::GridBoard::create(marker_x, marker_y, marker_len, marker_sep, ArucoBoard::getPredefinedDictionaryFromName(dict));

		return board;
	}

	static std::vector<ArucoBoard> load_from_directory(std::string const & dir_name) {
		namespace fs = boost::filesystem;
		std::vector<ArucoBoard> boards;
		// this wants to be transform_if but that doesn't exist because ranges aren't ready yet :(
		std::for_each( fs::directory_iterator(fs::path(dir_name)), fs::directory_iterator(),
				[&boards] (fs::directory_entry const & entry) {
			if (fs::is_regular_file(entry.status())) {
				boards.push_back(ArucoBoard::load_from_file(entry.path().native()));
			}
		});

		return boards;
	}

	static ArucoBoard load_from_param_ns(ros::NodeHandle const & nh, std::string const & name, std::string const & prefix = "~") {
		ArucoBoard board;

		std::string dict;
		int marker_x, marker_y, start_id;
		double marker_len, marker_sep;

		std::string const full_prefix = prefix + "/" + name + "/";

		ROS_DEBUG_STREAM("Building board for " << full_prefix);

		nh.getParam(full_prefix + "dictionary", dict);
		nh.getParam(full_prefix + "marker_x", marker_x);
		nh.getParam(full_prefix + "marker_y", marker_y);
		nh.getParam(full_prefix + "marker_length", marker_len);
		nh.getParam(full_prefix + "marker_separation", marker_sep);
		if (nh.hasParam(full_prefix + "start_id")) {
			nh.getParam(full_prefix + "start_id", start_id);
		} else {
			start_id = 0;
		}

		board.name = name;
		board.board = cv::aruco::GridBoard::create(marker_x, marker_y, marker_len, marker_sep, ArucoBoard::getPredefinedDictionaryFromName(dict), start_id);

		return board;
	}

	static std::vector<ArucoBoard> load_from_params(ros::NodeHandle const & nh) {
//		std::vector<std::string> topics;
		XmlRpc::XmlRpcValue topics;
		typedef std::pair<std::string, XmlRpc::XmlRpcValue> PairType;
		nh.getParam("board", topics);

		ROS_DEBUG("Found boards:");
		std::for_each(topics.begin(), topics.end(), [] (PairType const & pair) {
			ROS_DEBUG_STREAM("\t" << pair.first);
		});

		ROS_DEBUG("Building boards...");
		std::vector<ArucoBoard> boards;
		std::transform(
				topics.begin(),
				topics.end(),
				std::back_inserter(boards),
				[&nh] ( PairType const & pair) {
			return ArucoBoard::load_from_param_ns(nh, pair.first, "board");
		});

		return boards;
	}

};



void convert_cv_to_ros( cv::Mat const & rvec, cv::Mat const & tvec, geometry_msgs::Pose & pose)
{
	cv::Mat rot;
	cv::Rodrigues(rvec, rot);

	cv::Mat rotate_to_sys = (cv::Mat_<double>(3,3) << 1.0, 0.0, 0.0,
			0.0, -1.0, 0.0,
			0.0, 0.0, -1.0);

	rot = rot*rotate_to_sys.t();

	pose.position.x = tvec.at<double>(0);
	pose.position.y = tvec.at<double>(1);
	pose.position.z = tvec.at<double>(2);

	tf2::Matrix3x3 const mat(rot.at<double>(0), rot.at<double>(1), rot.at<double>(2),
			rot.at<double>(3), rot.at<double>(4), rot.at<double>(5),
			rot.at<double>(6), rot.at<double>(7), rot.at<double>(8));
	tf2::Quaternion q;
	mat.getRotation(q);

	tf2::convert(q, pose.orientation);
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


	public:
		ArucoDetectionNode()
			: cam_info_received(false),
			nh("~"),
			it(nh) {
				this->image_sub = this->it.subscribe("/image", 1, &ArucoDetectionNode::image_callback, this);
				this->cam_info_sub = this->nh.subscribe("/camera_info", 1, &ArucoDetectionNode::cam_info_callback, this);

				this->image_pub = this->it.advertise("result", 1);
				this->detection_pub = this->nh.advertise<aruco_detection::Boards>("detections", 100);
				this->nh.param<bool>("image_is_rectified", this->useRectifiedImages, true);

				this->boards = ArucoBoard::load_from_params(this->nh);
				ROS_INFO("Aruco board detector node started with boards: ");
				std::for_each(this->boards.begin(), this->boards.end(),
						[] (ArucoBoard const & board) {
					ROS_INFO_STREAM("\t" << board.name);
				});
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
						board_msg.pose.covariance = boost::array<float, 36>{
							0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
							0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
							0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
							0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
							0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
							0.0, 0.0, 0.0, 0.0, 0.0, 0.001,
						}; // TODO: something smarter with this


						if (build_marked_image) {
							cv::aruco::drawAxis(resultImg, this->cam_matrix, this->dist_coeffs, rvec, tvec, 0.1);
							// TODO: different color per board?
						}

					}

					return board_msg;
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

			ROS_INFO("Received camera info.");
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

