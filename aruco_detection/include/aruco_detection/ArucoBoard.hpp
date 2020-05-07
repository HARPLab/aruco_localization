/*
 * ArucoDetection.hpp
 *
 *  Created on: Jul 23, 2018
 *      Author: reubena
 */

#ifndef ARUCO_DETECTION_INCLUDE_ARUCO_DETECTION_ARUCOBOARD_HPP_
#define ARUCO_DETECTION_INCLUDE_ARUCO_DETECTION_ARUCOBOARD_HPP_

#include <ros/ros.h>
#include <opencv2/aruco.hpp>

#include "aruco_detection/CameraModel.hpp"

// handle different aruco dictionary definitions
// todo: check that this is actually the right version break
#if CV_VERSION_MAJOR >= 3 and CV_VERSION_MINOR >= 2
	typedef cv::Ptr<cv::aruco::Dictionary> DictType;
	typedef cv::Ptr<cv::aruco::Board> BoardType;
#define CV_ARUCO_ADV
#else
	typedef cv::aruco::Dictionary DictType;
	typedef cv::aruco::Board BoardType;
#endif

#if CV_VERSION_MAJOR >= 3 and CV_VERSION_MINOR >= 3
#define CV_ARUCO_OPENCV_3_3
#endif

namespace aruco {

extern const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> aruco_dict_lookup;


struct ArucoBoard {

#if CV_VERSION_MAJOR >= 3 and CV_VERSION_MINOR >= 2
	typedef cv::Ptr<cv::aruco::Dictionary> DictType;
	typedef cv::Ptr<cv::aruco::Board> BoardType;
#define CV_ARUCO_ADV
#else
	typedef cv::aruco::Dictionary DictType;
	typedef cv::aruco::Board BoardType;
#endif

#if CV_VERSION_MAJOR >= 3 and CV_VERSION_MINOR >= 3
#define CV_ARUCO_OPENCV_3_3
#endif

	std::string name;
	BoardType board;

	inline DictType const & get_dict() const {
#ifdef CV_ARUCO_ADV
		return this->board->dictionary;
#else // CV_ARUCO_ADV
		return this->board.dictionary;
#endif // CV_ARUCO_ADV
	}

	inline bool is_valid_id(int id) const {
		return std::any_of(board->ids.begin(), board->ids.end(), [id] (int val_id) { return id==val_id; } );
	}

	// Constructors
	static ArucoBoard::DictType getPredefinedDictionaryFromName(std::string const & name);
	static ArucoBoard load_from_param_ns(ros::NodeHandle const & nh, std::string const & name, std::string const & prefix = "~");
	static std::vector<ArucoBoard> load_from_params(ros::NodeHandle const & nh);
};



} // namespace aruco



#endif /* ARUCO_DETECTION_INCLUDE_ARUCO_DETECTION_ARUCOBOARD_HPP_ */
