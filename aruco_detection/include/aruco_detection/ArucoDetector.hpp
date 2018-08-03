/*
 * ArucoDetector.hpp
 *
 *  Created on: Jul 23, 2018
 *      Author: reubena
 */

#ifndef ARUCO_DETECTION_INCLUDE_ARUCO_DETECTION_ARUCODETECTOR_HPP_
#define ARUCO_DETECTION_INCLUDE_ARUCO_DETECTION_ARUCODETECTOR_HPP_

#include <vector>
#include <tf2/LinearMath/Transform.h>

#include "aruco_detection/Boards.h"

#include "aruco_detection/ArucoBoard.hpp"
#include "aruco_detection/CameraModel.hpp"

namespace aruco {

struct DetectionResult {
	struct BoardDetection {
		std::string name;
		tf2::Transform transform;
		boost::array<float, 36> covariance;
		std::size_t num_detections;
		std::size_t num_inliers;

		aruco_detection::Board asBoard() const;
		geometry_msgs::PoseWithCovariance asPose(bool invert = false) const;
		geometry_msgs::PoseWithCovariancePtr asPosePtr(bool invert = false) const;
	};

	cv::Mat debugImage;

	std::vector<BoardDetection> detections;

	aruco_detection::Boards asBoards() const;
};


struct ArucoDetector {

	public:
		ArucoDetector( std::vector<ArucoBoard> && boards, bool const useRectifiedImages );
		ArucoDetector( std::vector<ArucoBoard> const & boards, bool const useRectifiedImages );

		void update_camera_info(const sensor_msgs::CameraInfo & cam_info);
		inline CameraModel const & get_camera_model() const { return *cameraModel; }

		DetectionResult detect(cv::Mat const & image, bool const debug_image = false) const;

#ifdef CV_ARUCO_ADV
	typedef cv::Ptr<cv::aruco::DetectorParameters> DetectorParamsType;
#else // CV_ARUCO_ADV
	typedef cv::aruco::DetectorParameters DetectorParamsType;
#endif // CV_ARUCO_ADV

		DetectorParamsType detector_params;

		inline std::vector<ArucoBoard> const & get_boards() const { return this->boards; }
		inline bool is_initialized() const { return static_cast<bool>(this->cameraModel); }

	private:
		bool const useRectifiedImages;
		std::vector<ArucoBoard> const boards;
		std::unique_ptr<CameraModel const> cameraModel;
};


}


#endif /* ARUCO_DETECTION_INCLUDE_ARUCO_DETECTION_ARUCODETECTOR_HPP_ */
