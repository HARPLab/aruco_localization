/*
 * ArucoDetector.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: reubena
 */

#include "aruco_detection/ArucoDetector.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/make_shared.hpp>

namespace aruco {


aruco_detection::Board DetectionResult::BoardDetection::asBoard() const {
	aruco_detection::Board board_msg;
	board_msg.board_name = this->name;
	board_msg.num_detections = this->num_detections;
	board_msg.num_inliers = this->num_inliers;
	if (this->transform_computed) {
		board_msg.pose = this->asPose(false);
	}
	return board_msg;
}

geometry_msgs::PoseWithCovariance DetectionResult::BoardDetection::asPose(bool invert) const {
	if (this->num_inliers == 0) {
		throw std::runtime_error("Transform was not computed");
	}
	geometry_msgs::PoseWithCovariance pose;
	if (invert) {
		pose.pose = tf2::toMsg(this->transform.inverse(), pose.pose);
	} else {
		pose.pose = tf2::toMsg(this->transform, pose.pose);
	}
	pose.covariance = this->covariance;
	return pose;
}

geometry_msgs::PoseWithCovariancePtr DetectionResult::BoardDetection::asPosePtr(bool invert) const {
	return boost::make_shared<geometry_msgs::PoseWithCovariance>(this->asPose(invert));
}


aruco_detection::Boards DetectionResult::asBoards() const {
	aruco_detection::Boards boards;
	std::transform(this->detections.begin(), this->detections.end(),
			std::back_inserter(boards.boards), [] (DetectionResult::BoardDetection const & det) {
		return det.asBoard();
	} );
	return boards;
}

ArucoDetector::ArucoDetector(std::vector<ArucoBoard> && boards,
		bool const useRectifiedImages) :
		boards(boards), useRectifiedImages(useRectifiedImages),
#ifdef CV_ARUCO_ADV
				detector_params(cv::aruco::DetectorParameters::create()),
#else // CV_ARUCO_ADV
				detector_params(),
#endif // CV_ARUCO_ADV
				cameraModel() {
}

ArucoDetector::ArucoDetector(std::vector<ArucoBoard> const & boards,
		bool const useRectifiedImages) :
		boards(boards), useRectifiedImages(useRectifiedImages),
#ifdef CV_ARUCO_ADV
				detector_params(cv::aruco::DetectorParameters::create()),
#else // CV_ARUCO_ADV
				detector_params(),
#endif // CV_ARUCO_ADV
				cameraModel() {
}

ArucoDetector::ArucoDetector(ArucoDetector const & other) :
		boards(other.boards),
		useRectifiedImages(other.useRectifiedImages),
#ifdef CV_ARUCO_ADV
		detector_params(new cv::aruco::DetectorParameters(*other.detector_params)),
#else // CV_ARUCO_ADV
		detector_params(other.detector_params),
#endif // CV_ARUCO_ADV
		cameraModel(std::unique_ptr<CameraModel const>(new CameraModel(*other.cameraModel)))
	{}

ArucoDetector::ArucoDetector(ArucoDetector && other) :
		boards(std::move(other.boards)),
		useRectifiedImages(std::move(other.useRectifiedImages)),
		detector_params(std::move(other.detector_params)),
		cameraModel(std::move(other.cameraModel))
	{}



DetectionResult ArucoDetector::detect(cv::Mat const & image,
		bool const debug_image) const {
	DetectionResult result;

	if (!this->is_initialized()) {
		throw std::runtime_error("No camera info received!");
	}

	cv::Mat ref_image;
	if (!this->useRectifiedImages) {
		this->cameraModel->rectifyImage(image, ref_image);
	} else {
		ref_image = image;
	}

	if (debug_image) {
		result.debugImage = ref_image.clone();
	}

	std::transform(this->boards.begin(), this->boards.end(),
			std::back_inserter(result.detections),
			[&] (ArucoBoard const & board) {

				DetectionResult::BoardDetection detection;
				detection.name = board.name;
				// Detect raw markers
				std::vector< std::vector< cv::Point2f > > rejected_corners;

				cv::aruco::detectMarkers(ref_image, board.get_dict(), detection.marker_corners, detection.marker_ids, detector_params, rejected_corners);
				cv::aruco::refineDetectedMarkers(ref_image, board.board, detection.marker_corners, detection.marker_ids, rejected_corners);

				detection.num_detections = detection.marker_ids.size();
				detection.num_inliers = 0;

				if (debug_image) {
					cv::aruco::drawDetectedMarkers(result.debugImage, detection.marker_corners, detection.marker_ids);
				}

				if (detection.marker_ids.size() > 3) { // at least 3 detected points
					cv::Mat rvec, tvec;// TODO: cache last position

					// manually run estimatePoseBoard instead of calling the function so we can
					// run getBoardObjAndImagePoints only once
//					detection.num_inliers = cv::aruco::estimatePoseBoard(detection.marker_corners, detection.marker_ids, board.board, this->cameraModel->fullIntrinsicMatrix(),
//							cv::Mat_<double>::zeros(4, 1), rvec, tvec);
					cv::Mat obj_points, img_points;
					cv::aruco::getBoardObjectAndImagePoints(board.board, detection.marker_corners, detection.marker_ids, obj_points, img_points);
					cv::solvePnP(obj_points, img_points, this->cameraModel->fullIntrinsicMatrix(), cv::Mat_<double>::zeros(4, 1), rvec, tvec, false);
					//TODO: cache previous solutions and use it to seed the current one
					detection.num_inliers = obj_points.total()/4;

					if (detection.num_inliers > 0) {
						cv::Mat rot;
						cv::Rodrigues(rvec, rot);

						detection.transform = tf2::Transform(
								tf2::Matrix3x3(
										rot.at<double>(0), rot.at<double>(1), rot.at<double>(2),
										rot.at<double>(3), rot.at<double>(4), rot.at<double>(5),
										rot.at<double>(6), rot.at<double>(7), rot.at<double>(8)),
								tf2::Vector3(
										tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

						// Compute detection covariance
						cv::Mat img_points_proj, jacobian_full;
						cv::projectPoints(obj_points, rvec, tvec, this->cameraModel->fullIntrinsicMatrix(), cv::Mat_<double>::zeros(4, 1), img_points_proj, jacobian_full);
						cv::Mat jacobian(jacobian_full, cv::Range::all(), cv::Range(0, 6));
						static cv::Mat const permutation = (cv::Mat_<double>(6,6) <<
								0, 0, 0, 1, 0, 0,
								0, 0, 0, 0, 1, 0,
								0, 0, 0, 0, 0, 1,
								1, 0, 0, 0, 0, 0,
								0, 1, 0, 0, 0, 0,
								0, 0, 1, 0, 0, 0
							);
						double const error = cv::norm(img_points, img_points_proj) * 2 / img_points.total();
						cv::Mat cov = (jacobian.t()*jacobian).inv() * permutation * cv::norm(img_points, img_points_proj);

						cov.copyTo(cv::Mat(6, 6, cv::DataType<decltype(detection.covariance)::value_type>::depth, detection.covariance.c_array()));

						if (debug_image) {
							// project axis points
							cv::Point2d origin = this->cameraModel->rectifyPoint(this->cameraModel->projectPoint(rvec, tvec, cv::Point3d(0, 0, 0)));
							cv::Point2d ax_x = this->cameraModel->rectifyPoint(this->cameraModel->projectPoint(rvec, tvec, cv::Point3d(0.1, 0, 0)));
							cv::Point2d ax_y = this->cameraModel->rectifyPoint(this->cameraModel->projectPoint(rvec, tvec, cv::Point3d(0, 0.1, 0)));
							cv::Point2d ax_z = this->cameraModel->rectifyPoint(this->cameraModel->projectPoint(rvec, tvec, cv::Point3d(0, 0, 0.1)));

							// draw axis lines
							cv::line(result.debugImage, origin, ax_x, cv::Scalar(0, 0, 255), 3);
							cv::line(result.debugImage, origin, ax_y, cv::Scalar(0, 255, 0), 3);
							cv::line(result.debugImage, origin, ax_z, cv::Scalar(255, 0, 0), 3);
						}
					}

				}

				return detection;
			});
	return result;
}


void ArucoDetector::update_camera_info(const sensor_msgs::CameraInfo & cam_info) {
	this->cameraModel.reset(new CameraModel(cam_info));
}

} // namespace aruco

