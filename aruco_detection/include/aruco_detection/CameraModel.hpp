/*
 * CameraModel.hpp
 *
 *  Created on: Jul 23, 2018
 *      Author: reubena
 */

#ifndef ARUCO_DETECTION_INCLUDE_CAMERAMODEL_HPP_
#define ARUCO_DETECTION_INCLUDE_CAMERAMODEL_HPP_

#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

namespace aruco {

struct CameraModel : public image_geometry::PinholeCameraModel {
	// need to get access to protected cache
	// PinholeCameraModel methods are *NOT* declared as virtual :(
	// so if you want access to fisheye functionality, you must use this class directly
	// or it will silently ignore your calibration
public:
	CameraModel(sensor_msgs::CameraInfo const & msg);
	void rectifyImage(cv::Mat const & raw, cv::Mat & rectified, int interpolation = cv::INTER_LINEAR) const;
	cv::Point2d rectifyPoint(cv::Point2d const & uv_raw) const;
	std::vector<cv::Point2d> rectifyPoints(std::vector<cv::Point2d> const & uv_raw) const;
	cv::Point2d projectPoint(cv::Mat const & rvec, cv::Mat const & tvec, const cv::Point3d& point3) const;
	void initRectificationMapsFisheye() const;

};


} // namespace aruco



#endif /* ARUCO_DETECTION_INCLUDE_CAMERAMODEL_HPP_ */
