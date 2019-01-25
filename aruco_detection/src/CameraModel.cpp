#include "aruco_detection/CameraModel.hpp"
#include <sensor_msgs/distortion_models.h>
#include <ros/ros.h>
namespace image_geometry {

// copied from image_geometry/pinhole_camera_model.cpp
// so we can access the internals...
enum DistortionState {
	NONE, CALIBRATED, UNKNOWN
};

struct PinholeCameraModel::Cache {
	DistortionState distortion_state;

	cv::Mat_<double> K_binned, P_binned; // Binning applied, but not cropping

	mutable bool full_maps_dirty;
	mutable cv::Mat full_map1, full_map2;

	mutable bool reduced_maps_dirty;
	mutable cv::Mat reduced_map1, reduced_map2;

	mutable bool rectified_roi_dirty;
	mutable cv::Rect rectified_roi;

	Cache();
};

}

namespace aruco {

CameraModel::CameraModel(sensor_msgs::CameraInfo const & msg) :
		image_geometry::PinholeCameraModel() {
	this->fromCameraInfo(msg);
}

void CameraModel::initRectificationMapsFisheye() const {
	/// @todo For large binning settings, can drop extra rows/cols at bottom/right boundary.
	/// Make sure we're handling that 100% correctly.

	// we should ONLY be here if we're explicitly using the fisheye calibration model
	if (cache_->distortion_state != image_geometry::UNKNOWN
			|| this->cam_info_.distortion_model != "equidistant") {
		throw image_geometry::Exception("not a fisheye calibration!");
	}

	if (cache_->full_maps_dirty) {
		// Create the full-size map at the binned resolution
		/// @todo Should binned resolution, K, P be part of public API?
		cv::Size binned_resolution = fullResolution();
		binned_resolution.width /= binningX();
		binned_resolution.height /= binningY();

		cv::Matx33d K_binned;
		cv::Matx34d P_binned;
		if (binningX() == 1 && binningY() == 1) {
			K_binned = K_full_;
			P_binned = P_full_;
		} else {
			K_binned = K_full_;
			P_binned = P_full_;
			if (binningX() > 1) {
				double scale_x = 1.0 / binningX();
				K_binned(0, 0) *= scale_x;
				K_binned(0, 2) *= scale_x;
				P_binned(0, 0) *= scale_x;
				P_binned(0, 2) *= scale_x;
				P_binned(0, 3) *= scale_x;
			}
			if (binningY() > 1) {
				double scale_y = 1.0 / binningY();
				K_binned(1, 1) *= scale_y;
				K_binned(1, 2) *= scale_y;
				P_binned(1, 1) *= scale_y;
				P_binned(1, 2) *= scale_y;
				P_binned(1, 3) *= scale_y;
			}
		}

		// Note: m1type=CV_16SC2 to use fast fixed-point maps (see cv::remap)
		cv::fisheye::initUndistortRectifyMap(K_binned, D_, R_, P_binned,
				binned_resolution,
				CV_16SC2, cache_->full_map1, cache_->full_map2);
		cache_->full_maps_dirty = false;
	}

	if (cache_->reduced_maps_dirty) {
		/// @todo Use rectified ROI
		cv::Rect roi(cam_info_.roi.x_offset, cam_info_.roi.y_offset,
				cam_info_.roi.width, cam_info_.roi.height);
		if (roi.x != 0 || roi.y != 0 || roi.height != (int) cam_info_.height
				|| roi.width != (int) cam_info_.width) {

			// map1 contains integer (x,y) offsets, which we adjust by the ROI offset
			// map2 contains LUT index for subpixel interpolation, which we can leave as-is
			roi.x /= binningX();
			roi.y /= binningY();
			roi.width /= binningX();
			roi.height /= binningY();
			cache_->reduced_map1 = cache_->full_map1(roi)
					- cv::Scalar(roi.x, roi.y);
			cache_->reduced_map2 = cache_->full_map2(roi);
		} else {
			// Otherwise we're rectifying the full image
			cache_->reduced_map1 = cache_->full_map1;
			cache_->reduced_map2 = cache_->full_map2;
		}
		cache_->reduced_maps_dirty = false;
	}
}

cv::Point2d CameraModel::rectifyPoint(cv::Point2d const & uv_raw) const {
	try {
		return this->PinholeCameraModel::rectifyPoint(uv_raw);
	} catch (image_geometry::Exception &) {
		if (this->cam_info_.distortion_model == "equidistant") {
			/// @todo cv::undistortPoints requires the point data to be float, should allow double
			cv::Point2f raw32 = uv_raw, rect32;
			const cv::Mat src_pt(1, 1, CV_32FC2, &raw32.x);
			cv::Mat dst_pt(1, 1, CV_32FC2, &rect32.x);
			cv::fisheye::undistortPoints(src_pt, dst_pt, K_, D_, R_, P_);
			return rect32;
		} else {
			throw;
		}
	}
}

std::vector<cv::Point2d> CameraModel::rectifyPoints(std::vector<cv::Point2d> const & uv_raw) const {
	std::vector<cv::Point2d> out_pts;
	try {
		std::transform(uv_raw.begin(), uv_raw.end(), std::back_inserter(out_pts), [this] (cv::Point2d const & pt) {
			return this->PinholeCameraModel::rectifyPoint(pt);
		} );
		return out_pts;
	} catch (image_geometry::Exception &) {
		if (this->cam_info_.distortion_model == "equidistant") {
			/// @todo cv::undistortPoints requires the point data to be float, should allow double
			cv::fisheye::undistortPoints(uv_raw, out_pts, K_, D_, R_, P_);
			return out_pts;
		} else {
			throw;
		}
	}
}
void CameraModel::rectifyImage(const cv::Mat& raw, cv::Mat& rectified,
		int interpolation) const {

	assert(initialized());

	try {
		this->PinholeCameraModel::rectifyImage(raw, rectified, interpolation);
	} catch (image_geometry::Exception &) {
		this->CameraModel::initRectificationMapsFisheye();
		if (raw.depth() == CV_32F || raw.depth() == CV_64F) {
			cv::remap(raw, rectified, this->cache_->reduced_map1,
					this->cache_->reduced_map2, interpolation,
					cv::BORDER_CONSTANT,
					std::numeric_limits<float>::quiet_NaN());
		} else {
			cv::remap(raw, rectified, this->cache_->reduced_map1,
					this->cache_->reduced_map2, interpolation);
		}
	}
}

cv::Point2d CameraModel::projectPoint(cv::Mat const & rvec, cv::Mat const & tvec, const cv::Point3d& point3) const {

	std::vector<cv::Point2d> image_point;
	if (this->cam_info_.distortion_model == "equidistant") {
		cv::fisheye::projectPoints(std::vector<cv::Point3d>(1, point3), image_point, rvec,
			tvec, K_, D_);
	} else {
		cv::projectPoints(std::vector<cv::Point3d>(1, point3), rvec,
			tvec, K_, D_, image_point);
	}

	return image_point[0];
}

} // namespace aruco
