#include "aruco_detection/ArucoDetector.hpp"
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/filesystem.hpp>

struct CovarianceEstimator {
	CovarianceEstimator(aruco::ArucoDetector && detector) :
		detector(detector) {}

	std::vector<cv::Mat> process_stationary_video(std::string const & video_file) {
		cv::VideoCapture video_cap(video_file);
		std::size_t const num_frames = video_cap.get(cv::CAP_PROP_FRAME_COUNT);

		// can't use the default std::vector<...>(size, default) because cv::Mat copy-constructor acts as a shared pointer
		// so need to explicitly generate each pair
		std::vector< std::pair<cv::Mat, std::size_t> > estimates;
		estimates.reserve(detector.get_boards().size());
		std::generate_n(std::back_inserter(estimates), detector.get_boards().size(), [&num_frames] {
			return std::make_pair(cv::Mat(num_frames, 6, CV_64F), std::size_t(0));
		});


		cv::Mat frame;

		for (std::size_t cur_frame = 0; video_cap.read(frame); ++cur_frame) {
			aruco::DetectionResult result = detector.detect(frame);
			for (unsigned int i=0; i<detector.get_boards().size(); ++i) {
				if (result.detections[i].num_inliers > 0) {
					cv::Mat tf(1, 6, CV_64F);
					cv::Mat(1, 3, cv::DataType<tf2Scalar>::depth, &*(result.detections[i].transform.getOrigin())).copyTo(tf.colRange(0,3));

					cv::Mat R(3, 3, CV_64F);
					cv::Mat rvec;
					cv::Mat(1, 3, cv::DataType<tf2Scalar>::depth, &*(result.detections[i].transform.getBasis()[0])).copyTo(R.row(0));
					cv::Mat(1, 3, cv::DataType<tf2Scalar>::depth, &*(result.detections[i].transform.getBasis()[1])).copyTo(R.row(1));
					cv::Mat(1, 3, cv::DataType<tf2Scalar>::depth, &*(result.detections[i].transform.getBasis()[2])).copyTo(R.row(2));
					cv::Rodrigues(R, rvec);
					rvec.reshape(1, 1).copyTo(tf.colRange(3, 6));

					tf.copyTo(estimates[i].first.row(estimates[i].second++));
				}
			}
		}

		std::vector<cv::Mat> covariances;
		std::transform(estimates.begin(), estimates.end(), std::back_inserter(covariances),
				[] (std::pair<cv::Mat, std::size_t> & estimate) {
			estimate.first.resize(estimate.second);
			ROS_INFO_STREAM("tfs: " << estimate.first);
			cv::Mat mean, cov;
			cv::calcCovarMatrix(estimate.first, cov, mean, cv::CovarFlags::COVAR_ROWS + cv::CovarFlags::COVAR_NORMAL + cv::CovarFlags::COVAR_SCALE);
			return cov;
		} );
		return covariances;
	}



private:
	aruco::ArucoDetector detector;

};


template<class T>
T get_param(ros::NodeHandle & nh, std::string const & name, T const & def) {
	T t;
	nh.param<T>(name, t, def);
	return t;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "covariance_estimator");

	ros::NodeHandle nh("~");
	aruco::ArucoDetector aruco_detector(aruco::ArucoBoard::load_from_params(nh), false);
	aruco_detector.update_camera_info(
			camera_info_manager::CameraInfoManager(nh,
					get_param<std::string>(nh, "camera_name", ""),
					get_param<std::string>(nh, "camera_info_url", "")).getCameraInfo());

	CovarianceEstimator estimator(std::move(aruco_detector));

	std::string const video_dir = get_param<std::string>(nh, "video_dir", "");
	if (video_dir.empty()) {
		throw std::runtime_error("Missing required param video_dir");
	}

	std::for_each(boost::filesystem::directory_iterator(video_dir), boost::filesystem::directory_iterator(),
			[&estimator] (boost::filesystem::path const & path) {
		std::vector<cv::Mat> cov = estimator.process_stationary_video(path.native());
		std::cout << path << ": \n";
		std::copy(cov.begin(), cov.end(), std::ostream_iterator<cv::Mat>(std::cout, "\n"));
	});
}
