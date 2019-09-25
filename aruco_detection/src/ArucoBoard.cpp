/*
 * ArucoDetection.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: reubena
 */

#include "../include/aruco_detection/ArucoBoard.hpp"

/* TODO(pkoppol): Example of forgotten comment. */

namespace aruco {

const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME>
    aruco_dict_lookup = {
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
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}};

ArucoBoard::DictType ArucoBoard::getPredefinedDictionaryFromName(
    std::string const& name) {
  try {
    return cv::aruco::getPredefinedDictionary(
        aruco::aruco_dict_lookup.at(name));
  } catch (std::out_of_range& e) {
    throw std::out_of_range(std::string("No such dictionary name: ") + name);
  }
}

ArucoBoard ArucoBoard::load_from_param_ns(ros::NodeHandle const& nh,
                                          std::string const& name,
                                          std::string const& prefix) {
  ArucoBoard board;

  std::string dict;
  int marker_x, marker_y, start_id;
  double marker_len, marker_sep;

  std::string const full_prefix = prefix + "/" + name + "/";

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

#ifdef CV_ARUCO_ADV
  board.board = cv::aruco::GridBoard::create(
      marker_x, marker_y, marker_len, marker_sep,
      ArucoBoard::getPredefinedDictionaryFromName(dict), start_id);
#else   // CV_ARUCO_ADV
  if (start_id != 0) {
    ROS_WARN("OpenCV version does not allow start_id!");
  } else {
    board.board = cv::aruco::GridBoard::create(
        marker_x, marker_y, marker_len, marker_sep,
        ArucoBoard::getPredefinedDictionaryFromName(dict));
  }
#endif  // CV_ARUCO_ADV
  return board;
}

std::vector<ArucoBoard> ArucoBoard::load_from_params(
    ros::NodeHandle const& nh) {
  XmlRpc::XmlRpcValue topics;
  typedef std::pair<std::string, XmlRpc::XmlRpcValue> PairType;
  nh.getParam("board", topics);

  std::vector<ArucoBoard> boards;
  std::transform(topics.begin(), topics.end(), std::back_inserter(boards),
                 [&nh](PairType const& pair) {
                   return ArucoBoard::load_from_param_ns(nh, pair.first,
                                                         "board");
                 });

  return boards;
}

}  // namespace aruco
