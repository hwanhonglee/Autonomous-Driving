// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <tag_based_pnp_calibrator/math.hpp>

#include <vector>

double getReprojectionError(
  const std::vector<cv::Point2d> & points1, const std::vector<cv::Point2d> & points2)
{
  double error = 0.0;

  for (std::size_t i = 0; i < points1.size(); i++) {
    error += cv::norm(points1[i] - points2[i]);
  }

  return error / points1.size();
}

double getReprojectionError(
  const std::vector<cv::Point3d> & object_points, const std::vector<cv::Point2d> & image_points,
  const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix,
  const cv::Matx33d & camera_matrix, const cv::Mat_<double> & distortion_coeffs)
{
  std::vector<cv::Point2d> projected_points;

  cv::Matx31d rvec;
  cv::Rodrigues(rotation_matrix, rvec);

  cv::projectPoints(
    object_points, rvec, translation_vector, camera_matrix, distortion_coeffs, projected_points);

  return getReprojectionError(image_points, projected_points);
}

double getReprojectionError(
  const std::vector<cv::Point3d> & object_points, const std::vector<cv::Point2d> & image_points,
  const cv::Matx31d & translation_vector, const cv::Matx31d & rotation_vector,
  const cv::Matx33d & camera_matrix, const cv::Mat_<double> & distortion_coeffs)
{
  std::vector<cv::Point2d> projected_points;

  cv::projectPoints(
    object_points, rotation_vector, translation_vector, camera_matrix, distortion_coeffs,
    projected_points);

  return getReprojectionError(image_points, projected_points);
}
