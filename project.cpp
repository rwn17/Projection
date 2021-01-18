#include"eigen3/Eigen/Dense"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

#include <iostream>

int radius = 1;
int range = 20;

int main(int argc, char** argv)
{
  float max_dist = 0;
  float min_dist = 0;
  cv::FileStorage file_settings("/home/will/Desktop/Thesis/utils/projection/cali.yaml", cv::FileStorage::READ);
  cv::Mat camera_matrix,distortion_coeff,extrin_matrix;
  cv::Mat R,t_vec;
  std::string pic_path,pcd_path,output_path;
  // read yaml
  file_settings["CameraMat"] >> camera_matrix;
  file_settings["DistCoeff"] >> distortion_coeff;
  file_settings["CameraExtrinsicMat"]>>extrin_matrix;
  file_settings["PicPath"] >> pic_path;
  file_settings["PcdPath"] >> pcd_path;
  file_settings["OutputPath"] >> output_path;
  cv::Mat image_origin = cv::imread(pic_path);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_rotation(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_path, *cloud_origin);
  //read R t from Extrin_matrix
  R = extrin_matrix(cv::Range(0,3),cv::Range(0,3));
  t_vec = extrin_matrix(cv::Range(0,3),cv::Range(3,4));
  R = R.t();
  t_vec = -R * t_vec;
  //rotate the cloud
  Eigen::Matrix3f init_rotation;
  cv::cv2eigen(R, init_rotation);
  Eigen::Translation3f init_translation(t_vec.at<double>(0,0),
                                        t_vec.at<double>(1,0),
                                        t_vec.at<double>(2,0));
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
  std::cout<<"SE(3) is:"<<std::endl<<init_guess<<std::endl;
  //filter
  std::vector<cv::Point3f> pts_3d;
  std::vector<cv::Point3f> rotation_pts_3d;
  pcl::transformPointCloud (*cloud_origin, *cloud_after_rotation, init_guess);
  for (size_t i = 0; i < cloud_origin->size(); ++i)
  {
    pcl::PointXYZI point_3d = cloud_origin->points[i]; // for filter
    pcl::PointXYZI point_3d_rotation = cloud_after_rotation->points[i]; // for rotation(depth color)
    if (point_3d.x > -range && point_3d.x < range && point_3d.y > -range && point_3d.y < range && point_3d_rotation.z > 0)
    {
      pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
      rotation_pts_3d.emplace_back(cv::Point3f(point_3d_rotation.x,point_3d_rotation.y,point_3d_rotation.z));
      double this_dist = point_3d_rotation.z;
      max_dist = (this_dist > max_dist)? this_dist: max_dist;
      min_dist = (this_dist < min_dist)? this_dist: min_dist;
    }
  }

  // project 3d-points into image view
  std::vector<cv::Point2f> pts_2d;
  cv::projectPoints(pts_3d, R, t_vec, camera_matrix, distortion_coeff, pts_2d);
  cv::Mat image_project = image_origin.clone();
  int image_rows = image_origin.rows;
  int image_cols = image_origin.cols;
  std::cout<<"max dist is :"<<max_dist<<std::endl;
  std::cout<<"min dist is :"<<min_dist<<std::endl;
  for (size_t i = 0; i < pts_2d.size(); ++i)
  {
    cv::Point2f point_2d = pts_2d[i];
    cv::Point3f point_3d = rotation_pts_3d[i];
    float this_dist = point_3d.z;
    double ratio = (this_dist - min_dist) / (max_dist - min_dist);
    if (point_2d.x > 0 && point_2d.x < image_cols && point_2d.y > 0 && point_2d.y < image_rows)
    {
      cv::Scalar color(255-(int)(ratio * 255), 0, (int)(ratio * 255));
      cv::rectangle(image_project,cv::Point(point_2d.x + radius,point_2d.y + radius),
                                  cv::Point(point_2d.x - radius,point_2d.y - radius),
                                  color,
                                  1);
    }
  }
  cv::imshow("project image2", image_project);
  cv::imwrite(output_path, image_project);
  cv::waitKey(100000);

  return 0;
}