#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include <iostream>

int main(int argc, char** argv)
{
  
  cv::FileStorage file_settings("/home/weining/summer_intern/project/cali.yaml", cv::FileStorage::READ);
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
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_withoutNAN(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_path, *cloud_origin);
  //read R t from Extrin_matrix
  R = extrin_matrix(cv::Range(0,3),cv::Range(0,3));
  t_vec = extrin_matrix(cv::Range(0,3),cv::Range(3,4));
  R = R.t();
  t_vec = -R * t_vec; 
  
  //filter
  std::vector<cv::Point3f> pts_3d;
  for (size_t i = 0; i < cloud_origin->size(); ++i)
  {
    pcl::PointXYZI point_3d = cloud_origin->points[i];
    if (point_3d.x > -5 && point_3d.x < 5 && point_3d.y > -5 && point_3d.y < 5)
    {
      pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
    }
  }


  /*read parameter by hand
  double fx = 1.9607785586746156e+03, fy =1.9639001481653022e+03;
  double cx = 9.7332324904180166e+02, cy = 5.5391985294502831e+02;
  double k1 =  -5.8994561002105106e-01, k2 = 4.2660715173492120e-01, k3 = -2.6257685901504318e-01;
  double p1 =  -3.1956656332285339e-04, p2 = -3.8298406550321359e-03;
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  cv::Mat distortion_coeff = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3); 
  */

  /*calculate R2*R1.inv()
  cv::Mat R1 = (cv::Mat_<double>(4, 4) << 0.9992, 0.0317,0.0251,0.3000,
       0.0235,0.051,-0.9984,0.1100,
      -0.0330,0.9982,0.0502,-0.85,
      0,0,0,1);
  cv::Mat R2 = (cv::Mat_<double>(4, 4) << 0.0109, -0.9995,-0.0311,0.3500,
      0.9997,0.0102,0.0206,0,
      -0.0202,-0.0313,0.9993,-0.13,
      0,0,0,1);
  cv::Mat R1_inv;
  cv::invert(R1,R1_inv);
  RCL = R2 * R1_inv;
  std::cout<<RCL<<std::endl;
 */

  // project 3d-points into image view
  std::vector<cv::Point2f> pts_2d;
  cv::projectPoints(pts_3d, R, t_vec, camera_matrix, distortion_coeff, pts_2d);
  cv::Mat image_project = image_origin.clone();
  int image_rows = image_origin.rows;
  int image_cols = image_origin.cols;

  for (size_t i = 0; i < pts_2d.size(); ++i)
  {
    cv::Point2f point_2d = pts_2d[i];
    if (point_2d.x < 0 || point_2d.x > image_cols || point_2d.y < 0 || point_2d.y > image_rows)
    {
      continue;
    }
    else
    {
      image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] = 0;
      image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] = 0;
      image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[2] = 255;
    }
    
    if (point_2d.x > 0 && point_2d.x < image_cols && point_2d.y > 0 && point_2d.y < image_rows)
    {
      image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] = 0;
      image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] = 0;
      image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[2] = 255;
    } 
    else
    {
      continue;
    }  
  }

  cv::imshow("project image", image_project);
  cv::imwrite(output_path, image_project);
  cv::waitKey(100000);

  return 0;
}