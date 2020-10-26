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
  // read a image and a pcd
  cv::Mat image_origin = cv::imread("/media/weining/Elements/cali/2020-09-02-11-44-02/right_usb_cam/1599018242717845172.jpg");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_withoutNAN(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> ("/media/weining/Elements/cali/2020-09-02-11-44-02/lidar/1599018242.718678000.pcd", *cloud_origin);
  std::vector<cv::Point3f> pts_3d;
  for (size_t i = 0; i < cloud_origin->size(); ++i)
  {
    pcl::PointXYZI point_3d = cloud_origin->points[i];
    if (point_3d.x > -10 && point_3d.x < 10 && point_3d.y > -10 && point_3d.y < 10)
    {
      pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
    }
  }

  // read calibration parameter
  double fx = 1.9627131246784029e+03, fy = 9.7515971493690063e+02;
  double cx = 1.9682008536409182e+03, cy = 5.4221079144097973e+02;
  double k1 = -5.8117640997518183e-01, k2 = 3.6604723385540339e-01, k3 = 1.8051326156641742e-03;
  double p1 = -4.4527176642138689e-03, p2 = -5.6322522686054435e-02;
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  cv::Mat distortion_coeff = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3); 
  cv::Mat r_vec;
  cv::Mat t_vec = (cv::Mat_<double>(3, 1) << 2.2921520534373985e-01,-2.9521706502045625e-01, -1.5647908848559949e-01);
  cv::Mat R = (cv::Mat_<double>(3, 3)<<-7.0779903194427174e-02, -3.7849245198657733e-02,9.9677361519137297e-01, 
       -9.9745937080164426e-01, 1.0762875640075276e-02,-7.0419912723205857e-02, 
       -8.0628099180842794e-03, -9.9922549764597579e-01,-3.8514879597068452e-02);
  R = R.t();
  t_vec = -R * t_vec;
  cv::Rodrigues(R,r_vec); 
  // project 3d-points into image view
  std::vector<cv::Point2f> pts_2d;
  cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff, pts_2d);
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

  cv::imshow("origin image", image_origin);
  cv::imshow("project image", image_project);
  cv::imwrite("/media/weining/Elements/cali/2020-09-02-11-44-02/image_origin.jpg", image_origin);
  cv::imwrite("/media/weining/Elements/cali/2020-09-02-11-44-02/image_project.jpg", image_project);
  cv::waitKey(50000);

  return 0;
}