#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  // read calibration parameter
  double fx = 1.9627131246784029e+03, fy =1.9682008536409182e+03 ;
  double cx = 9.7515971493690063e+02, cy = 5.4221079144097973e+02;
  double k1 = -5.8117640997518183e-01, k2 = 3.6604723385540339e-01, k3 = 1.8051326156641742e-03;
  double p1 = -4.4527176642138689e-03, p2 = -5.6322522686054435e-02;
  Matrix3f camera_matrix;
  //Matrix3d distortion_coeff; 
  Vector3f r_vec;
  Vector3f t_vec;
  Matrix3f R;
  camera_matrix<<fx,0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  R<< -7.0779903194427174e-02, -3.7849245198657733e-02,9.9677361519137297e-01, 
       -9.9745937080164426e-01, 1.0762875640075276e-02,-7.0419912723205857e-02, 
       -8.0628099180842794e-03, -9.9922549764597579e-01,-3.8514879597068452e-02;
  t_vec<<2.2921520534373985e-01,-2.9521706502045625e-01, -1.5647908848559949e-01;
  /*
  t_vec = -R * t_vec;
  Vector3f origin_point(2,-1,0);
  Vector3f projected_point = camera_matrix * (R.transpose() * (origin_point  t_vec));
  std::cout<<"x is:"<<projected_point.x()/projected_point.z()<<"y is :"<<projected_point.y()/projected_point.z()<<endl;
  std::cout<<"R is :"<<R<<endl;;
  std::cout<<"t_vec is :"<<t_vec<<endl;
  */
  // read a image and a pcd
  cv::Mat image_origin = cv::imread("/media/weining/Elements/cali/2020-09-02-11-44-02/right_usb_cam/1599018242717845172.jpg");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_withoutNAN(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> ("/media/weining/Elements/cali/2020-09-02-11-44-02/lidar/1599018242.718678000.pcd", *cloud_origin);
  std::vector<cv::Point3f> pts_3d;
  std::vector<cv::Point2f> pts_2d;
  // project 3d-points into image view
  for (size_t i = 0; i < cloud_origin->size(); ++i)
  {
    pcl::PointXYZI point_3d = cloud_origin->points[i];
    if (point_3d.x > -5 && point_3d.x < 5 && point_3d.y > -5 && point_3d.y < 5)
    {
      Vector3f origin_point(point_3d.x,point_3d.y, point_3d.z);
      Vector3f projected_point = camera_matrix*(R.transpose() * ( origin_point - t_vec));
      //cout<<"projected x:"<<projected_point.x()<<"projected y :"<< projected_point.y()<<endl;
      pts_2d.emplace_back(cv::Point2f(projected_point.x()/projected_point.z(), projected_point.y()/projected_point.z()));
    }
  }
  std::cout<<"pt_2d size is :"<<pts_2d.size()<<endl;
  cv::Mat image_project = image_origin.clone();
  int image_rows = image_origin.rows;
  int image_cols = image_origin.cols;
  int counter = 0;
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
      cout<<"projected x:"<<point_2d.x<<"projected y :"<<point_2d.y<<endl;
      counter ++;
    }
    
    if (point_2d.x > 0 && point_2d.x < image_cols && point_2d.y > 0 && point_2d.y < image_rows)
    {
      image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] = 0;
      image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] = 0;
      image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[2] = 255;
      counter ++;
    } 
    else
    {
      continue;
    }  
  }
  std::cout<< "the counter is:"<<counter<<endl;
  cv::imshow("origin image", image_origin);
  cv::imshow("project image", image_project);
  cv::imwrite("/media/weining/Elements/cali/2020-09-02-11-44-02/image_origin.jpg", image_origin);
  cv::imwrite("/media/weining/Elements/cali/2020-09-02-11-44-02/image_project.jpg", image_project);
  cvWaitKey(500000);
  return 0;
}