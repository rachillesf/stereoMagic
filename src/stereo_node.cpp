#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

using namespace message_filters;
using namespace sensor_msgs;
using namespace cv;
using namespace cv::ximgproc;
using namespace std;


bool show_PointCloud = 0;

image_transport::Publisher pub;
ros::Publisher pcpub;
//global variable initialization
double vis_mult = 3.0;
int wsize = 3;
int max_disp = 16 * 8;
double lambda = 5000.0;
double sigma = 1.0;

Mat left_for_matcher,right_for_matcher;
Mat left_disp, right_disp;
Mat filtered_disp;

Rect ROI ;
Ptr<DisparityWLSFilter> wls_filter;

Mat filtered_disp_vis;


void compute_stereo(Mat& imL, Mat& imR)
{
  Mat conf_map = Mat(imL.rows,imL.cols,CV_8U);
  conf_map = Scalar(255);

  // downsample images
  max_disp/=2;
  if(max_disp%16 != 0) max_disp += 16-(max_disp%16);
  resize(imL, left_for_matcher,Size(),0.5,0.5);
  resize(imR, right_for_matcher,Size(),0.5,0.5);

  //compute disparity
  Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
  left_matcher->setP1(24*wsize*wsize);
  left_matcher->setP2(96*wsize*wsize);
  left_matcher->setPreFilterCap(63);
  left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
  wls_filter = createDisparityWLSFilter(left_matcher);
  Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

  left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
  right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);

  //filter
  wls_filter->setLambda(lambda);
  wls_filter->setSigmaColor(sigma);
  wls_filter->filter(left_disp,imL,filtered_disp,right_disp);
  conf_map = wls_filter->getConfidenceMap();
  ROI = wls_filter->getROI();

  //visualization
  getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
  double w = imR.cols;
  double  h = imR.rows;
  double f = 0.8*w ;

  // Q matrix (guess until we can do the correct calib process)
  Mat Q = Mat(4,4, CV_64F, double(0));
  Q.at<double>(0,0) = 1.0;
  Q.at<double>(0,3) = -0.5*w;
  Q.at<double>(1,1) = -1.0;
  Q.at<double>(1,3) = 0.5*h;
  Q.at<double>(2,3) = -f;
  Q.at<double>(3,2) = 0.5;

  // create pointcloud
  Mat imgDisparity8U(filtered_disp);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new  pcl::PointCloud<pcl::PointXYZRGB>());
  Mat xyz;
  reprojectImageTo3D(imgDisparity8U, xyz, Q, true);
  pointcloud->width = static_cast<uint32_t>(imgDisparity8U.cols);
  pointcloud->height = static_cast<uint32_t>(imgDisparity8U.rows);
  pointcloud->is_dense = false;
  pcl::PointXYZRGB point;
  for (int i = 0; i < imgDisparity8U.rows; ++i)
  {
    uchar* rgb_ptr = imL.ptr<uchar>(i);
    uchar* imgDisparity8U_ptr = imgDisparity8U.ptr<uchar>(i);
    double* xyz_ptr = xyz.ptr<double>(i);

    for (int j = 0; j < imgDisparity8U.cols; ++j)
    {

      uchar d = imgDisparity8U_ptr[j];
      //if (d == 0) continue;
      Point3f p = xyz.at<Point3f>(i, j);

      double radius = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
      if(radius < 8)
      {
        point.z = p.z;   // I have also tried p.z/16
        point.x = p.x;
        point.y = p.y;

        point.b = 255;//rgb_ptr[3 * j];
        point.g = 0;//rgb_ptr[3 * j + 1];
        point.r = 0;//rgb_ptr[3 * j + 2];
        pointcloud->points.push_back(point);
      }
      else
      {
        point.z = 0.0;   // I have also tried p.z/16
        point.x = 0.0;
        point.y = 0.0;

        point.b = 0;//rgb_ptr[3 * j];
        point.g = 0;//rgb_ptr[3 * j + 1];
        point.r = 0;//rgb_ptr[3 * j + 2];
        pointcloud->points.push_back(point);
      }
    }
  }

  // voxel grid filter
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (pointcloud);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (*cloud_filtered);


  //outliner removal filter
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new  pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor1;
  sor1.setInputCloud (cloud_filtered);
  sor1.setMeanK (50);
  sor1.setStddevMulThresh (0.01);
  sor1.filter (*cloud_filtered2);

   // Convert to ROS data type
   sensor_msgs::PointCloud2 output;
   pcl:: toROSMsg(*cloud_filtered2,output);
   // Publish the data
   pcpub.publish(output);

   if(show_PointCloud)
   {
     pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
     viewer.showCloud(cloud_filtered2);
     while (!viewer.wasStopped ())
     {
     }
   }

 }



void callback(const ImageConstPtr& left, const ImageConstPtr& right) {

  cv_bridge::CvImagePtr cv_left;
  try
    {
      cv_left = cv_bridge::toCvCopy(left);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_bridge::CvImagePtr cv_right;
  try
      {
        cv_right = cv_bridge::toCvCopy(right);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }


   compute_stereo(cv_left->image,cv_right->image);
   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8",filtered_disp_vis).toImageMsg();
   pub.publish(msg);

   //cv::imshow(OPENCV_WINDOW, filtered_disp_vis);
   //cv::waitKey(3);


}

int main(int argc, char **argv) {


//  namedWindow("filtered disparity", WINDOW_AUTOSIZE);
  ros::init(argc, argv, "stereo_node");
	ros::NodeHandle nh;
  pcpub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/pointcloud", 1);
  image_transport::ImageTransport it(nh);
  pub = it.advertise("/camera/depth/image", 1);
	message_filters::Subscriber<Image> left_sub(nh, "camera/left/ret", 1);
	message_filters::Subscriber<Image> right_sub(nh, "camera/right/ret", 1);

	TimeSynchronizer<Image, Image> sync(left_sub, right_sub, 10);
	sync.registerCallback(boost::bind(&callback, _1, _2));




	ros::spin();
	return 0;
}
