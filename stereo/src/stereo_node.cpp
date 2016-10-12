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

using namespace message_filters;
using namespace sensor_msgs;
using namespace cv;
using namespace cv::ximgproc;
using namespace std;

//static const std::string OPENCV_WINDOW = "Image window";

image_transport::Publisher pub;

//global variable initialization
double vis_mult = 3.0;
int wsize = 3;
int max_disp = 160;
double lambda = 8000.0;
double sigma = 2.0;

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


  image_transport::ImageTransport it(nh);
  pub = it.advertise("camera/depth_image", 1);
	message_filters::Subscriber<Image> left_sub(nh, "camera/left/ret", 1);
	message_filters::Subscriber<Image> right_sub(nh, "camera/right/ret", 1);

	TimeSynchronizer<Image, Image> sync(left_sub, right_sub, 10);
	sync.registerCallback(boost::bind(&callback, _1, _2));




	ros::spin();
	return 0;
}
