#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <highgui.h>
//#include "opencv2/videoio.hpp"
int main( int argc, char **argv )
{
  ros::init( argc, argv, "read_video_stream" );

  ros::NodeHandle n;

  // Open camera with CAMERA_INDEX (webcam is typically #0).
  //  const std::string videoStreamAddress = "http://10.10.10.13/videostream.cgi?loginuse=admin&amp;loginpas=&.mjpg";
  //  const std::string videoStreamAddress = "http://admin:@10.10.10.13/mjpeg.cgi?user=admin&password=admin:&channel=0&.mjpg";
  //const std::string videoStreamAddress = "http://10.10.10.111:81/videostream.cgi?loginuse=admin&amp;loginpas=12345&.mjpg";
  //const std::string videoStreamAddress = "http://10.10.10.13:34567/videostream.cgi?loginuse=admin&amp;loginpas=&.mjpg";
  const std::string videoStreamAddress = "rtsp://admin:@10.10.10.13/user=admin_password=_channel=1_stream=0.sdp";
  //const std::string videoStreamAddress = "http://192.168.11.20:81/video?x.mjpeg";
  cv::VideoCapture capture(videoStreamAddress );
  if(!capture.isOpened() )
  {
    ROS_ERROR_STREAM(
      "Failed to open camera with address " << videoStreamAddress  << "!"
    );
    ros::shutdown();
  }

  double dWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
  double dHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

  ROS_INFO_ONCE(" Frame Size: Width=%f, Height=%f", dWidth, dHeight);

  image_transport::ImageTransport it( n );
  image_transport::Publisher pub_image = it.advertise( "camera", 1 );

  cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();
  frame->encoding = sensor_msgs::image_encodings::BGR8;

  while( ros::ok() ) {
    capture >> frame->image;

    if( frame->image.empty() )
    {
      ROS_ERROR_STREAM( "Failed to capture frame!" );
      ros::shutdown();
    }

    frame->header.stamp = ros::Time::now();
    pub_image.publish( frame->toImageMsg() );

    cv::waitKey( 3 );

    ros::spinOnce();
  }

  capture.release();


  return EXIT_SUCCESS;
}
