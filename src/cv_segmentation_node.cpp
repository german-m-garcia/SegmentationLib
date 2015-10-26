#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

// cv_bridge
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

/// Global Variables
const int sp_slider_max = 80;
int sp,sp_slider;

const int sr_slider_max = 80;
int sr,sr_slider;

ros::Publisher segments_pub;



// set all mat values at given channel to given value
void setChannel(Mat &mat, unsigned int channel, unsigned char value)
{
    // make sure have enough channels
    if (mat.channels() < channel + 1)
        return;

    const int cols = mat.cols;
    const int step = mat.channels();
    const int rows = mat.rows;
    for (int y = 0; y < rows; y++) {
        // get pointer to the first byte to be changed in this row
        unsigned char *p_row = mat.ptr(y) + channel; 
        unsigned char *row_end = p_row + cols*step;
        for (; p_row != row_end; p_row += step)
            *p_row = value;
    }
}

static void hsvExperiments(Mat& src,Mat& dst){
	Mat hsv;
	cvtColor(src, hsv, CV_BGR2HLS);
	setChannel(hsv,1,250);
	cvtColor(hsv, dst, CV_HLS2BGR);
	imshow("HSV normalised",dst);
}



//This colors the segmentations
static void floodFillPostprocess( Mat& img, const Scalar& colorDiff=Scalar::all(1) )
{
    CV_Assert( !img.empty() );
    RNG rng = theRNG();
    Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
    for( int y = 0; y < img.rows; y++ )
    {
        for( int x = 0; x < img.cols; x++ )
        {
            if( mask.at<uchar>(y+1, x+1) == 0 )
            {
                Scalar newVal( rng(256), rng(256), rng(256) );
                floodFill( img, mask, Point(x,y), newVal, 0, colorDiff, colorDiff );
            }
        }
    }
}

static void imgCallback(const sensor_msgs::ImageConstPtr& kinect_image)
{
  ROS_INFO("Incoming image");
  
	// convert message from ROS to openCV
	cv_bridge::CvImagePtr cv_imgptr;
	try
	{
		cv_imgptr = cv_bridge::toCvCopy(kinect_image, enc::TYPE_8UC4);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// convert to Mat
	
	Mat img,img2;
	Mat& ref = cv_imgptr->image;
	cvtColor(ref, img, CV_BGRA2BGR);       // 1. change the number of channels
	
	//hsvExperiments(img,img);
		

	Mat res;
	bilateralFilter ( img, img2, 15, 30, 50 );
	pyrMeanShiftFiltering( img2, ref, sp, sr, 1,cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,10,1));	
	floodFillPostprocess(ref,Scalar::all(2));
	//imshow( "Bilateral", img2 );	
	//imshow( "Meanshift", res );	
	//imshow("RGB",img);
	//waitKey(1);
	
	//put back into a sensor_msg
	/*cvtColor(res, res, CV_BGR2BGRA);
	try
	{
		cv_imgptr = cv_bridge::toImageMsg(sensor_msgs::Image& ros_image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}*/
	
  
}

void on_trackbar( int, void* )
{
	sp=sp_slider;
	sr=sr_slider;
	ROS_INFO("sp=%d sr=$%d",sp,sr);
}



static void initWindows(){
	 
     namedWindow("Meanshift", 1);
	 /// Create Trackbars
	 char TrackbarName[50];
	 sprintf( TrackbarName, "Sp x %d", sp_slider_max );
     sp = sp_slider = 10;
	 createTrackbar( TrackbarName, "Meanshift", &sp_slider, sp_slider_max, on_trackbar );
	 
	 sprintf( TrackbarName, "Sr x %d", sr_slider_max );
     sr = sr_slider = 15;
	 createTrackbar( TrackbarName, "Meanshift", &sr_slider, sr_slider_max, on_trackbar );
	 
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "segmentation");
	ros::NodeHandle n;
	
	ros::Rate loop_rate(100);
	initWindows();
	//have to subscribe to 
	ros::Subscriber sub = n.subscribe("/workspace_camera_right/rgb/image_raw", 1, imgCallback);
	//segments_pub = n.advertise<std_msgs::String>("image_segments", 100);
	
	

	
	
	while (n.ok()) {
		
		ros::spinOnce();
	}

}




