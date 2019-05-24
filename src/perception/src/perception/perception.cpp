#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include "AprilTags/TagFamily.h"
// #include "AprilTags/TagDetector.h"
// #include "AprilTags/Tag36h11.h"
// #include "AprilTags/Tag36h11_other.h"
#include "perception/Observation.h"
#include "perception/perception.h"
using namespace std;
Perception::
Perception() : // tag_detector( AprilTags::tagCodes36h11 ),
tagsize( 0.175 ),
observations() {
}
Perception::
~Perception() {
}

void
Perception::
handleDepthImage( const sensor_msgs::Image::ConstPtr& msg ){
	cout << "got depth image" << endl;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::TYPE_16UC1 );
	cv::imshow( "Depth Image", cv_image->image );
	cv::waitKey(3);
	return;
}
ostream&
operator<<( ostream& out,
const Perception& other ){
	return out;
}
