#include <ros/ros.h>

//#include <opencv2/core/core.hpp>

//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>
#include <vector>

#include <curl/curl.h>






//#include <thread>         // std::this_thread::sleep_for
//#include <chrono>         // std::chrono::seconds
#include <pthread.h>

using namespace cv;
using namespace std;

RNG rng(12345);







void bounding_box_detection(Mat& input_img, Mat& output_img, Mat& output_img2, bool& flag)
{
	Point center;
	Mat occludedSquare = input_img;
	//resize(occludedSquare, occludedSquare, Size(0, 0), 0.25, 0.25);
	//resize(input_img, output_img2, Size(0, 0), 0.25, 0.25);
	Mat occludedSquare8u;
	cvtColor(occludedSquare, occludedSquare8u, CV_BGR2GRAY);

	Mat thresh;
	threshold(occludedSquare8u, thresh, 20, 255.0, THRESH_BINARY_INV);
	//threshold(occludedSquare8u, thresh, 80, 80.0, THRESH_BINARY_INV);


	//morphological opening (removes small objects from the foreground)
	erode(thresh, thresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(thresh, thresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (removes small holes from the foreground)
	dilate(thresh, thresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(thresh, thresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


	GaussianBlur(thresh, thresh, Size(7, 7), 2.0, 2.0);




	imshow("thre", thresh);




	Mat edges;
	Canny(thresh, edges, 66.0, 133.0, 3);



	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
        //Detect contours
	findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));



	/// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	//vector<Point2f>center(contours.size());
	vector<float>radius(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		//minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
	}

	/// Draw polygonal contour + bonding rects + circles
	Mat drawing2 = Mat::zeros(thresh.size(), CV_8UC3);


        int detected_contour=0;
        float max_diagonal=0;
	Point top_left;
 	Point bottom_right;
	for (int i = 0; i< contours.size(); i++)
	{
		Point diff = boundRect[i].tl() - boundRect[i].br();
		double diagonal = sqrt(diff.x*diff.x + diff.y*diff.y);
		//cout << "diagonal" << endl;
		//cout << diagonal << endl;
		if (diagonal > 30) {
			detected_contour=detected_contour+1;

			if(diagonal>max_diagonal){
				max_diagonal=diagonal;	
				center = (boundRect[i].tl()+boundRect[i].br())*0.5;
				top_left=boundRect[i].tl();
				bottom_right=boundRect[i].br();

			}
                        


			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			drawContours(drawing2, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
			rectangle(drawing2, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);





			//rectangle(output_img2, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
  			

                       
			//center = (boundRect[i].tl()+boundRect[i].br())*0.5;

 			//cout<<"top left corner is"<<endl;
                        //cout<<boundRect[i].tl()<<endl;
			//cout<<"bottom right corner is"<<endl;
                        //cout<<boundRect[i].br()<<endl;
			//cout<<"center is"<<endl;
                        //cout<<center.x<<endl;
			//cout<<center.y<<endl;





			//if (center.x<input_img.cols/5*2) {
 			// on the left
			//cout<<"turn left"<<endl;
			//}
			//else if (center.x>input_img.cols/5*3) {
  			// On the right 
			//cout<<"turn right"<<endl;
			//}
			//else{
  			// At the middle
 			//cout<<"at the middle"<<endl;
			//}
		}
	}








	if(detected_contour>0){
        flag=true;
	}





	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(output_img2, top_left, bottom_right, color, 2, 8, 0);



	output_img = drawing2;
}


int main(int argc, char* argv[])
{
	bool Detection_Flag=false;

	//cv::Ptr<Tracker> tracker = Tracker::create("MIL");
	//VideoCapture video("C:/Users/Orson Lin/Desktop/kenland_farm_Feb24.mov");
	//VideoCapture video("/home/orson/Desktop/kenland_farm_Feb24.mov");
	//test url
	//char* linkChar="http://www.google.com";
        //ShellExecute(NULL, NULL, linkChar, NULL, NULL, SW_SHOWNORMAL);
  	//create a tracker object
 	//Ptr<Tracker> tracker = Tracker::create( "KCF" );






        string ip_right = "http://10.10.10.13/?command=ptz_req&req=start&param=directionleft&channel=1&stream=0";
	string ip_left = "http://10.10.10.13/?command=ptz_req&req=start&param=directionright&channel=1&stream=0";
	string ip_up = "http://10.10.10.13/?command=ptz_req&req=start&param=directiondown&channel=1&stream=0";
	string ip_down = "http://10.10.10.13/?command=ptz_req&req=start&param=directionup&channel=1&stream=0";
	string ip_zoomIn = "http://10.10.10.13/?command=ptz_req&req=start&param=zoomtile&channel=1&stream=0";
	string ip_zoomOut = "http://10.10.10.13/?command=ptz_req&req=start&param=zoomwide&channel=1&stream=0";
	string ip_stop = "http://10.10.10.13/?command=ptz_req&req=stop&param=directionright&channel=1&stream=0";



	CURL *curl;
  	CURLcode res;
 
  	curl = curl_easy_init();










    
        const std::string videoStreamAddress = "rtsp://admin:@10.10.10.13/user=admin_password=_channel=1_stream=0.sdp";
        VideoCapture video(videoStreamAddress );


        



        
















	// Check video is open
	if (!video.isOpened())
	{
		cout << "Could not read video file" << endl;
		return 1;
	}

	// Read first frame. 
	Mat frame;
	Mat frame2;
	video.read(frame);
	while (video.read(frame))
	{
		Mat boundingboxFrame;
		resize(frame, frame, Size(0, 0), 0.25, 0.25);

                //row    number is 270
                //column number is 480
// 		cout<<"row number"<<endl;
//		cout<<frame.rows<<endl;
//		cout<<"column number"<<endl;
//		cout<<frame.cols<<endl;




		//rotate the camera 
               	//response = ul.urlopen(ip_right)
		//rospy.sleep(4)



		//detect bounding box		
		bounding_box_detection(frame, boundingboxFrame, frame, Detection_Flag);
		imshow("cam2", frame);
		//imshow("cam1", boundingboxFrame);





		int k = waitKey(1);
		if (k == 27) break;


                if(!Detection_Flag){
 			if(curl) {
    			curl_easy_setopt(curl, CURLOPT_URL, "http://10.10.10.13/?command=ptz_req&req=start&param=directionleft&channel=1&stream=0");
      
    			/* Perform the request, res will get the return code */ 
    			res = curl_easy_perform(curl);

                	sleep(32*10/360);

               		curl_easy_setopt(curl, CURLOPT_URL, "http://10.10.10.13/?command=ptz_req&req=stop&param=directionright&channel=1&stream=0");

			res = curl_easy_perform(curl);

               		//std::this_thread::sleep_for (std::chrono::seconds(1));

    			/* Check for errors */ 
    			if(res != CURLE_OK)
      			fprintf(stderr, "curl_easy_perform() failed: %s\n",
              		curl_easy_strerror(res));
 
	    		/* always cleanup */ 
	    		curl_easy_cleanup(curl);
  	        	}
		}                
	        
	}

	return 0;
}
