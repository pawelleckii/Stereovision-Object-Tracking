//works with server_HSV_track app

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include <math.h>
#include <list>

#include <raspicam/raspicam_cv.h>
#include <iostream>
 
#include <arpa/inet.h>
#include <iostream>
#include <iomanip>

#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

using namespace std;
using namespace cv;

#define FLIP_IMAGE

#define PI 3.14159265

#define BASELINE 20.45 //distance between cameras in [cm]
#define FOV_DEG 62.2 //horizontal field of view [deg]
 
#define PORT "1500"//"3490" // the port client will be connecting to 
#define IP_ADDRESS "169.254.33.199"//"192.168.1.69" // "127.1.1.1"

#define MAXDATASIZE 1000000 // max number of bytes we can get at once 

#define BYTES_PER_PIXEL 3
#define IMG_FORMAT CV_8UC3

const int WINDOW_WIDTH = 320;
const int WINDOW_HEIGHT = 240;
const float VIEW_SCALE = 1;

const double FOV_RAD = FOV_DEG * PI / 180;
const int RECV_BUF_LEN = 16384;
    
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

void prepare_connection(int *sockfd){
	
	char buf[MAXDATASIZE];
    struct addrinfo hints, *servinfo, *p;
    int rv;
    char s[INET6_ADDRSTRLEN];

    /*if (argc != 2) {
        fprintf(stderr,"usage: client hostname\n");
        exit(1);
    }*/

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
	cout << "getaddrinfo\n";
    if ((rv = getaddrinfo(IP_ADDRESS, PORT, &hints, &servinfo)) != 0) { //argv[1] jaki #1 argument
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return;
    }
	cout << "finding connection\n";
	while(p==NULL){ //try to connect in loop
		// loop through all the results and connect to the first we can
		for(p = servinfo; p != NULL; p = p->ai_next) {
			if ((*sockfd = socket(p->ai_family, p->ai_socktype,
					p->ai_protocol)) == -1) {
				perror("client: socket");
				continue;
			}

			if (connect(*sockfd, p->ai_addr, p->ai_addrlen) == -1) {
				close(*sockfd);
				perror("client: connect");
				continue;
			}

			break;
		}
	}
	cout << "found connection\n";
    if (p == NULL) {
        fprintf(stderr, "client: failed to connect\n");
        return;
    }

	inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
			s, sizeof s);
	printf("client: connecting to %s\n", s);
	
	freeaddrinfo(servinfo); // all done with this structure
}

cv::Mat bytesToMat(unsigned char * bytes,int width,int height)
{
    cv::Mat image = cv::Mat(height,width,IMG_FORMAT ,bytes).clone(); // make a copy
    return image;
}

string make_string(const int param, const int length){
	std::ostringstream ostr;
	if(param < 0) ostr << '-';
	ostr << std::setfill('0') << std::setw(length) << (param < 0 ? -param : param);
	return ostr.str();
}

bool recv_all(int socket, void *buffer, size_t length)
{
    char *ptr = (char*) buffer;
    while (length > 0)
    {
        int i = recv(socket, ptr, length, 0);
        if (i < 1) return false;
        ptr += i;
        length -= i;
    }
    return true;
}

bool send_all(int socket, void *buffer, size_t length)
{
    char *ptr = (char*) buffer;
    while (length > 0)
    {
        int i = send(socket, ptr, length, 0);
        if (i < 1) return false;
        ptr += i;
        length -= i;
    }
    return true;
}

void mouse_callback(int event, int x, int y, int flag, void *param){

	vector<Point> *prev_points = (vector<Point>*) param;
	
	if(event == EVENT_LBUTTONDOWN){
		Point calibPoint = Point(x,y);				
		#ifdef FLIP_IMAGE
			calibPoint = Point(WINDOW_WIDTH - x, y);
		#endif
		for(int i = 0; i < 5; i++){
			prev_points->erase(prev_points->begin());
			prev_points->push_back(calibPoint);
		}
		cout << "\n\nPOINTS RESET\n\n\n";
	}
	
}

void filter_hsv(cv::Mat *mat, cv::Mat *hsv_mat, cv::Mat *thr, cv::Scalar hsv_min, cv::Scalar hsv_max, int erode_size, int dilate_size, int gauss_size){

	// Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
	cv::cvtColor(*mat, *hsv_mat, CV_RGB2HSV);
	// Filter out colors which are out of range.
	cv::inRange(*hsv_mat, hsv_min, hsv_max, *thr);

	// hough detector works better with some smoothing of the image
	cv::Mat erode_elem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_size, erode_size));
	cv::Mat dilate_elem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_size, dilate_size));
	cv::dilate(*thr, *thr, dilate_elem); //3
	//cv::smooth( thr, thr, CV_GAUSSIAN, gauss_size, gauss_size );//9 , 9
	cv::blur(*thr, *thr, Size(gauss_size, gauss_size));
	
	cv::erode(*thr, *thr, erode_elem); //3
}

double *scoreContourSize(vector<vector<Point>> &contours){

	double *sizeScore = new double[contours.size()];
	
	for(int i=0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i], false);  //  Find the area of contour			                
		sizeScore[i] = area;
	}
	return sizeScore;
}

Point predictPosition(vector<Point> &centerPositions){

	Point predictedNextPosition; 
	int numPositions = centerPositions.size();
	
	if (numPositions >= 5) {
		int sumOfXChanges = ((centerPositions[numPositions - 1].x - centerPositions[numPositions - 2].x) * 4) +
			((centerPositions[numPositions - 2].x - centerPositions[numPositions - 3].x) * 3) +
			((centerPositions[numPositions - 3].x - centerPositions[numPositions - 4].x) * 2) +
			((centerPositions[numPositions - 4].x - centerPositions[numPositions - 5].x) * 1);

		int deltaX = (int)std::round((float)sumOfXChanges / 10.0);

		int sumOfYChanges = ((centerPositions[numPositions - 1].y - centerPositions[numPositions - 2].y) * 4) +
			((centerPositions[numPositions - 2].y - centerPositions[numPositions - 3].y) * 3) +
			((centerPositions[numPositions - 3].y - centerPositions[numPositions - 4].y) * 2) +
			((centerPositions[numPositions - 4].y - centerPositions[numPositions - 5].y) * 1);

		int deltaY = (int)std::round((float)sumOfYChanges / 10.0);

		predictedNextPosition.x = centerPositions.back().x + deltaX;
		predictedNextPosition.y = centerPositions.back().y + deltaY;
	}
	return predictedNextPosition;
}

double *scoreContourPos(vector<vector<Point>> &contours, Point &predictedPos){
	
	double *posScore = new double[contours.size()];

	for(int i=0; i < contours.size(); i++)
	{
		Rect br = boundingRect(contours[i]);
		int cx = br.x+br.width/2;
		int cy = br.y+br.height/2;
		posScore[i] = -5 *sqrt( (cx - predictedPos.x)*(cx - predictedPos.x) + (cy - predictedPos.y)*(cy - predictedPos.y) );
		
	}
	return posScore;
}

cv::Rect getBestContour(vector<vector<Point>> &contours, double *contourScore){
	
	double bestScore = 0;
	int bestIndex = 0;
	for(int i=0; i<contours.size(); i++){
		if(contourScore[i] > bestScore){ 
			bestScore = contourScore[i];
			bestIndex = i;
		}
	}
	return boundingRect(contours[bestIndex]);
}

void drawContourScoreOnImage(cv::Mat &thr, vector<vector<Point>> &contours, double *posScore, double *sizeScore) {

    for (int i = 0; i < contours.size(); i++) {		
		
		int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
		double dblFontScale = 0.4;
		int intFontThickness = (int)std::round(dblFontScale * 1.0);
		Point upperPoint = Point(contours[i][0].x, contours[i][0].y - 10);
		
		cv::putText(thr, "S: " + std::to_string((int)sizeScore[i]), contours[i][0], intFontFace, dblFontScale, Scalar(255, 255, 255), intFontThickness);
		cv::putText(thr, "P: " + std::to_string((int)posScore[i]), upperPoint    , intFontFace, dblFontScale, Scalar(255, 255, 255), intFontThickness);
    }
}

//returns rectangle surrounding the best contour in terms of contourScore
cv::Rect find_object(cv::Mat *mat, cv::Mat *thr, vector<Point> &prev_positions){

	Rect bounding_rect;

	vector<vector<Point>> contours; // Vector for storing contour
	vector<Vec4i> hierarchy;

	cv::findContours(*thr, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image
	
	if(contours.size() > 0){

		double *sizeScore = scoreContourSize(contours);
		Point predictedPos = predictPosition(prev_positions);
		circle(*mat, predictedPos, 3, Scalar(0, 0, 255), CV_FILLED, 1,0);
		
		double *posScore = scoreContourPos(contours, predictedPos);
		
		double *contourScore = new double[contours.size()];
		
		for(int i=0; i<contours.size(); i++){
			contourScore[i] = sizeScore[i] + posScore[i];
		}
		drawContourScoreOnImage(*thr, contours, posScore, sizeScore);
		
		bounding_rect = getBestContour(contours, contourScore);
		
		delete[] sizeScore;
		delete[] posScore;
		delete[] contourScore;
	}
	
	return bounding_rect;
}

void draw_object(cv::Mat *mat, Rect bounding_rect, Point rect_center){
	
	rectangle(*mat, bounding_rect,  Scalar(255,255,255),1, 8,0);  
	circle(*mat, rect_center, 2, Scalar(255, 255, 255), CV_FILLED, 1,0);
}

Point3d calc_distance(Point leftPos, Point rightPos){
	
	double distance = (BASELINE * WINDOW_WIDTH) / (2 * tan(FOV_RAD/2)* (leftPos.x - rightPos.x) );
	
	double focal_len = WINDOW_WIDTH/(2* tan(FOV_RAD/2));	//265.235 in [px]
	
	double x1 = WINDOW_WIDTH/2 - leftPos.x;
	double y1 = WINDOW_HEIGHT/2 - leftPos.y;
	
	
	double X = -x1 * distance / focal_len;
	double Y = y1 * distance / focal_len;
	
	return Point3d(X, Y, distance);
	
}

int main(int argc, char* argv[])
{
 
	int sockfd, numbytes; 
    cout << "conn prep\n";
    cout<<"unsigned size: " << sizeof(unsigned char)<<endl;
    prepare_connection(&sockfd);
    cout << "conn granted\n";
	int bufsize = WINDOW_WIDTH * WINDOW_HEIGHT* BYTES_PER_PIXEL * sizeof(unsigned char);
	int offset = (bufsize/RECV_BUF_LEN) * RECV_BUF_LEN;
	unsigned char * buffer_ptr = new unsigned char[bufsize];
	
	
	raspicam::RaspiCam_Cv Camera;
    Camera.set( CV_CAP_PROP_FORMAT, IMG_FORMAT ); //CV_32FC3
    Camera.set( CV_CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH);
    Camera.set( CV_CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT);
	cout<<"Opening Camera..."<<endl;	
    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
    
    CvSize size = cvSize(WINDOW_WIDTH,WINDOW_HEIGHT);
 
    // Create a window in which the captured images will be presented
    cv::namedWindow( "Camera", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "After Color Filtering", CV_WINDOW_AUTOSIZE );
	//cv::namedWindow( "HSV", CV_WINDOW_AUTOSIZE );
	cv::namedWindow("Plan view", CV_WINDOW_AUTOSIZE );


	cv::namedWindow( "Camera2", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "After Color Filtering2", CV_WINDOW_AUTOSIZE );
    
    //Detect light green ball
    int hl = 32, hu = 81, sl = 0, su = 255, vl = 0, vu = 255;
 
	createTrackbar("Hmin", "After Color Filtering", &hl,180);
	createTrackbar("Hmax", "After Color Filtering", &hu, 180);
	createTrackbar("Smin", "After Color Filtering", &sl,255);
	createTrackbar("Smax", "After Color Filtering", &su, 255);
	createTrackbar("Vmin", "After Color Filtering", &vl,255);
	createTrackbar("Vmax", "After Color Filtering", &vu, 255);
 
	//int cannyTres= 40, accuTres=10, minRad = 20, maxRad = 400;
	int dilate_size = 3, erode_size = 3, gauss_size = 9;
	createTrackbar("Dilate", "Camera", &dilate_size,20);
	createTrackbar("Erode", "Camera", &erode_size, 20);
	//createTrackbar("Gauss", "Camera", &gauss_size,20);
	
	cv::Mat mat; //image from camera on this device matrix
	cv::Mat hsv_mat; //hsv transform of image matrix
	cv::Mat thr; //thresholded object image matrix 
	
	cv::Mat mat2;
	cv::Mat hsv_mat2;
	cv::Mat thr2;
	
	std::vector<Point> prev_positions;
	for(int i=0; i<5; i++){
		prev_positions.push_back(Point(WINDOW_WIDTH/2, WINDOW_HEIGHT/2));
	}
	std::vector<Point> prev_positions2;
	for(int i=0; i<5; i++){
		prev_positions2.push_back(Point(WINDOW_WIDTH/2, WINDOW_HEIGHT/2));
	}
	
	cv::setMouseCallback("Camera", mouse_callback, &prev_positions);
	cv::setMouseCallback("Camera2", mouse_callback, &prev_positions2);
	
	//bool show_contours = true;
	//Rect search_area = Rect(WINDOW_WIDTH/2 - WINDOW_WIDTH/8, WINDOW_HEIGHT/2 - WINDOW_HEIGHT/8, WINDOW_WIDTH/4, WINDOW_HEIGHT/4);
    
    while( 1 )
    {
		
		//auto params = make_string(hl, 3) + make_string(hu, 3) +make_string(sl, 3) +make_string(su, 3) +make_string(vl, 3) +make_string(vu, 3) + make_string(dilate_size, 2) +make_string(erode_size, 2);
		//send(sockfd, "P", 1, 0);
		//send_all(sockfd, &params, 22);
        // Get one frame
		send(sockfd, "T", 1, 0); //trigger camera
		Camera.grab();
		Camera.retrieve(mat); //save own picture to matrix
		for(int i=0; i < bufsize/RECV_BUF_LEN; i++) {
			recv_all(sockfd, buffer_ptr + i*RECV_BUF_LEN, RECV_BUF_LEN);
		}
		recv_all(sockfd, buffer_ptr + offset, bufsize - offset); //receive the last package of leftover size
		mat2 = bytesToMat(buffer_ptr, WINDOW_WIDTH,WINDOW_HEIGHT); //save received picture to matrix2

        cv::Scalar hsv_min = Scalar(hl, sl, vl, 0);
        cv::Scalar hsv_max = Scalar(hu, su, vu, 0);
        
		///1st DEVICE
        filter_hsv(&mat, &hsv_mat, &thr, hsv_min, hsv_max, erode_size, dilate_size, gauss_size);
		Rect bounding_rect = find_object(&mat, &thr, prev_positions);
		
		Point rect_center = (bounding_rect.br() + bounding_rect.tl())*0.5;
		prev_positions.erase(prev_positions.begin()); //delete first element
		prev_positions.push_back(rect_center); //add last position
		

		for(int i=0; i < prev_positions.size(); i++){
			circle(mat, prev_positions[i], 3, Scalar(0,0,0), CV_FILLED, 1,0);
		}

		draw_object(&mat, bounding_rect, rect_center);

 
 
		///2nd DEVICE
        filter_hsv(&mat2, &hsv_mat2, &thr2, hsv_min, hsv_max, erode_size, dilate_size, gauss_size);
		Rect bounding_rect2 = find_object(&mat2, &thr2, prev_positions2);
		Point rect_center2 = (bounding_rect2.br()+bounding_rect2.tl())*0.5;
		prev_positions2.erase(prev_positions2.begin()); //delete first element
		prev_positions2.push_back(rect_center2); //add last position
		
		for(int i=0; i < prev_positions2.size(); i++){
			circle(mat2, prev_positions2[i], 3, Scalar(0,0,0), CV_FILLED, 1,0);	
		}
		
		//search_area = Rect(bounding_rect.x-bounding_rect.width, bounding_rect.y-bounding_rect.height, bounding_rect.width*3, bounding_rect.height*3);

		//if(show_contours) drawContours( mat, contours,largest_contour_index, color, CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
		draw_object(&mat2, bounding_rect2, rect_center2);

		//rectangle(mat2, search_area2,  Scalar(255,0,0),1, 8,0);
		
		
		///Calculate distance 
		Point3d objectPosition = calc_distance(rect_center2, rect_center);
		cout << "D(cm): " << objectPosition.z << " X: " << objectPosition.x << " Y: " << objectPosition.y << endl;
		
		///draw plan view
		int PLAN_VIEW_SIZE = 320;   //pixels
		int PLAN_VIEW_AREA = 200; //cube in [cm]
		cv::Mat planView = cv::Mat::zeros(PLAN_VIEW_SIZE,PLAN_VIEW_SIZE, CV_8UC3);
		Point planPoint = Point(objectPosition.x/PLAN_VIEW_AREA * PLAN_VIEW_SIZE  + PLAN_VIEW_SIZE/2, objectPosition.z/PLAN_VIEW_AREA * PLAN_VIEW_SIZE);
		int planPointSize = objectPosition.y*15/PLAN_VIEW_AREA + 5;
		if(planPointSize >= 10) planPointSize = 10;
		if(planPointSize  <  1) planPointSize = 1;
		circle(planView, planPoint, planPointSize, Scalar(255,255,255), CV_FILLED, 1,0);	
		
		Point planPointZero = Point(0/PLAN_VIEW_AREA * PLAN_VIEW_SIZE  + PLAN_VIEW_SIZE/2, 10/PLAN_VIEW_AREA * PLAN_VIEW_SIZE);
		circle(planView, planPointZero, 3, Scalar(200,200,200), CV_FILLED, 1,0);	
		
		#ifdef FLIP_IMAGE
			cv::flip(mat, mat, 1);
			cv::flip(mat2,mat2,1);
			cv::flip(thr,thr,1);
			cv::flip(thr2,thr2,1);
			cv::flip(planView,planView,1);
		#endif
		
		imshow( "Camera", mat );// Original stream with detected ball overlay
        imshow( "After Color Filtering", thr); // The stream after color filtering
		imshow( "Camera2", mat2 );// Original stream with detected ball overlay
        imshow( "After Color Filtering2", thr2); // The stream after color filtering
		imshow( "Plan view", planView);
		
		int key = waitKey(1);                         
        switch(key){
            //case 32 : show_contours != show_contours; break; //space button
        }
    }
     return 0;
}
