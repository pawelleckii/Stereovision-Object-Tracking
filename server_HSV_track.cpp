//works with client_HSV_track app

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <iostream>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <raspicam/raspicam_cv.h>

using namespace std;

#define PORT "1500"//"3490"  // the port users will be connecting to
#define ADDRESS "169.254.33.199"

#define BACKLOG 10

#define BYTES_PER_PIXEL 3
#define IMG_FORMAT CV_8UC3


const int WINDOW_WIDTH = 320;
const int WINDOW_HEIGHT = 240;
const int VIEW_SCALE = 2;


void sigchld_handler(int s)
{
    // waitpid() might overwrite errno, so we save and restore it:
    int saved_errno = errno;
    while(waitpid(-1, NULL, WNOHANG) > 0);
    errno = saved_errno;
}


// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

void prepare_connection(int *sockfd, int *new_fd, sockaddr_storage *their_addr, socklen_t *sin_size)
{
	struct sigaction sa;
    struct addrinfo hints, *servinfo, *p;
    int yes=1;
    int rv;
    //struct sockaddr_storage their_addr; // connector's address information
    //socklen_t sin_size;
	char s[INET6_ADDRSTRLEN]; //connection got from %s, s

	//timeout
	/*struct timeval timeout;
	timeout.tv_sec = 11;
	timeout.tv_usec = 0;
	*/
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    
    hints.ai_socktype = SOCK_STREAM;
    //hints.ai_flags = AI_PASSIVE; // use my IP

    if ((rv = getaddrinfo(ADDRESS, PORT, &hints, &servinfo)) != 0) { //£1 NULL
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return;
    }
    

    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((*sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("server: socket");
            continue;
        }
        cout << "setting server on: " << ADDRESS << endl;

        if (setsockopt(*sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
		    perror("setsockopt");
            cin >> yes; //STOP
            exit(1);
        }
        

        if (bind(*sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(*sockfd);
            perror("server: bind");
            cin >> yes; //STOP
            continue;
        }

        break;
    }

    freeaddrinfo(servinfo); // all done with this structure

    if (p == NULL)  {
        fprintf(stderr, "server: failed to bind\n");
        cin >> yes; //STOP
        exit(1);
    }
    
    if (listen(*sockfd, BACKLOG) == -1) {
        perror("listen");
        cin >> yes; //STOP
        exit(1);
    }

    sa.sa_handler = sigchld_handler; // reap all dead processes
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    if (sigaction(SIGCHLD, &sa, NULL) == -1) {
        perror("sigaction");
        cin >> yes; //STOP
        exit(1);
    }

    printf("server: waiting for connections...\n");

    *sin_size = sizeof(*their_addr);
	*new_fd = accept(*sockfd, (struct sockaddr *)&(* their_addr), &(*sin_size));
	if (*new_fd == -1) {
		perror("accept");
		//continue;
	}

	inet_ntop(their_addr->ss_family,
		get_in_addr((struct sockaddr *)&their_addr),
		s, sizeof s);
	printf("server: got connection from %s\n", s);
	
	//timeouts - optional
	/*
	if (setsockopt(*new_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0) {
		perror("setsockopt");
		exit(1);
	}
	if (setsockopt(*new_fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout)) < 0) {
		perror("setsockopt");
		exit(1);
	}
	if (setsockopt(*sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0) {
		perror("setsockopt");
		exit(1);
	}
	if (setsockopt(*sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout)) < 0) {
		perror("setsockopt");
		exit(1);
	}*/
}

void show_frame(string name, cv::Mat imgFrame){
		cv::namedWindow(name, CV_WINDOW_NORMAL);
		cv::resizeWindow(name, WINDOW_WIDTH*VIEW_SCALE, WINDOW_HEIGHT*VIEW_SCALE);
		cv::imshow(name, imgFrame);
		cv::waitKey(0); 
}

void matToBytes(cv::Mat image, unsigned char * bytes)
{
   int size = WINDOW_WIDTH * WINDOW_HEIGHT* BYTES_PER_PIXEL * sizeof(unsigned char);//image.total() * image.elemSize();

   memcpy(bytes,image.data,size * sizeof(unsigned char));
}

cv::Mat bytesToMat(unsigned char * bytes,int width,int height)
{
    cv::Mat image = cv::Mat(height,width,IMG_FORMAT ,bytes).clone(); // make a copy //CV_8UC3 
    return image;
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

int main(void)
{
	//for reconnecting
	struct sockaddr_storage their_addr; // connector's address information
    socklen_t sin_size;
	
    int sockfd, new_fd;  // listen on sock_fd, new connection on new_fd
	int recv_buf_len = 16384;
    
    prepare_connection(&sockfd, &new_fd, &their_addr, &sin_size);
    
	raspicam::RaspiCam_Cv Camera;
    Camera.set( CV_CAP_PROP_FORMAT, IMG_FORMAT ); //CV_32FC3
    Camera.set( CV_CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH);
    Camera.set( CV_CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT);
	
    cv::Mat imgFrame;
    
	cout<<"Opening Camera..."<<endl;	
    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}

		Camera.grab();
		Camera.retrieve(imgFrame);
		
		int bufsize = WINDOW_WIDTH * WINDOW_HEIGHT* BYTES_PER_PIXEL * sizeof(unsigned char); //20; //640*480; //230400
		
		cout << "size of image frame: " << sizeof(imgFrame) << endl;
		unsigned char * byte_img_ptr = new unsigned char[bufsize];
		matToBytes(imgFrame, byte_img_ptr);
		
		//show_frame("Display frame", imgFrame); 
				
		cv::Mat restored_image;
		restored_image = bytesToMat(byte_img_ptr, WINDOW_WIDTH,WINDOW_HEIGHT);
		cout << "WH = " <<WINDOW_WIDTH <<" " << WINDOW_HEIGHT << endl;
		int restored_size = restored_image.total() * restored_image.elemSize() * sizeof(unsigned char);
		cout << "restored size = " << restored_size << endl;
		//show_frame("Restored frame", restored_image);
		
	
		cout << "recv_buf_len " << recv_buf_len << endl;
		int offset = (bufsize/recv_buf_len) * recv_buf_len;
		unsigned char cmd_buf[1];
		cv::namedWindow("Sent frame", CV_WINDOW_NORMAL);
		cv::resizeWindow("Sent frame", WINDOW_WIDTH*VIEW_SCALE, WINDOW_HEIGHT*VIEW_SCALE);
		cv::Mat imgSent;
		int num = 0;
		int receive_flag = 1;
		
		while(1){
			receive_flag = 1;
			num++;
			receive_flag = recv(new_fd, cmd_buf, 1, 0); //receive and check for 0 (client discon)

			if (receive_flag <= 0){ //connection closed by client
				cout << "closing connection" << endl;
				close(new_fd);
				cout << "reconnecting" << endl;
				new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
				if (new_fd == -1) {
					perror("accept");
					continue;
				}
				continue;
			}
			else if(cmd_buf[0] == 'T'){ //trigger camera
				Camera.grab();
				Camera.retrieve(imgFrame);
				matToBytes(imgFrame, byte_img_ptr);
				for(int i=0; i < bufsize/recv_buf_len; i++) {
					send_all(new_fd, byte_img_ptr + i*recv_buf_len, recv_buf_len);	
				}
				send_all(new_fd, byte_img_ptr + offset, bufsize - offset); //send the last package (of a leftover size)
			}else if(cmd_buf[0] == 'S'){
				break;
			}
		}
    return 0;
}
