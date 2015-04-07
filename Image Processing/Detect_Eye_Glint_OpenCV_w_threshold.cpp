
/*
 * File:   Detect_Eye_Glint_OpenCV.cpp
 * Author: Adarsh Ramakrishna
 * 		   Madhusudhan Ramesh Kumar
 *         Divya Uthappa Mandeda 
 *
 * Created on March 21, 2013, 12:10 AM
 */


#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
//#include <time.h>
#include <wiringPi.h>

using namespace cv;

/* DEBUG_FLAG set to 0 will disable all LOGS
 * DEBUG_FLAG set to 1 will enable all LOGS
 * */
#define DEBUG_FLAG 1
#define ENABLE_UART 0

#if DEBUG_FLAG == 1
#define LOG(...) 	printf(__VA_ARGS__)
#else
#define LOG(...)
#endif

int diffCounter = 0;

void imageProcessing(cv::Mat &image1,cv::Mat &image2);
void transmit_glint_location(int x, int y);
void transmit_gl(int x, int y);										//£DEV
void init_communications();
void detect(cv::Mat &image1,cv::Mat &image2);
void close_uart();

int uart0_filestream = -1;

int main ( int argc, char **argv )
{
#if ENABLE_UART == 1
    init_communications();
#endif
    
    CvCapture* capture = 0;
    
    capture = cvCaptureFromCAM(0);
    
#if DEBUG_FLAG == 1
    //cvNamedWindow("Video");
    //cvMoveWindow("Video",50,70);
    //cvNamedWindow( "Difference Image", CV_WINDOW_AUTOSIZE );
    //cvMoveWindow("Difference Image",720,70);
#endif
    wiringPiSetup ();						
    pinMode (0, OUTPUT);
    
    //CvConvexityDefect::start;
    
	//clock_t start_time, end_time;
    //double cpu_time_used;
     
    //start_time = clock();
     
    //int i=0;
    //for(i=0;i<500;i++)
    int c,cc;
    while((char)c!=27)							//NEW::Replacement for infinite loop
	{											//exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
        cv::Mat frame_1, frame_2; 				
        cv::Mat frame_1_copy, frame_2_copy; 	
        cv::Mat temp1, temp2;
    
		//WiringPi LED actions :: LED-ON   
		digitalWrite(0, HIGH); //delay(500);	//<REDUCES FPS> 
		
        IplImage* iplImg_1 = cvQueryFrame( capture );
		frame_1 = iplImg_1;
		if( frame_1.empty() )
		{
			LOG("Could not capture frame 1 \n");
			exit(0);
		}
		frame_1.copyTo(frame_1_copy);
        
        
        //New::LED-OFF
        digitalWrite(0, LOW); //delay(500);		//<REDUCES FPS>
        
        IplImage* iplImg_2 = cvQueryFrame(capture);
        frame_2 = iplImg_2; 						
        if( frame_2.empty() )
        {
            LOG("Could not capture frame 2 \n");
            exit(0);
        }
        frame_2.copyTo(frame_2_copy);
        
        cc++; //printf("Capture Count: %d \n",cc);						//__TESTING__
        //LOG("Before Image processing \n");

        cv::cvtColor(frame_1_copy,frame_1_copy,CV_RGB2GRAY);
        cv::cvtColor(frame_2_copy,frame_2_copy,CV_RGB2GRAY);
		
        detect(frame_1_copy, frame_2_copy);								//**RESOLVED::ERROR
        //printf("Detection is Executed \n");							//__TESTING__
        //wait for 40 milliseconds
        c = cvWaitKey(05);
        
    }
   /*   	end_time = clock();
    *  	cpu_time_used = ((double) (end_time - start_time)) / CLOCKS_PER_SEC;
    * 
	*	printf("CPU time used is: %f\n",cpu_time_used); 
    * 
	*	int y;
    * 	y= i/cpu_time_used;
	*	printf("frames processed per sec: %d\n",y);
    */
    cvWaitKey(0);
    
#if DEBUG_FLAG == 1
    cvDestroyWindow("Video");
#endif
    cvReleaseCapture(&capture);
    
#if ENABLE_UART == 1
    close_uart();
#endif
    //return 0;
    
    
}

/*
 *  Name: 			imageProcessing
 *  Description: 	Performs image processing on two image frames,
 * 					the difference is computed and from the difference
 * 					image the brightest spot is obtained
 * 	Input Arguments: Two image frames image1 and image2
 *  Output : Computes (x,y) co-ordinates of the brightest glint.
 *
 */
void imageProcessing(cv::Mat &image1,cv::Mat &image2)
{
    Mat scaledImage1,scaledImage2;
    Mat diffImage;
    Mat blockFindingMatrix;
    
	//?UNUSED::DELETE
	//char filename3[100];
    //int x1y1Found, x2y2Found;
    int k,l;				//i,j;
    int xNew = 1, yNew = 1; //x1 = 1, y1 = 1, x2 = 1, y2 = 1;
    int rowNum, colNum;
    int rows, cols;
    int xSize = 3;
    int ySize = 3;
    double movingMax = 0.0, currentBlockSum= 0.0;
    static int xpos_prev = 0;
    static int ypos_prev = 0;
    
    // scaling the images to half its size
    Size s1(image1.size().width/2,image1.size().height/2);
    resize(image1,scaledImage1,s1, 0,0, CV_INTER_AREA);
    
    Size s2(image2.size().width/2,image2.size().height/2);
    resize(image2,scaledImage2,s2, 0,0, CV_INTER_AREA);
    
    //taking absolute difference of the 2 images
#if DEBUG_FLAG == 1
    //imshow( "Video", scaledImage1 ); // preview (only in a debug show ), take image and display in window
    //imshow( "Video", scaledImage2 );
#endif
    
    diffImage = abs(scaledImage1-scaledImage2);						//**FIX
    
    diffCounter++;
    rows = diffImage.rows;
    cols = diffImage.cols;
    
    threshold( diffImage, diffImage, 80, 255, 3 );
    
    /* Conventional Image processing logic start
     Moving max logic start */
    blockFindingMatrix = Mat::zeros(rows, cols, CV_32F);
    
    for (rowNum = 1; rowNum < xSize; rowNum++)
    {
        for (colNum = 1; colNum < cols; colNum++)
        {
            //copy the first row(s) as it is
            blockFindingMatrix.at<double>(rowNum,colNum) = diffImage.at<double>(rowNum,colNum);
        }
    }
    
    //setting initial value to -1000 so that the currentBlockSum is always
    //greater than movingMax during very first pass
    movingMax = -1000.0;
    for (rowNum = xSize+1; rowNum < rows; rowNum++)
    {
        for (colNum = 1; colNum < cols; colNum++)
        {
            currentBlockSum = 0.0;
            // blockFindingMatrix.at<double>(rowNum,colNum) = diffImage.at<double>(rowNum,colNum);
            //Always skip the first column as it wont have left neighbor
            if (colNum > ySize )
            {
                for (k = (rowNum - xSize) ; k < rowNum; k++)
                {
                    for (l = (colNum - ySize) ; l < colNum; l++)
                    {
                        currentBlockSum = currentBlockSum + diffImage.at<uchar>(k,l);
                    }
                }
                //save the values if the current block is the movingMax
                if ( currentBlockSum > movingMax )
                {
                    movingMax =  currentBlockSum;
                    /*Doing this because XNew and YNew
                     * are representing Y and X respectively
                     * when they are returned to calling function.
                     * Centre of gravity of the glint.. */
                    xNew = rowNum-(xSize/2);
                    yNew = colNum-(ySize/2);
                }
            }
        }
    }
    
    if(xNew == 0 && yNew == 0)
    {
        xNew = xpos_prev;
        yNew = ypos_prev;
    }
    else if(abs(xNew - xpos_prev) > 500 || abs(yNew - ypos_prev) > 500)
    {
        xNew = xpos_prev;
        yNew = ypos_prev;
    }
    else
    {
        xpos_prev = xNew;
        ypos_prev = yNew;
    }
    
    LOG("Value of x,y = %d %d \n",xNew, yNew);
    
#if ENABLE_UART == 1
    transmit_glint_location(xNew, yNew);
#endif
    
#if DEBUG_FLAG == 1
    cv::circle(diffImage, cvPoint(xNew, yNew),5, CV_RGB(0,0,255), 4, 8,0);
    imshow( "Difference Image", diffImage );
#endif
    
}

void transmit_glint_location(int x, int y)
{
    //?UNUSED::DELETE
    //char start_bit = 0;
    char msg[8];
    int count;
    
    sprintf(msg,"%04d%04d",x,y);
    
    assert (uart0_filestream != -1);
    
    /* Transmit x_pos and y_pos */
    
    count = write(uart0_filestream, &msg, sizeof(char) * 8);		//Filestream, bytes to write, number of bytes to write
    if (count < 0)
    {
        printf("UART TX error for X_POS \n");
    }
    
}

void transmit_gl(int x, int y)
{
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;
	
	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = x;
	*p_tx_buffer++ = y;
	
	if(uart0_filestream != -1){
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));
		if (count<0){
			printf("UART TX error\n");
		}
	}
}
/*
 * Use this function to init all global variables and connection information
 */
void init_communications()
{
    uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
    if (uart0_filestream == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
        assert(0);
    }
    
    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE:- CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);
    
}


void detect(cv::Mat &image1,cv::Mat &image2)
{
    Mat scaledImage1,scaledImage2;
    Mat diffImage;
    Mat blockFindingMatrix;
    
    //?UNUSED::DELETE
    //int rows, cols;
    //int x1y1Found, x2y2Found;
    //int i,j,k,l;
    //int rowNum, colNum;
    //int x1 = 1, y1 = 1, x2 = 1, y2 = 1, xNew = 1, yNew = 1;
    //int xSize = 3;
    //int ySize = 3;
    //double movingMax = 0.0, currentBlockSum= 0.0;
    //char filename3[100];
    
    static int xpos_prev = 0;
    static int ypos_prev = 0;
    static int xpos = 0;
    static int ypos = 0;
    
    
    // scaling the images to half its size
    Size s1(image1.size().width/2,image1.size().height/2);
    resize(image1,scaledImage1,s1, 0,0, CV_INTER_AREA);
    
    Size s2(image2.size().width/2,image2.size().height/2);
    resize(image2,scaledImage2,s2, 0,0, CV_INTER_AREA);
    
    //printf("Image Scaling Occurred \n");											//__TESTING__
    //printf("Dimensions:: sI1:%d sI2:%d \n",scaledImage1.dims,scaledImage2.dims);	//__TESTING__
    //taking absolute difference of the 2 images
#if DEBUG_FLAG == 1
    imshow( "Video", scaledImage1 );												//__TESTING__
    imshow( "Video", scaledImage2 );
    
#endif
    resize(image2,diffImage,s2,0,0,CV_INTER_AREA);					//QUICK-FIX::NEEDS MODIFICATION
    diffImage = abs(&scaledImage1-&scaledImage2);					//**RESOLVED
    
    // Variables required for the MinmaxLoc function
    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    //printf("Dimension:: %d \n",diffImage.dims);						//__TESTING__
    threshold( diffImage, diffImage, 80, 255, 3 ); 					//**RESOLVED
    //printf("Threshold Executed \n");								//__TESTING__
    
    cv::Mat dst;
    cv::flip(diffImage,dst,0);
    Point2f src_center(dst.cols/2.0F, dst.rows/2.0F);
    
    cv::Mat rot_matrix = getRotationMatrix2D(src_center, 180.0, 1.0);
    cv::Mat rotated_img(Size(dst.size().height, dst.size().width), dst.type());
    
    warpAffine(dst, rotated_img, rot_matrix, dst.size());
    
    // This Opencv function finds the position (x,y co-ordinates) of the brightest spot in the image
    minMaxLoc( rotated_img, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    xpos=maxLoc.x;
    ypos=maxLoc.y;
    // Try to avoid very fast & small movement of the mouse due to small changes in the glint position
    // We therefore try to keep the mouse pointer relatively fixed for very small changes in (x,y) positions
    if(xpos <= xpos_prev + 2 && xpos >= xpos_prev - 2)
        xpos = xpos_prev;
    if(ypos <= ypos_prev + 2 && ypos >= ypos_prev - 2)
        ypos = ypos_prev;
    
    if(xpos == 0 && ypos == 0)
    {
        xpos = xpos_prev;
        ypos = ypos_prev;
    }
    else if(abs(xpos - xpos_prev) > 50 || abs(ypos - ypos_prev) > 50)
    {
        xpos = xpos_prev;
        ypos = ypos_prev;
    }
    else
    {
        xpos_prev = xpos;
        ypos_prev = ypos;
    }
    //LOG("Value of x,y = %d %d \n",xpos, ypos);									//__TESTING__
    //transmit_glint_location(xpos, ypos);
    transmit_gl(xpos, ypos);
    
#if ENABLE_UART == 1

	transmit_glint_location(xpos, ypos);
#endif
    
#if DEBUG_FLAG == 1
    //cv::circle(rotated_img, cvPoint(xpos, ypos),5, CV_RGB(0,0,255), 4, 8,0);			//__TESTING__
    //imshow( "Difference Image", rotated_img );
#endif
			 
}


void close_uart()
{
    close(uart0_filestream);	
}
