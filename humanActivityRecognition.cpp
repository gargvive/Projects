#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <bits/stdc++.h>
#include <string.h>
#include <opencv/cv.h>
#include "opencv2/features2d/features2d.hpp"
#include <opencv/highgui.h>
#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;

Mat frame;
Mat fgMaskMOG;
Ptr<BackgroundSubtractor> pMOG;
int keyboard;

#define PI 3.14159265
#define FPS 30 //frame per second
#define IDEALHEIGHT 5.5 //feet
#define MINWIDTH 10

vector<Point> cog; //Array of center of gravity of body for each number..
vector<double> velocity; // velocity vector for for a interval equal to FPS
vector<int> Width;
vector<int> Height;
vector<double> aspectRatio; // a(t)=w(t)/h(t)
vector<double> leftAngle;
vector<double> rightAngle;
vector<double> pixelRatio;
vector<Point> rectangle_center;

void processVideo(char* videoFilename);
ofstream out("center_of_gravity.txt");
ofstream output("output.txt");
ofstream walking("walking.txt");
ofstream running("running.txt");
ofstream handWaving("handWaving.txt");

// Assuming 2 meter/second is threshold
double Runningthreshold = 2.8;
double walkingThreshold = 0.7;
double joggingThreshold = 2.0;

int frame_number=0;
int height_of_object=0;
int option;
bool standing;
string activity;

int calculateContourWidth(vector< vector<Point> > contours)
{
	int size=contours.size();
	//cout<<size<<endl;
    long total_points=0;

    /*minX is the minimum pixel in X direction of the body
    maxX is the maximum Pixel in X of body
    So width can be taken as maxX-minX 
    and Height to be maxY-minY */
    
    int x,y;
    int minX=INT_MAX,minY=INT_MAX;
    int maxX=INT_MIN,maxY=INT_MIN;
    int y_minX,x_minY,y_maxX,x_maxY;
    
    if(size)
    {
        for(int i=0;i<size;i++)
        {
            for(int j=0;j<contours[i].size();j++)
                {
                    x=contours[i][j].x;
                    y=contours[i][j].y;
                    if(minX>x) {
                    	minX=x;
                    	y_minX=y;
                    }
                    if(maxX<x) {
                    	maxX=x;
                    	y_maxX=y;
                    }
                    if(minY>y) {
                    	minY=y;
                    	x_minY=x;
                    }
                    if(maxY<y) {
                    	maxY=y;
                    	x_maxY=x;
                    }

                    total_points++;
                }
        }

        int w=maxX-minX;
        return w;
    
	}
}

bool isObjectPresent(vector< vector<Point> > contours)
{
	// Width less than this is considered as noise 
	if(calculateContourWidth(contours) > MINWIDTH) return true;
	return false;
}

void evaluateParameters(vector< vector<Point> > contours)
{
	Point center_of_gravity;

    center_of_gravity.x=0;
    center_of_gravity.y=0;

    int size=contours.size();
    long total_points=0;

    /*minX is the minimum pixel in X direction of the body
    maxX is the maximum Pixel in X of body
    So width can be taken as maxX-minX 
    and Height to be maxY-minY */
    
    int x,y;
    int minX=INT_MAX,minY=INT_MAX;
    int maxX=INT_MIN,maxY=INT_MIN;
    int y_minX,x_minY,y_maxX,x_maxY;
    
    if(size)
    {
        for(int i=0;i<size;i++)
        {
            for(int j=0;j<contours[i].size();j++)
                {
                    x=contours[i][j].x;
                    y=contours[i][j].y;
                    center_of_gravity.x+=x;
                    center_of_gravity.y+=y;
                    
                    if(minX>x) {
                    	minX=x;
                    	y_minX=y;
                    }
                    if(maxX<x) {
                    	maxX=x;
                    	y_maxX=y;
                    }
                    if(minY>y) {
                    	minY=y;
                    	x_minY=x;
                    }
                    if(maxY<y) {
                    	maxY=y;
                    	x_maxY=x;
                    }

                    total_points++;
                }
        }

        int w=maxX-minX;
        int h=maxY-minY;

        Point rectangleCenter;
        rectangleCenter.x=minX+(w/2.0);
        rectangleCenter.y=minY+(h/2.0);

        if(isObjectPresent(contours)) 
        {
            //cout<<maxY-minY<<endl;
            center_of_gravity.x/=total_points;
            center_of_gravity.y/=total_points;
            //cout<<"x= "<<center_of_gravity.x<<"  y= "<<center_of_gravity.y<<endl;
            if(option == 3)
            running<<center_of_gravity.x<<"     "<<center_of_gravity.y<<"     ";
            if(option == 2)
            walking<<center_of_gravity.x<<"     "<<center_of_gravity.y<<"     ";
            output<<center_of_gravity.x<<"     "<<center_of_gravity.y<<"     ";
            if(option == 3)
            running<<rectangleCenter.x<<"     "<<rectangleCenter.y<<"     "<<w<<"     "<<h<<endl;
            if(option == 2)
            walking<<rectangleCenter.x<<"     "<<rectangleCenter.y<<"     "<<w<<"     "<<h<<endl;
            if(option == 1)
            handWaving<<rectangleCenter.x<<"     "<<rectangleCenter.y<<"     "<<w<<"     "<<h<<endl;
            output<<rectangleCenter.x<<"     "<<rectangleCenter.y<<"     "<<w<<"     "<<h<<"     ";
            
            double AspectRatio = 1.0*w/h;
            double thetaRightHand = 1.0*(center_of_gravity.y-y_maxX)/(maxX-center_of_gravity.x);
            double thetaLeftHand = 1.0*(center_of_gravity.y-y_minX)/(center_of_gravity.x-minX);
            
            
            cog.push_back(center_of_gravity);
            rectangle_center.push_back(rectangleCenter);
            //out<<center_of_gravity.x<<" "<<center_of_gravity.y<<endl;
            Width.push_back(w);
            Height.push_back(h);
            aspectRatio.push_back(AspectRatio);
            leftAngle.push_back(atan(thetaLeftHand)*180/PI);
            rightAngle.push_back(atan(thetaRightHand)*180/PI);
            output<<atan(thetaLeftHand)*180/PI<<"     "<<atan(thetaRightHand)*180/PI<<endl;
            pixelRatio.push_back(1.0*maxY/minY);
            //cout<<w<<" "<<h<<endl;

            /*for(int i=0;i<Height.size();i++)
                height_of_object = max(height_of_object,Height[i]);*/
        }
        

        
    }
}

double cogVelocity()
{
    double time_per_frame=1000/FPS ; // Time is in milliSecond
    double vel=0;
    int window = 15 ;
    int Time = time_per_frame*window;
     height_of_object = 60 ;
    double conversion_factor = (IDEALHEIGHT*1000)/(1.67*height_of_object);
    int counter=0;
    int frame_size = cog.size();
    //cout<<rectangle_center.size()<<" "<<frame_number<<" ";
    //cout<<endl;
    for(int i = 0;i<frame_size-window-1;i++)
    {
        
        int j = i+window;

        double dist2 = (1.0*(cog[j].x-cog[i].x)*(cog[j].x-cog[i].x))+(cog[j].y-cog[i].y)*((cog[j].y-cog[i].y));
        //cout<<dist2<<" ";
        double distance = sqrt(dist2);
        //cout<<distance<<" ";
        double frame_vel= ((1.0*distance/Time)*conversion_factor); // meter/second
        
        if(frame_vel)
        {
            
            if(frame_vel<50) {
                counter++;
                vel+=frame_vel;
                //cout<<frame_vel<<" ";
            }
        }
        
    }
    //cout<<endl;
    vel/=(counter);
    //cout<<vel<<endl;
    int velocitySize = velocity.size();
    int abruptChange = 10;
   
    velocity.push_back(vel);
    
    return vel;
          
}

void emptyNecessaryContainers()
{
    cog.clear();
    rectangle_center.clear();
    leftAngle.clear();
    rightAngle.clear();
    Width.clear();
    Height.clear();
}

bool object_stationary()
{
	double vel = cogVelocity();
	if(vel<=walkingThreshold)
		return true;
	return false;
}

bool object_handwaving()
{
	int width_size = Width.size();
    double xLeft_Prev,xRight_Prev,xLeft_Current,xRight_Current;
    double DxLeft,DxRight;
    double sumDXLeft=0,sumDXRight=0;
    for(int i=1; i<width_size; i++ )
    {
        xLeft_Prev = rectangle_center[i-1].x-(Width[i-1]/2.0);
        xRight_Prev = rectangle_center[i-1].x+(Width[i-1]/2.0);
        xLeft_Current = rectangle_center[i].x-(Width[i]/2.0);
        xRight_Current = rectangle_center[i].x+(Width[i]/2.0);
        DxLeft = fabsf(xLeft_Prev-xLeft_Current);
        DxRight = fabsf(xRight_Prev-xRight_Current);
        sumDXLeft+=DxLeft;
        sumDXRight+=DxRight;
    }


    //cout<<sumDXLeft<<" "<<sumDXRight<<" ";
    bool leftFlag =false, rightFlag = false;
    double threshold = 1.0*height_of_object*width_size/60;
    //cout<<threshold<<endl;
    if(sumDXLeft>=threshold) leftFlag = true;
    else leftFlag = false;
    if(sumDXRight>=threshold) rightFlag = true;
    else rightFlag = false;

    //if(leftFlag) cout<<"LeftFlag true\n";
   // if(rightFlag) cout<<"RightFlag true\n";
    if(leftFlag && rightFlag) return true;
    return false;
    
}

bool object_boxing()
{
	int width_size = Width.size();
    double xLeft_Prev,xRight_Prev,xLeft_Current,xRight_Current;
    double DxLeft,DxRight;
    double sumDXLeft=0,sumDXRight=0;
    for(int i=1; i<width_size; i++ )
    {
        xLeft_Prev = rectangle_center[i-1].x-(Width[i-1]/2.0);
        xRight_Prev = rectangle_center[i-1].x+(Width[i-1]/2.0);
        xLeft_Current = rectangle_center[i].x-(Width[i]/2.0);
        xRight_Current = rectangle_center[i].x+(Width[i]/2.0);
        DxLeft = fabsf(xLeft_Prev-xLeft_Current);
        DxRight = fabsf(xRight_Prev-xRight_Current);
        sumDXLeft+=DxLeft;
        sumDXRight+=DxRight;
    }


    //cout<<sumDXLeft<<" "<<sumDXRight<<" "<<height_of_object;
    bool leftFlag , rightFlag;
    double threshold = 1.0*height_of_object*width_size/60;
    //cout<<threshold<<endl;
    if(sumDXLeft>=threshold) leftFlag = true;
    else leftFlag = false;
    if(sumDXRight>=threshold) rightFlag = true;
    else rightFlag = false;

    if(! (leftFlag && rightFlag) && (leftFlag || rightFlag)) return true;
    return false;
    
}

bool object_walking()
{
	double vel = cogVelocity(); 
	if(vel>walkingThreshold && vel<=Runningthreshold)
  	 	 return true;
	return false;
}

bool object_running()
{
	double vel = cogVelocity();
     if(vel>Runningthreshold)
         return true;
     return false;
}

void applyRules(vector< vector<Point> > contours)
{
	if(Width.size()<(FPS/2))
		{
			cout<<"Object not present\n";
			activity = "Object not present";
		}
	else
	{
		if(object_stationary())
		{
			if(object_handwaving())
				{
					cout<<"Object is standing and handwaving\n";
					activity = "Object is standing and handwaving";
				}
			else
				if(object_boxing())
					{
						cout<<"Object is standing and boxing\n";
						activity = "Object is standing and boxing";
					}
				else 
					{
						cout<<"Object is standing\n";
						activity = "Object is standing";
					}
		}
		else
		{
			if(object_walking())
				{
					cout<<"Object is Walking\n";
					activity = "Object is Walking";
				}
			else
				if(object_running())
					{
						cout<<"Object is running\n";
						activity = "Object is running";
					}
				else
					{
						cout<<"Rule for above activity is not defined\n";
						activity = "Rule for above activity is not defined";
					}
		}
	}

}

void writeInFile()
{
	if(option == 3 )
    running<<"Frame_Number  cog.x  cog.y  rect.x  rect.y  rect.width  rect.height\n";
    else if(option == 2)
    walking<<"Frame_Number  cog.x  cog.y  rect.x  rect.y  rect.width  rect.height\n";
    else if(option == 1)
    handWaving<<"Frame_Number  cog.x  cog.y  rect.x  rect.y  rect.width  rect.height\n";
    output<<"Frame_Number  cog.x  cog.y  rect.x  rect.y  rect.width  rect.height  LeftHand  RightHand\n";
}

VideoCapture getInput()
{
	ofstream outputFile;
    outputFile.open("output.txt");
    namedWindow("Frame");
    namedWindow("FG Mask MOG");
    pMOG= new BackgroundSubtractorMOG();
    CvVideoWriter* outputMovie = cvCreateVideoWriter("outputvide.avi",
    CV_FOURCC('P', 'I', 'M', '1'), 30, cvSize(320,240),true);

    char *videoFilename;

    cout<<"Press\n1. HandWaving\n2. Walking\n3. Running\nDefault: Boxing\n";
    
    cin>>option;

    switch(option){
    	case 1: videoFilename="videos/handWaving.avi";
    			break;
    	case 2: videoFilename="videos/walking.avi";
    			break;
    	case 3: videoFilename="videos/running.avi";
    			break;
    	default: videoFilename="videos/boxing.avi";
    }

    cv::VideoCapture cap(videoFilename);

    IplImage* fgmask;

    if (!cap.isOpened())
    {
        std::cout << "!!! Failed to open file: " << std::endl;
        return -1;
    }
    VideoCapture capture(videoFilename);
    
    writeInFile();

    if(!capture.isOpened()){
        cerr << "Unable to open video file: " << videoFilename << endl;
        exit(EXIT_FAILURE);
    }

    return capture;

}

int main()
{
 	
 	VideoCapture capture = getInput();
 	activity = "";
    while( (char)keyboard != 'q' && (char)keyboard != 27 ){

        if(!capture.read(frame)) {
            cerr << "Unable to read next frame." << endl;
            cerr << "Exiting..." << endl;
            break;
        }

        frame_number++;
        running<<frame_number<<"     ";
        walking<<frame_number<<"     ";
        handWaving<<frame_number<<"		";
        output<<frame_number<<"     ";

        pMOG->operator()(frame, fgMaskMOG);
        //imshow("FG Mask MOG", fgMaskMOG);


        Mat img0= fgMaskMOG;
        Mat img1;

        img1=img0;
        Canny(img1, img1, 100, 200);

        vector< vector<Point> > contours;
        findContours(img1, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);


        Mat mask = Mat::zeros(img1.rows, img1.cols, CV_8UC1);

        drawContours(mask, contours, -1, Scalar(255), CV_FILLED);
        
        //cout<<contours.size()<<"  ";
        evaluateParameters(contours);
        if(frame_number%FPS==0) {

            applyRules(contours);
            emptyNecessaryContainers();
        }

      	normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);

      	//Put text in main frame window showing the activity it is doing.
    	IplImage tmp= frame;
    	CvFont font;
		double hScale=0.5;
		double vScale=0.5;
		int    lineWidth=1;
		cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
    	//cvPutText (&tmp, "Vivek Garg", cvPoint(10,10), &font, cvScalar(255,255,0));
    	putText(frame, activity, Point2f(0,20), FONT_HERSHEY_PLAIN, 0.5,  Scalar(255,0,0,255),1,8);
        
        /*imshow("mask", mask); // Use to show the masked image frame
        imshow("canny", img1); // Use to show canny image frame */
    	imshow("Frame", frame);

    	
    	keyboard = waitKey( FPS );
        }

    capture.release();
    destroyAllWindows();
    return EXIT_SUCCESS;
}
