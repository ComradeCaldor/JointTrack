#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include <time.h>
#include <sys/time.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace aruco;

void cvTackBarEvents(int pos,void*);
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CP,Size size);

void DrawBar(cv::Mat picture, int bin_x1, int bin_x2, double position, double pos_min, double pos_max, double aver, double thresholdDelta);
void DrawBar_2(cv::Mat picture, int bin_x1, int bin_x2, double position, double pos_min, double pos_max, double aver, double thresholdDelta, int color[3]);
void DrawCube2(cv::Mat picture, vector<Marker> TheMarkers, CameraParameters TheCameraParameters, float Range, double euler2[3]);

void DrawBar(cv::Mat picture, int bin_x1, int bin_x2, double position, double pos_min, double pos_max, double aver, double thresholdDelta)
{
      double xmin, xmax;
      double line1 = 0, lineMax = 0, lineMin = 0, lineAver = 0, lineThresh = 0;
      xmin = pos_min - (pos_max - pos_min)/3;
      xmax = pos_max + (pos_max - pos_min)/3;
      line1 = ((double)position- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
      lineMax = ((double)pos_max- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
      lineMin = ((double)pos_min- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
      lineAver = ((double)aver- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
      lineThresh = ((double)(aver+thresholdDelta)- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;

      cv::rectangle(picture, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(255,0,0), -1, CV_AA);
//        cv::rectangle(pic, cv::Point(bin_x1,70-2), cv::Point(bin_x2, 70+2), cv::Scalar(175,0,0), -1, CV_AA);                  //line
//      cv::rectangle(pic, cv::Point(bin_x1,195-2), cv::Point(bin_x2, 195+2), cv::Scalar(175,0,0), -1, CV_AA);                //line
     // cv::rectangle(pic, cv::Point(bin_x1,lineMax), cv::Point(bin_x2, lineMax+4), cv::Scalar(75,0,0), -1, CV_AA);           // Min
     // cv::rectangle(pic, cv::Point(bin_x1,lineMin), cv::Point(bin_x2, lineMin+4), cv::Scalar(75,0,0), -1, CV_AA);           // Max
      cv::rectangle(picture, cv::Point(bin_x1,lineThresh-2), cv::Point(bin_x2, lineThresh+2), cv::Scalar(0,75,0), -1, CV_AA);      // aver
      cv::rectangle(picture, cv::Point(bin_x1,lineAver-2), cv::Point(bin_x2, lineAver+2), cv::Scalar(0,175,0), -1, CV_AA);      // aver
      cv::rectangle(picture, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);                // Now
      cv::putText(picture, "R.", cvPoint(240,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);
}

void DrawBar_2(cv::Mat picture, int bin_x1, int bin_x2, double position, double pos_min, double pos_max, double aver, double thresholdDelta, int color[3] )
{
      double xmin, xmax;
      double line1 = 0, lineMax = 0, lineMin = 0, lineAver = 0, lineThresh = 0;
      xmin = aver + 2 * thresholdDelta;
      xmax = aver -  thresholdDelta;
      line1 = ((double)position- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
      lineMax = ((double)pos_max- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
      lineMin = ((double)pos_min- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
      lineAver = ((double)aver- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
      lineThresh = ((double)(aver+thresholdDelta)- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;

      cv::rectangle(picture, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(color[0],color[1],color[2]), -1, CV_AA);
      cv::rectangle(picture, cv::Point(bin_x1,lineThresh-2), cv::Point(bin_x2, lineThresh+2), cv::Scalar(0,75,0), -1, CV_AA);      // aver
      cv::rectangle(picture, cv::Point(bin_x1,lineAver-2), cv::Point(bin_x2, lineAver+2), cv::Scalar(0,175,0), -1, CV_AA);      // aver
      cv::rectangle(picture, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);                // Now
      cv::putText(picture, "R.", cvPoint(240,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);
}

void DrawCube2(cv::Mat picture, vector<Marker>  TheMarkers, CameraParameters TheCameraParameters, float Range, double euler2[3])
{
                Mat objectPoints (8,3,CV_32FC1);
                double halfSize=TheMarkers[0].ssize/2;
                double cubeSize=TheMarkers[0].ssize;

                objectPoints.at<float>(0,0)=-halfSize; 	//x
                objectPoints.at<float>(0,1)=0;		      //y
                objectPoints.at<float>(0,2)=-halfSize;	//z
                objectPoints.at<float>(1,0)=halfSize;		//x
                objectPoints.at<float>(1,1)=0;		    	//y
                objectPoints.at<float>(1,2)=-halfSize;	//z
                objectPoints.at<float>(2,0)=halfSize;	  //x
                objectPoints.at<float>(2,1)=0;		      //y
                objectPoints.at<float>(2,2)=halfSize;	  //z
                objectPoints.at<float>(3,0)=-halfSize;	//x
                objectPoints.at<float>(3,1)=0;			    //y
                objectPoints.at<float>(3,2)=halfSize;		//z

                objectPoints.at<float>(4,0)=-halfSize;	//x
                objectPoints.at<float>(4,1)=cubeSize;	  //y
                objectPoints.at<float>(4,2)=-halfSize;	//z
                objectPoints.at<float>(5,0)=halfSize;		//x
                objectPoints.at<float>(5,1)=cubeSize;		//y
                objectPoints.at<float>(5,2)=-halfSize;	//z
                objectPoints.at<float>(6,0)=halfSize;	  //x
                objectPoints.at<float>(6,1)=cubeSize; 	//y
                objectPoints.at<float>(6,2)=halfSize;	  //z
                objectPoints.at<float>(7,0)=-halfSize;	//x
                objectPoints.at<float>(7,1)=cubeSize;		//y
                objectPoints.at<float>(7,2)=halfSize;		//z

                TheMarkers[0].Tvec.ptr<float>(0)[0] = 0;//NowTvec[60][0];  //  y
                TheMarkers[0].Tvec.ptr<float>(0)[1] = 0;//NowTvec[60][1];  //  |
                TheMarkers[0].Tvec.ptr<float>(0)[2] = Range + 11;          //  z---x

                TheMarkers[0].Rvec.ptr<float>(0)[0] = euler2[0];//NowRvec[460][0];
                TheMarkers[0].Rvec.ptr<float>(0)[1] = euler2[1];//NowRvec[460][1];
                TheMarkers[0].Rvec.ptr<float>(0)[2] = euler2[2];//NowRvec[460][2];

                vector<Point2f> imagePoints;
                projectPoints( objectPoints, TheMarkers[0].Rvec, TheMarkers[0].Tvec,  TheCameraParameters.CameraMatrix,TheCameraParameters.Distorsion,   imagePoints);

                for (int i=0; i<4; i++)
                    cv::line(picture,imagePoints[i],imagePoints[(i+1)%4],Scalar(0,0,255,255),1,CV_AA);
                for (int i=0; i<4; i++)
                    cv::line(picture,imagePoints[i+4],imagePoints[4+(i+1)%4],Scalar(0,255,0,255),1,CV_AA);
                for (int i=0; i<4; i++)
                    cv::line(picture,imagePoints[i],imagePoints[i+4],Scalar(255,0,0,255),1,CV_AA);
}
