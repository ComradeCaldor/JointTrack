/*****************************************************************************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/
#include "jointtracker.h"
#include<thread> //Файл в котором определен класс thread 
extern "C" {
#include <xdo.h>
};

xdo_t* xdoMain;
//void CvDrawingUtils::QBe1(cv::Mat &Image,Marker &m,const CameraParameters &CP);
int bprint_angles = 0;
int bprint_range = 0,
    bprint_time_detection = 0;


string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=-1;
int ThePyrDownLevel;
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
vector<Marker> TheMarkers2;
Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;

pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=0;

int id_marker1 = 384,
    id_marker2 = 728;
/*int id_marker1 = 60,
    id_marker2 = 490;*/
/*int id_marker1 = 366,
    id_marker2 = 514;*/

int disp_res_x = 1900,
    disp_res_y = 800,
    mouse_to_x = 0,
    mouse_to_y = 0;

int readArguments ( int argc,char **argv )
{
    if (argc<2) {
        cerr<<"Invalid number of arguments"<<endl;
        cerr<<"Usage: (in.avi|live) [intrinsics.yml] [size]"<<endl;
        return 0;
    }
    TheInputVideo=argv[1];
    if (argc>=3)
        TheIntrinsicFile=argv[2];
    if (argc>=4)
        TheMarkerSize=atof(argv[3]);

    if (argc==3)
        cerr<<"NOTE: You need makersize to see 3d info!!!!"<<endl;
    return 1;
}
/************************************
 *
 *
 *
 *
 ************************************/

int main(int argc,char **argv)
{
    int icamera = 1;                          // LoadCamera /dev/video0-1-2-3
    int bDetectMode = 2;                      // 0-off 1-Rng 2-a0 3-a1 4-a2
    int slider_R[4] = {5154,4726,4478,6537  };  // thresholds for activation

    int id = 0;                               // Marker id   
    float NowTvec[1024][3];                   // Vector Coord
    float NowRvec[1024][3];                   // Vector angles
    Mat rod460, rod60, rod60460;              // Rotation matrices for Rodrigues
    short  bM11 = 0,                       // Marker 1 has been detected on this step
           bM21 = 0;                       //        2
    double euler2[3];                         // Euler angles from rotation matrices

    float dT[3];
    float dR[3];
    float dTmax[3] = {0,0,0};
    float dRmax[3] = {0,0,0};

    float Range = 0,
          Range_min = 999999990,
          Range_max = 0,
          Range_aver = 0,
          Angle = 0,
          Angle_max = 0,
          Angle_min = 99999999,
          Angle_xyz[3] = {0,0,0},
          Angle_xyz_max[3] = {0,0,0},
          Angle_xyz_min[3] = {9999999,99999,99999},
          Angle_aver[3] = {0,0,0},
          aver_coeff[4] = {150,150,150,150},
          AnglePerc[3] = {0,0,0},
          Percent = 0,
//        thresholdPercent[4] = {0.3,0.3,0.3,0.3}, // NA
          thresholdDelta[4] = {0.3,0.3,0.3,6};     // thresholds for activation

    int bin_x1, bin_x2;

    int bFirst = 1;
    int bBeyond = 0;    // pos > threshold 
    int bTrigd = 0;
    int bAdaptive = 1;
    int bOutmode = 0;
    int bMovemode = 0;
  	int bVideowrite = 0;
    char arg[] = "../video/test_video.avi";

    double t_button = 0;  
    double t_death = 0.1;
    short untriggered = 1;

  	struct timeval tp;
    gettimeofday(&tp, NULL);
  	long int ms_start = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    long int ms, ms_prev;

    int ret = 0;
    char ssystem[100];

    std::string shelp("\td: switch detection mode; 0-off 1-Rng 2-a0 3-a1 4-a2\n");
    shelp.append("\tf: adaptive mode on/off;\n");
    shelp.append("\tc: ranges = reset absolute thresholds vals;\n");
    shelp.append("\tt: print times of detection; on/off\n");
    shelp.append("\te: set exposure to 10\n");
    shelp.append("\to: set output mode:\n");
    shelp.append("\t      0: press 'space'\n");
    shelp.append("\t      1: move mouse\n");
    shelp.append("\t      N/A: hold 'space'\n");
    shelp.append("\tv: mouse mode\n");
    shelp.append("\t      0: vertical\n");
    shelp.append("\t      1: horizontal\n");
    shelp.append("\t      2: 2d\n");
    shelp.append("\tw: write video\n");
    cout << shelp << endl;

    try
    {
        xdoMain = xdo_new(NULL);

        if (readArguments (argc,argv)==0) {
            return 0;
        }
        //parse arguments
        ;
        //read from camera or from  file
        if (TheInputVideo=="live") {
cout << "CAMERA" << endl;
            TheVideoCapturer.open(icamera);
//            TheVideoCapturer.set(15,0);
            TheVideoCapturer.set(5,1);
            waitTime=1;
        }
        else  TheVideoCapturer.open(TheInputVideo);
        //check video is open
        if (!TheVideoCapturer.isOpened()) {
            cerr<<"Could not open video"<<endl;
            return -1;

        }

        //read first image to get the dimensions
        TheVideoCapturer>>TheInputImage;

        //read camera parameters if passed
        if (TheIntrinsicFile!="") {
            TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
            TheCameraParameters.resize(TheInputImage.size());
        }
        //Configure other parameters
        if (ThePyrDownLevel>0)
            MDetector.pyrDown(ThePyrDownLevel);

        //Create gui___________________________________________________________
        cv::namedWindow("thres",1);
        cv::namedWindow("in",1);
        cv::namedWindow("options",1);
        cv::Mat pic = cv::Mat::zeros(260,250,CV_8UC3);
        MDetector.getThresholdParams( ThresParam1,ThresParam2);
        MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
        iThresParam1=ThresParam1;
        iThresParam2=ThresParam2;
        cv::createTrackbar("ThresParam1", "in",&iThresParam1, 13, cvTackBarEvents);
        cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);
        //________________________________________________________________________
        char key=0;// for input
        //capture until press ESC or until the end of the video

        createTrackbar("R0","options",&slider_R[0],10000);
        createTrackbar("R1","options",&slider_R[1],10000);
        createTrackbar("R2","options",&slider_R[2],10000);
        createTrackbar("R","options",&slider_R[3],10000);

        int ret = system("v4l2-ctl -d /dev/video1 --set-ctrl power_line_frequency=2  --set-ctrl exposure_auto=1 --set-ctrl exposure_absolute=10;");

        VideoWriter video(arg,CV_FOURCC('M','J','P','G'),30, Size(640,480),true);

//   double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
//   double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
        int iNFrame = 0;

        while ( key!=27 && TheVideoCapturer.grab())
        {
            iNFrame++;

  
            if(iNFrame%10 == 0)
            {
                  gettimeofday(&tp, NULL);
                  ms_prev = ms;
                  ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
           // if(bprint_time_detection == 1 && iNFrame%10 == 0) //cout << ((double)ms - ms_start)/1000 << endl;
                    cout << "fps:" << 1000/((double)ms - ms_prev)*10 << endl;
            }


            bM11 = 0;
            bM21 = 0;

            if(ms - ms_start < 3000)
            {
                   Range_min = Range;
                   Range_max = Range;
                   Range_aver = Range;
                   for(int i = 0; i < 3; i++)
                   {
                       Angle_aver[i] = euler2[i];
                       Angle_xyz_min[i] = euler2[i];
                       Angle_xyz_max[i] = euler2[i];
                   }
            }

            for(int i = 0; i < 4; i++)
              thresholdDelta[i] = ((double)slider_R[i]-5000)*0.001;

            TheVideoCapturer.retrieve( TheInputImage);
            //copy image

            //Detection of markers in the image passed
            MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,TheMarkerSize);

            //print marker info and draw the markers in image
            TheInputImage.copyTo(TheInputImageCopy);

            for (unsigned int i=0; i<TheMarkers.size(); i++) {
                // cout<< i << ":" << TheMarkers[i] <<endl;
                TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
            }
            //print other rectangles that contains no valid markers
            /**     for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
                     aruco::Marker m( MDetector.getCandidates()[i],999);
                     m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
                 }*/

            //draw a 3d cube in each marker if there is 3d info
            if (  TheCameraParameters.isValid())
                for (unsigned int i=0; i<TheMarkers.size(); i++) // Marker cycle
                {

                    id = TheMarkers[i].id;

                    //CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
                    //                    CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[5],TheCameraParameters);
                    CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);

                    if ( id == id_marker1)
                    {
                        //cout << "==========" << id_marker1 << "===========" << endl;
                        cv::Rodrigues(TheMarkers[i].Rvec,rod60);       //Извлекаем матрицы поворотов для первого и второго маркера
                        cv::Rodrigues(TheMarkers[i].Rvec,rod60460);
                        bM11 = 1;
                    }
                    if ( id == id_marker2)
                    {
                        //cout << "==========" << id_marker2 << "==========" << endl;
                        cv::Rodrigues(TheMarkers[i].Rvec,rod460);       //Извлекаем матрицы поворотов для первого и второго маркера
                        bM21 = 1;
                    }
                    if(bM11 || bM21)
                    {
					              NowTvec[id][0] = TheMarkers[i].Tvec.ptr<float>(0)[0];
					              NowTvec[id][1] = TheMarkers[i].Tvec.ptr<float>(0)[1];
					              NowTvec[id][2] = TheMarkers[i].Tvec.ptr<float>(0)[2];
					              NowRvec[id][0] = TheMarkers[i].Rvec.ptr<float>(0)[0];
					              NowRvec[id][1] = TheMarkers[i].Rvec.ptr<float>(0)[1];
					              NowRvec[id][2] = TheMarkers[i].Rvec.ptr<float>(0)[2];
                    }
                }//markers cycle

            if(bM11 && bM21)
            {
                dT[0] = NowTvec[id_marker2][0] - NowTvec[id_marker1][0];
                dT[1] = NowTvec[id_marker2][1] - NowTvec[id_marker1][1];
                dT[2] = NowTvec[id_marker2][2] - NowTvec[id_marker1][2];

                Range = sqrt(dT[0]*dT[0]+dT[1]*dT[1]+dT[2]*dT[2]);
                if(bprint_range == 1)  cout << "\nRange(1:2)= " << Range << endl;

                for (int i=0; i<3; i++)
                    for(int j=0; j<3; j++)
                    {
                        rod60460.at<float>(i,j) = 0;
                    }

                rod60460 += rod460.inv()*rod60;
  
                if(bprint_angles)
                {
      
                    cout << "60 * 460^t: " << endl;
                    for (int i=0; i<3; i++)
                        for(int j=0; j<3; j++)
                        {
                            cout << "\t" << rod60460.at<float>(i,j);
                            if(j == 2)
                                cout << endl;
                        }
                }

                Mat anglesfromrod;
                cv::Rodrigues(rod60460,anglesfromrod);

                if(bprint_angles)
                {
                  cout << "-_-_-_-_-_-_-_-_-_anglesfromrod: " << endl;

                  for(int i = 0; i < 3; i++)
                      cout << anglesfromrod.at<float>(i) << "\t";
                }

                euler2[0] =  anglesfromrod.at<float>(0);
                euler2[1] =  anglesfromrod.at<float>(1);
                euler2[2] =  anglesfromrod.at<float>(2);

                if(Range > Range_max)
                    Range_max = Range;

                if(Range < Range_min)
                    Range_min = Range;


                if(bAdaptive)
                   Range_aver = (aver_coeff[3]*Range_aver + Range)/(aver_coeff[3]+1);

                for(int i = 0; i < 3; i++)
                {                
                    if(euler2[i] > Angle_xyz_max[i])
                        Angle_xyz_max[i] = euler2[i];
                    if(euler2[i] < Angle_xyz_min[i])
                        Angle_xyz_min[i] = euler2[i];

                    if(bAdaptive == 1)
                          Angle_aver[i] = (aver_coeff[i]*Angle_aver[i]+euler2[i])/(aver_coeff[i]+1);  //Adaptive
                      else if(bAdaptive == 2)
                             Angle_aver[i] += (euler2[i] - Angle_aver[i]) * 0.01;
                        else if(bAdaptive == 3)
                                Angle_aver[i] += 0.001*copysignf(1, (euler2[i] - Angle_aver[i]));         //Hard
                           else if(bAdaptive == 4)
                                {
                                    cout << fabs(euler2[1]) << endl;
                                    cout << fabs(Angle_aver[1] + thresholdDelta[1]) << endl;
                                     cout << fabs(euler2[1]) -  fabs(Angle_aver[1] + thresholdDelta[1]) << endl << endl;

                                     if(  fabs(euler2[i]) < fabs(Angle_aver[i] + thresholdDelta[i]))
                                     {
                                         Angle_aver[i] = (2*aver_coeff[i]*Angle_aver[i]+euler2[i])/(2*aver_coeff[i]+1);
                                         if(i == 1)cout << "a" << endl;
                                     }
                                       else
                                        {
                                             Angle_aver[i] += (euler2[i] - Angle_aver[i]) * 0.1;
                                             if(i == 1)cout << "b" << endl;
                                        }
                                }
                              else if(bAdaptive == 5)
                                   {
                                          Angle_aver[i] = (aver_coeff[i]*Angle_aver[i]+euler2[i])/(aver_coeff[i]+1);  //Adaptive
                                          if( euler2[i] <  Angle_aver[i] && thresholdDelta[i] > 0 || euler2[i] >  Angle_aver[i] && thresholdDelta[i] < 0)
                                                    Angle_aver[i] = euler2[i];
                                   }
                                else if(bAdaptive == 6)
                                     {
                                            Angle_aver[i] = (aver_coeff[i]*Angle_aver[i]+euler2[i])/(aver_coeff[i]+1);  //Adaptive
                                            if( euler2[i] <  Angle_aver[i] && thresholdDelta[i] > 0 || euler2[i] >  Angle_aver[i] && thresholdDelta[i] < 0)
                                                      Angle_aver[i] = euler2[i];

                                            if( euler2[i] >  Angle_aver[i] + 3*thresholdDelta[i] &&  thresholdDelta[i] > 0)
                                                      Angle_aver[i] = euler2[i] - 3*thresholdDelta[i];

                                            if( euler2[i] <  Angle_aver[i] + 3*thresholdDelta[i] &&  thresholdDelta[i] < 0)
                                                      Angle_aver[i] = euler2[i] - 3*thresholdDelta[i];
                                     }
                }
//cout << "Diff: " << (euler2[1] - Angle_aver[1]) << endl;

                //-----------Add cube and draw-----------------------
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
                    cv::line(TheInputImageCopy,imagePoints[i],imagePoints[(i+1)%4],Scalar(0,0,255,255),1,CV_AA);
                for (int i=0; i<4; i++)
                    cv::line(TheInputImageCopy,imagePoints[i+4],imagePoints[4+(i+1)%4],Scalar(0,255,0,255),1,CV_AA);
                for (int i=0; i<4; i++)
                    cv::line(TheInputImageCopy,imagePoints[i],imagePoints[i+4],Scalar(255,0,0,255),1,CV_AA);
                //------^^^^^^^Add cube and draw^^^^^--------------------
            }//if(bM11 && bM21)

            switch (key)
            {
              case 'd':
                {
                    if(bDetectMode == 4)
                        bDetectMode = 0;
                      else bDetectMode++;
                }
               break;
              case 'f':
                {
                    bAdaptive++;
                    if(bAdaptive == 7)
                       bAdaptive = 0;
                }
               break;
              case 'c':
                {
                   Range_min = Range;
                   Range_max = Range;
                   Range_aver = Range;
                   for(int i = 0; i < 3; i++)
                   {
                       Angle_aver[i] = euler2[i];
                       Angle_xyz_min[i] = euler2[i];
                       Angle_xyz_max[i] = euler2[i];
                   }
                }
               break;
              case 't':
                {
                  if(bprint_time_detection == 0)
                      bprint_time_detection = 1;
                    else
                      bprint_time_detection = 0;
                }
               break;
              case 'e':
                {
                    int ret = system("v4l2-ctl -d /dev/video1 --set-ctrl power_line_frequency=2  --set-ctrl exposure_auto=1 --set-ctrl exposure_absolute=10;");
                }
               break;
              case 'o':
                {
                   xdo_send_keysequence_window_up(xdoMain, CURRENTWINDOW, "space", 500);
                   bOutmode += 1;// slider_R[1] = 2;
                   if(bOutmode > 1)
                      bOutmode = 0; 
                }
               break;
              case 'v':
                {
                    xdo_send_keysequence_window_up(xdoMain, CURRENTWINDOW, "space", 500);
                    bMovemode += 1;
                    if(bMovemode == 3)
                        bMovemode = 0;
                }
               break;
              case 'h':
                {
                    cout << shelp << endl;
                    getchar();
                }
               break;
              case 'w':
                {
                    if(bVideowrite == 0)
                    {
                          cout << "Start recording..." << endl;
                          bVideowrite = 1;
                    }
                      else
                        {
                             cout << "...stop recording." << endl;
                             bVideowrite = 0;
                        }
                }
               break;
            }//switch(k)

            AnglePerc[0] = ((double)euler2[0] - (double)Angle_xyz_min[0])/((double)Angle_xyz_max[0] - (double)Angle_xyz_min[0]);
            AnglePerc[1] = ((double)euler2[1] - (double)Angle_xyz_min[1])/((double)Angle_xyz_max[1] - (double)Angle_xyz_min[1]);
            AnglePerc[2] = ((double)euler2[2] - (double)Angle_xyz_min[2])/((double)Angle_xyz_max[2] - (double)Angle_xyz_min[2]);

            //------------Draw bins-----------------------
            cv::rectangle(pic, cv::Point(0,0), cv::Point(250, 250), cv::Scalar(0,0,0), -1, CV_AA);

            double xmin, xmax;
            double line1 = 0, lineMax = 0, lineMin = 0, lineAver = 0, lineThresh = 0;
            xmin = Range_min - (Range_max - Range_min)/3;
            xmax = Range_max + (Range_max - Range_min)/3;
            line1 = ((double)Range- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineMax = ((double)Range_max- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineMin = ((double)Range_min- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineAver = ((double)Range_aver- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineThresh = ((double)(Range_aver+thresholdDelta[3])- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;

            bin_x1 = 240;
            bin_x2 = 250;

            cv::rectangle(pic, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(255,0,0), -1, CV_AA);
    //        cv::rectangle(pic, cv::Point(bin_x1,70-2), cv::Point(bin_x2, 70+2), cv::Scalar(175,0,0), -1, CV_AA);                  //line
      //      cv::rectangle(pic, cv::Point(bin_x1,195-2), cv::Point(bin_x2, 195+2), cv::Scalar(175,0,0), -1, CV_AA);                //line
           // cv::rectangle(pic, cv::Point(bin_x1,lineMax), cv::Point(bin_x2, lineMax+4), cv::Scalar(75,0,0), -1, CV_AA);           // Min
           // cv::rectangle(pic, cv::Point(bin_x1,lineMin), cv::Point(bin_x2, lineMin+4), cv::Scalar(75,0,0), -1, CV_AA);           // Max
            cv::rectangle(pic, cv::Point(bin_x1,lineThresh-2), cv::Point(bin_x2, lineThresh+2), cv::Scalar(0,75,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,lineAver-2), cv::Point(bin_x2, lineAver+2), cv::Scalar(0,175,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);                // Now
            cv::putText(pic, "R.", cvPoint(240,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);

            xmin = 2;
            xmax = -3;

            bin_x1 = 135;
            bin_x2 = 145;
            //xmin = 0;
            //xmax = 0;
            line1 = 0;
            //xmin = Angle_xyz_min[2] - (Angle_xyz_max[2] - Angle_xyz_min[2])/3;
            //xmax = Angle_xyz_max[2] + (Angle_xyz_max[2] - Angle_xyz_min[2])/3;
            lineMax = ((double)Angle_xyz_max[0]- (double)xmin) * (250) / ((double)xmax - (double)xmin) ;
            lineMin = ((double)Angle_xyz_min[0]- (double)xmin) * (250) / ((double)xmax - (double)xmin) ;
            lineAver = ((double)Angle_aver[0]- (double)xmin) * (250) / ((double)xmax - (double)xmin) ;
            lineThresh = ((double)(Angle_aver[0]+thresholdDelta[0])- (double)xmin) * (250) / ((double)xmax - (double)xmin) ;
            line1 = ((double)euler2[0]- (double)xmin) * (250) / ((double)xmax - (double)xmin) ;
            cv::rectangle(pic, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(255,0,0), -1, CV_AA);           // Столбец
           // cv::rectangle(pic, cv::Point(bin_x1,lineMin), cv::Point(bin_x2, lineMin+4), cv::Scalar(175,0,0), -1, CV_AA);      // min
           // cv::rectangle(pic, cv::Point(bin_x1,lineMax-2), cv::Point(bin_x2, lineMax+2), cv::Scalar(175,0,0), -1, CV_AA);      // max
            cv::rectangle(pic, cv::Point(bin_x1,lineThresh-2), cv::Point(bin_x2, lineThresh+2), cv::Scalar(0,75,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,lineAver-2), cv::Point(bin_x2, lineAver+2), cv::Scalar(0,175,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);      // Измеренное значение 
            cv::putText(pic, "0", cvPoint(135,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);

            bin_x1 = 180;
            bin_x2 = 190;
            xmax = (double)Angle_aver[1]+2*(double)thresholdDelta[1];
            xmin = (double)Angle_aver[1]-2*(double)thresholdDelta[1];
            if (lineThresh > line1)
            {
                xmin = line1;
                xmax = lineThresh;
            }
            lineMax = ((double)Angle_xyz_max[0]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineMin = ((double)Angle_xyz_min[0]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineAver = ((double)Angle_aver[0]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineThresh = ((double)(Angle_aver[0]+thresholdDelta[0])- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            line1 = ((double)euler2[0]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            cv::rectangle(pic, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(255,0,0), -1, CV_AA);           // Столбец
            cv::rectangle(pic, cv::Point(bin_x1,lineMin), cv::Point(bin_x2, lineMin+4), cv::Scalar(175,0,0), -1, CV_AA);      // min//---------------------------------------------------------------ZOOM
            cv::rectangle(pic, cv::Point(bin_x1,lineMax-2), cv::Point(bin_x2, lineMax+2), cv::Scalar(175,0,0), -1, CV_AA);      // max
            cv::rectangle(pic, cv::Point(bin_x1,lineThresh-2), cv::Point(bin_x2, lineThresh+2), cv::Scalar(0,75,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,lineAver-2), cv::Point(bin_x2, lineAver+2), cv::Scalar(0,175,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);      // Измеренное значение 
            cv::putText(pic, "0", cvPoint(135,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);

            xmin = 3;
            xmax = -3;
            bin_x1 = 150;
            bin_x2 = 160;
            line1 = 0;
            lineMax = ((double)Angle_xyz_max[1]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineMin = ((double)Angle_xyz_min[1]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineAver = ((double)Angle_aver[1]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineThresh = ((double)(Angle_aver[1]+thresholdDelta[1])- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            line1 = ((double)euler2[1]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            cv::rectangle(pic, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(255,0,0), -1, CV_AA);
           // cv::rectangle(pic, cv::Point(bin_x1,lineMin), cv::Point(bin_x2, lineMin+4), cv::Scalar(175,0,0), -1, CV_AA);      // min
           // cv::rectangle(pic, cv::Point(bin_x1,lineMax-2), cv::Point(bin_x2, lineMax+2), cv::Scalar(175,0,0), -1, CV_AA);      // max
            cv::rectangle(pic, cv::Point(bin_x1,lineThresh-2), cv::Point(bin_x2, lineThresh+2), cv::Scalar(0,75,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,lineAver-2), cv::Point(bin_x2, lineAver+2), cv::Scalar(0,175,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);    // Измеренное значение 
            cv::putText(pic, "1", cvPoint(150,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);

            bin_x1 = 195;
            bin_x2 = 205;
            xmax = (double)Angle_aver[1]+2*(double)thresholdDelta[1];
            xmin = (double)Angle_aver[1]-2*(double)thresholdDelta[1];
            lineMax = ((double)Angle_xyz_max[1]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineMin = ((double)Angle_xyz_min[1]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;                         //  << ZOOM
            lineAver = ((double)Angle_aver[1]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineThresh = ((double)(Angle_aver[1]+thresholdDelta[1])- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            line1 = ((double)euler2[1]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
/*cout << "xmax:" << xmax << endl;
cout << "xmin:" << xmin << endl;
cout << "euler2[1]:" << euler2[1] << endl;
cout << "%:" << ((double)euler2[1]-(double)xmin) / ((double)xmax-(double)xmin) << endl;
cout << "line1:" << line1 << endl;*/
            cv::rectangle(pic, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(255,100,100), -1, CV_AA);
  //          cv::rectangle(pic, cv::Point(bin_x1,lineMin), cv::Point(bin_x2, lineMin+4), cv::Scalar(175,0,0), -1, CV_AA);      // min
//            cv::rectangle(pic, cv::Point(bin_x1,lineMax-2), cv::Point(bin_x2, lineMax+2), cv::Scalar(175,0,0), -1, CV_AA);      // max
            cv::rectangle(pic, cv::Point(bin_x1,lineThresh-2), cv::Point(bin_x2, lineThresh+2), cv::Scalar(0,75,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,lineAver-2), cv::Point(bin_x2, lineAver+2), cv::Scalar(0,175,0), -1, CV_AA);      // aver//---------------------------------------------------------------ZOOM
            cv::rectangle(pic, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);    // Измеренное значение 
            cv::putText(pic, "1", cvPoint(150,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);


            xmin = 3;
            xmax = -3;
            bin_x1 = 165;
            bin_x2 = 175;
            line1 = 0;
            lineMax = ((double)Angle_xyz_max[2]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineMin = ((double)Angle_xyz_min[2]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineAver = ((double)Angle_aver[2]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineThresh = ((double)(Angle_aver[2]+thresholdDelta[2])- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            line1 = ((double)euler2[2]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            cv::rectangle(pic, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(255,0,0), -1, CV_AA);
           // cv::rectangle(pic, cv::Point(bin_x1,lineMin), cv::Point(bin_x2, lineMin+4), cv::Scalar(175,0,0), -1, CV_AA);      // min
           // cv::rectangle(pic, cv::Point(bin_x1,lineMax-2), cv::Point(bin_x2, lineMax+2), cv::Scalar(175,0,0), -1, CV_AA);      // max
            cv::rectangle(pic, cv::Point(bin_x1,lineThresh-2), cv::Point(bin_x2, lineThresh+2), cv::Scalar(0,75,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,lineAver-2), cv::Point(bin_x2, lineAver+2), cv::Scalar(0,175,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);      // Измеренное значение 
            cv::putText(pic, "2", cvPoint(165,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);

/*            bin_x1 = 210;
            bin_x2 = 220;
            line1 = 0;
            xmax = line1;
            xmin = lineThresh;
            lineMax = ((double)Angle_xyz_max[2]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineMin = ((double)Angle_xyz_min[2]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineAver = ((double)Angle_aver[2]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            lineThresh = ((double)(Angle_aver[2]+thresholdDelta[2])- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            line1 = ((double)euler2[2]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            cv::rectangle(pic, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(255,0,0), -1, CV_AA);
            cv::rectangle(pic, cv::Point(bin_x1,lineMin), cv::Point(bin_x2, lineMin+4), cv::Scalar(175,0,0), -1, CV_AA);      // min
            cv::rectangle(pic, cv::Point(bin_x1,lineMax-2), cv::Point(bin_x2, lineMax+2), cv::Scalar(175,0,0), -1, CV_AA);      // max
            cv::rectangle(pic, cv::Point(bin_x1,lineThresh-2), cv::Point(bin_x2, lineThresh+2), cv::Scalar(0,75,0), -1, CV_AA);      // aver
            cv::rectangle(pic, cv::Point(bin_x1,lineAver-2), cv::Point(bin_x2, lineAver+2), cv::Scalar(0,175,0), -1, CV_AA);      // aver//---------------------------------------------------------------ZOOM
            cv::rectangle(pic, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);      // Измеренное значение 
            cv::putText(pic, "2", cvPoint(165,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);*/
            /* 
            	bin_x1 = 180;
            	bin_x2 = 190;
            	//xmin = 0;
            	//xmax = 0;
            	line1 = 0;
            	//xmin = Angle_xyz_min[2] - (Angle_xyz_max[2] - Angle_xyz_min[2])/3;
            	//xmax = Angle_xyz_max[2] + (Angle_xyz_max[2] - Angle_xyz_min[2])/3;
            	line1 = ((double)dR[2]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            	cv::rectangle(pic, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(255,0,0), -1, CV_AA);
            	cv::rectangle(pic, cv::Point(bin_x1,70-2), cv::Point(bin_x2, 70+2), cv::Scalar(175,0,0), -1, CV_AA);
            	cv::rectangle(pic, cv::Point(bin_x1,195-2), cv::Point(bin_x2, 195+2), cv::Scalar(175,0,0), -1, CV_AA);
            	cv::rectangle(pic, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);

            	bin_x1 = 195;
            	bin_x2 = 205;
            	//xmin = 0;
            	//xmax = 0;                                                                                           // Какая-то херня в абсолютных значениях.
            	line1 = 0;
            	//xmin = Angle_xyz_min[1] - (Angle_xyz_max[1] - Angle_xyz_min[1])/3;
            	//xmax = Angle_xyz_max[1] + (Angle_xyz_max[1] - Angle_xyz_min[1])/3;
            	line1 = ((double)dR[1]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            	cv::rectangle(pic, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(255,0,0), -1, CV_AA);
            	cv::rectangle(pic, cv::Point(bin_x1,70-2), cv::Point(bin_x2, 70+2), cv::Scalar(175,0,0), -1, CV_AA);
            	cv::rectangle(pic, cv::Point(bin_x1,195-2), cv::Point(bin_x2, 195+2), cv::Scalar(175,0,0), -1, CV_AA);
            	cv::rectangle(pic, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);

            	bin_x1 = 210;
            	bin_x2 = 220;
            	//xmin = 0;
            	//xmax = 0;
            	line1 = 0;
            	//xmin = Angle_xyz_min[0] - (Angle_xyz_max[0] - Angle_xyz_min[0])/3;
            	//xmax = Angle_xyz_max[0] + (Angle_xyz_max[0] - Angle_xyz_min[0])/3;
            	line1 = ((double)dR[0]- (double)xmin) * (250-25) / ((double)xmax - (double)xmin) + 25;
            	cv::rectangle(pic, cv::Point(bin_x1,25), cv::Point(bin_x2, 250), cv::Scalar(255,0,0), -1, CV_AA);
            	cv::rectangle(pic, cv::Point(bin_x1,70-2), cv::Point(bin_x2, 70+2), cv::Scalar(175,0,0), -1, CV_AA);
            	cv::rectangle(pic, cv::Point(bin_x1,195-2), cv::Point(bin_x2, 195+2), cv::Scalar(175,0,0), -1, CV_AA);
            	cv::rectangle(pic, cv::Point(bin_x1,line1), cv::Point(bin_x2, line1+4), cv::Scalar(0,0,0), -1, CV_AA);
            */
            if(bM11 && bM21)
            {
                bTrigd = 0;
                cv::rectangle(pic, cv::Point(10,0), cv::Point(75, 50), cv::Scalar(255,0,0), -1, CV_AA);
                switch (bDetectMode)
                {
                  case 0:
                    {
                        cv::putText(pic, "Off.", cvPoint(10,15), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,0), 1, CV_AA);
                    }
                   break;
                  case 1:
                    {
                        cv::putText(pic, "Rng.", cvPoint(10,15), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255,255,255), 1, CV_AA);
     
                        if(bM11 && bM21)
                        {
                          if( thresholdDelta[3] < 0 && Range > Range_aver + thresholdDelta[3] || thresholdDelta[3] > 0 && Range < Range_aver + thresholdDelta[3])
                                bBeyond = 0;

                          if(bBeyond == 0 && thresholdDelta[3] < 0 && Range < Range_aver + thresholdDelta[3] || bBeyond == 0 && thresholdDelta[3] > 0 && Range > Range_aver + thresholdDelta[3])
                          {
                                bBeyond = 1;
                                bTrigd = 1;
                          }
                        }

                    }
                   break;
                  case 2:
                    {
                        cv::putText(pic, "R0.", cvPoint(10,15), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255,255,255), 1, CV_AA);

                        Percent = AnglePerc[0];

                        if(bM11 && bM21)
                        {
                          if( thresholdDelta[0] < 0 && euler2[0] > Angle_aver[0] + thresholdDelta[0] || thresholdDelta[0] > 0 && euler2[0] < Angle_aver[0] + thresholdDelta[0])
                                bBeyond = 0;

                          if(bBeyond == 0 && thresholdDelta[0] < 0 && euler2[0] < Angle_aver[0] + thresholdDelta[0] || bBeyond == 0 && thresholdDelta[0] > 0 && euler2[0] > Angle_aver[0] + thresholdDelta[0])
                          {
                                bBeyond = 1;
                                bTrigd = 1;
                          }
                        }
                    }
                   break;
                  case 3:
                    {
                        cv::putText(pic, "R1.", cvPoint(10,15), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255,255,255), 1, CV_AA);

                        Percent = AnglePerc[1];

                        if(bM11 && bM21)
                        {
                          if( thresholdDelta[1] < 0 && euler2[1] > Angle_aver[1] + thresholdDelta[1] || thresholdDelta[1] > 0 && euler2[1] < Angle_aver[1] + thresholdDelta[1])
                                bBeyond = 0;

                          if(bBeyond == 0 && thresholdDelta[1] < 0 && euler2[1] < Angle_aver[1] + thresholdDelta[1] || bBeyond == 0 && thresholdDelta[1] > 0 && euler2[1] > Angle_aver[1] + thresholdDelta[1])
                          {
                                bBeyond = 1;
                                bTrigd = 1;
                          }
                        }
                    }
                   break;
                  case 4:
                    {
                        cv::putText(pic, "R2.", cvPoint(10,15), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255,255,255), 1, CV_AA);

                        Percent = AnglePerc[2];

                        if(bM11 && bM21)
                        {
                          if( thresholdDelta[2] < 0 && euler2[2] > Angle_aver[2] + thresholdDelta[2] || thresholdDelta[2] > 0 && euler2[2] < Angle_aver[2] + thresholdDelta[2])
                                bBeyond = 0;

                          if(bBeyond == 0 && thresholdDelta[2] < 0 && euler2[2] < Angle_aver[2] + thresholdDelta[2] || bBeyond == 0 && thresholdDelta[2] > 0 && euler2[2] > Angle_aver[2] + thresholdDelta[2])
                          {
                                bBeyond = 1;
                                bTrigd = 1;
                          }
                        }
                    }
                   break;
                }
               // cout << bBeyond << "-" << bTrigd << endl; //switchcase result

      /*                        cout << "ms: " << (ms-ms_start)/1000 << endl;
                              cout << "t_button: " << t_button << endl;
                              cout << "t_death: " << t_death << endl;
                              cout << (ms-ms_start)/1000 - t_button << " < " << t_death << endl;
    */
    //cout << 1000*AvrgTime.first/AvrgTime.second - t_button
    //cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;


                        switch (bOutmode)
                        {
                          case 0: // space
                            {
      //                        if( (ms-ms_start)/1000 - t_button > t_death) //Death time
                                {
        //                            t_button = (ms-ms_start)/1000;
                                      if(bMovemode == 0)
                                      {
                                          if(bTrigd == 1)
                                          {
                      //                       int ret = system("xdotool key space;"); //  echo \"Jump!\";  // xdotool key --delay 2 space;");
                      //                       int ret = system("xset r off; echo "" && xdotool key --delay 2 space; xset r on;");
                      //                       int ret = system("xdotool search \"Mozilla Firefox\" windowactivate; xdotool --sync key --clearmodifiers  mousemove 800 400; xdotool click 1; xdotool key space");
                      //                        xdotool search "[Firefox Page Title]" windowactivate --sync key --clearmodifiers ctrl+r
                                               //ret = system("xdotool keydown space && sleep 0.02 & xdotool keyup space &");
                                               xdo_send_keysequence_window(xdoMain, CURRENTWINDOW, "space", 500);
                                               cout << "JUMP!!\n" << endl;
                                               bTrigd = 0;

                                               cv::putText(pic, "o", cvPoint(90,25), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(100,255,100), 1, CV_AA);
                                          }
                                          else
                                              ;//ret = system("xdotool keyup space &"); //!!!!PELIGRO!!!!Takes ~3 fps.
                                      }
                                      else 
                                        if(bMovemode == 1 && (bTrigd == 1 || bBeyond == 1) )
                                              xdo_send_keysequence_window_down(xdoMain, CURRENTWINDOW, "space", 500);
                                          else
                                               xdo_send_keysequence_window_down(xdoMain, CURRENTWINDOW, "space", 500);
                                }
      //                          else
      //                              cv::putText(pic, "red", cvPoint(10,35), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,255), 1, CV_AA);
                            }
                           break;
                          case 1: // mouse
                            {
                              mouse_to_x = disp_res_x / 2;
                              mouse_to_y = disp_res_y / 2;

                              if(bMovemode == 0)                                                  //x
                                    mouse_to_x = Percent*1800;
                                else if(bMovemode == 1)                                                  //y
                                    mouse_to_y =Percent*900;
                                else if(bMovemode == 2)                                                  //x+y
                                {
                                    mouse_to_x = Percent*1800;
                                    mouse_to_y =Percent*900;
                                }

                              if (bDetectMode != 0 && bM11 && bM21)
                                  {
                                      xdo_move_mouse(xdoMain,mouse_to_x,mouse_to_y,0);
                                  }
                              //int ret = system("xdotool mousemove 800 400; xdotool click 1; xdotool key space");    
                            }
                              break;
                        }
            }


            //show input with augmented information and  the thresholded image
             cv::imshow("in",TheInputImageCopy);
             cv::imshow("thres",MDetector.getThresholdedImage());

            char text_Mode[4];
            sprintf(text_Mode,"M:%d",bMovemode);
            cv::putText(pic, text_Mode, cvPoint(10,35), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255,255,255), 1, CV_AA);
            sprintf(text_Mode,"O:%d",bOutmode);
            cv::putText(pic, text_Mode, cvPoint(10,55), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255,255,255), 1, CV_AA);
            sprintf(text_Mode,"A:%d",bAdaptive);
            cv::putText(pic, text_Mode, cvPoint(10,75), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255,255,255), 1, CV_AA);
            imshow("options", pic);

            if(bVideowrite == 1)
             video.write(TheInputImage);

            key=cv::waitKey(waitTime);//wait for key to be pressed
            //--------------/Draw bins-----------------
        }//while key != 27

    } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }

}//main
/************************************
 *
 *
 *
 *
 ************************************/

void cvTackBarEvents(int pos,void*)
{
    if (iThresParam1<3) iThresParam1=3;
    if (iThresParam1%2!=1) iThresParam1++;
    if (ThresParam2<1) ThresParam2=1;
    ThresParam1=iThresParam1;
    ThresParam2=iThresParam2;
    MDetector.setThresholdParams(ThresParam1,ThresParam2);
//recompute
    MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters);
    TheInputImage.copyTo(TheInputImageCopy);
    for (unsigned int i=0; i<TheMarkers.size(); i++)	TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
    //print other rectangles that contains no valid markers
    /*for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
        aruco::Marker m( MDetector.getCandidates()[i],999);
        m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
    }*/

//draw a 3d cube in each marker if there is 3d info
    if (TheCameraParameters.isValid())
        for (unsigned int i=0; i<TheMarkers.size(); i++)
            CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);

  //  cv::imshow("in",TheInputImageCopy);
  //  cv::imshow("thres",MDetector.getThresholdedImage());
}
