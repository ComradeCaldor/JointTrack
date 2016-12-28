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
    bprint_fps = 1;


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

/*int id_marker1 = 384,
    id_marker2 = 728;*/
//int id_marker1 = 777, // 50mm
//    id_marker2 = 555;
int id_marker1 = 666, // 50mm
    id_marker2 = 114;
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
// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;
     
}
 
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    assert(isRotationMatrix(R));
     
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    return Vec3f(x, y, z);     
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
    int slider_R[4] = {5154,5075,4478,6537  };  // thresholds for activation 1-4726

    int id = 0;                               // Marker id   
    float NowTvec[1024][3];                   // Vector Coord
    float NowRvec[1024][3];                   // Vector angles
    Mat rod460, rod60, rod60460;              // Rotation matrices for Rodrigues
    short  bM11 = 0,                       // Marker 1 has been detected on this step
           bM21 = 0;                       //        2
    double euler2[3];                         // Euler angles from rotation matrices

    float dT[3];

    float Range = 0,
          Range_min = 999999990,
          Range_max = 0,
          Range_aver = 0,
          Angle_xyz_max[3] = {0,0,0},
          Angle_xyz_min[3] = {9999999,99999,99999},
          Angle_aver[3] = {0,0,0},
          aver_coeff[4] = {150,150,150,150},
          AnglePerc[4] = {0,0,0,0},
          Percent = 0,
//        thresholdPercent[4] = {0.3,0.3,0.3,0.3}, // NA
          thresholdDelta[4] = {0.3,0.3,0.3,6};     // thresholds for activation
    float beyondCoeff_default = 0.9;
    float beyondCoeff = beyondCoeff_default;

    int bBeyond = 0;    // pos > threshold 
    int bTrigd = 0;
    int bAdaptive = 6;
    int bOutmode = 0;
    int bMovemode = 0;
  	int bVideowrite = 0;
    int bwritetofile = 0;
    char arg[] = "../video/test_video.avi";

  	struct timeval tp;
    gettimeofday(&tp, NULL);
  	long int ms_start = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    long int ms, ms_prev;
    ofstream myfile;

    int color1[3]      = { 0, 255, 0 },
        color2[3]      = { 255, 75, 75 },
        color3[3]      = { 255, 75, 75 },
        colorDef[3]    = { 255, 75, 75 },
        colorChosen[3] = { 0, 255, 0 };

    std::string shelp("\td: switch detection mode; 0-off 1-Rng 2-a0 3-a1 4-a2\n");
    shelp.append("\tf: adaptive mode on/off;\n");
    shelp.append("\tc: ranges = reset absolute thresholds vals;\n");
    shelp.append("\tt: print frames per second; on/off\n");
    shelp.append("\te: set exposure to 10\n");
    shelp.append("\to: set output mode:\n");
    shelp.append("\t      0: press 'space'\n");
    shelp.append("\t      1: move mouse\n");
    shelp.append("\t      N/A: hold 'space'\n");
    shelp.append("\tv: mouse mode\n");
    shelp.append("\t  ------\n");
    shelp.append("\t      0: one press\n");
    shelp.append("\t      1: hold\n");
    shelp.append("\t      2: off\n");
    shelp.append("\t  ------\n");
    shelp.append("\t      0: vertical\n");
    shelp.append("\t      1: horizontal\n");
    shelp.append("\t      2: 2d\n");
    shelp.append("\tw: write video\n");

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

            gettimeofday(&tp, NULL);
            ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;

            if(iNFrame%10 == 0 && bprint_fps == 1)
            {
                  cout << "fps:" << 1000/((double)ms - ms_prev)*10 << endl;
                  ms_prev = ms;
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
                  bwritetofile = 1; // !!!!PELIGRO!!!
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

                    if ( id == id_marker1 )
                    {
                        //cout << "==========" << id_marker1 << "===========" << endl;
                        cv::Rodrigues(TheMarkers[i].Rvec,rod60);       //Извлекаем матрицы поворотов для первого и второго маркера
                        cv::Rodrigues(TheMarkers[i].Rvec,rod60460);
                        bM11 = 1;
                    }
                    if ( id == id_marker2 )
                    {
                        //cout << "==========" << id_marker2 << "==========" << endl;
                        cv::Rodrigues(TheMarkers[i].Rvec,rod460);       //Извлекаем матрицы поворотов для первого и второго маркера
                        bM21 = 1;
                    }
                    if(id == id_marker1 || id == id_marker2)
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
                //anglesfromrod = rotationMatrixToEulerAngles(rod60460);

                if(bprint_angles)
                {
                  cout << "-_-_-_-_-_-_-_-_-_anglesfromrod: " << endl;

                  for(int i = 0; i < 4; i++)
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
                DrawCube2(TheInputImageCopy, TheMarkers, TheCameraParameters, Range, euler2);
  
                //------^^^^^^^Add cube and draw^^^^^--------------------
            }//if(bM11 && bM21)

            switch (key)
            {
              case 'd':
                {
                    if(bDetectMode == 4)
                        bDetectMode = 0;
                      else bDetectMode++;

                    for(int i = 0; i < 3; i++)
                    {
                          color1[i] = colorDef[i];
                          color2[i] = colorDef[i];
                          color3[i] = colorDef[i];

                          if(bDetectMode == 2)
                              color1[i] = colorChosen[i];
                            else if(bDetectMode == 3)
                              color2[i] = colorChosen[i];
                            else if(bDetectMode == 4)
                              color3[i] = colorChosen[i];
                    }
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
                  if(bprint_fps == 0)
                      bprint_fps = 1;
                    else
                      bprint_fps = 0;
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
                   bMovemode = 0;
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
              case 'q':
                {                
                    bwritetofile++;
                    if(bwritetofile == 2)
                        bwritetofile = 0;
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
            AnglePerc[3] = ((double)Range - (double)Range_min ) / ( (double)Range_max - (double)Range_min   );

            cv::rectangle(pic, cv::Point(0,0), cv::Point(250, 250), cv::Scalar(0,0,0), -1, CV_AA); // fill window with black 

            if(bM11 && bM21)
            {
                bTrigd = 0;
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

                        Percent = AnglePerc[3];     

                        if(bM11 && bM21)
                        {
                          if(bBeyond == 1)
                          {
                              if( thresholdDelta[3] < 0 && Range > Range_aver + beyondCoeff*thresholdDelta[3] || thresholdDelta[3] > 0 && Range < Range_aver + beyondCoeff*thresholdDelta[3])  
                                  bBeyond = 0;
                          }
                          else
                             if( thresholdDelta[3] < 0 && Range > Range_aver + thresholdDelta[3] || thresholdDelta[3] > 0 && Range < Range_aver + thresholdDelta[3])
                                bBeyond = 0; //!!!!PELIGRO!!!!!meaningless

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
                          if(bBeyond == 1)
                          {
                              if( thresholdDelta[0] < 0 && euler2[0] > Angle_aver[0] + beyondCoeff*thresholdDelta[0] || thresholdDelta[0] > 0 && euler2[0] < Angle_aver[0] + beyondCoeff*thresholdDelta[0])  
                                  bBeyond = 0;
                          }
                          else
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
                          if(bBeyond == 1)
                          {
                              if( thresholdDelta[1] < 0 && euler2[1] > Angle_aver[1] + beyondCoeff*thresholdDelta[1] || thresholdDelta[1] > 0 && euler2[1] < Angle_aver[1] + beyondCoeff*thresholdDelta[1])  
                                  bBeyond = 0;
                          }
                          else
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
                          if(bBeyond == 1)
                          {
                              if( thresholdDelta[2] < 0 && euler2[2] > Angle_aver[2] + beyondCoeff*thresholdDelta[2] || thresholdDelta[2] > 0 && euler2[2] < Angle_aver[2] + beyondCoeff*thresholdDelta[2])  
                                  bBeyond = 0;
                          }
                          else
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
                                          beyondCoeff = beyondCoeff_default;
                                          if(bTrigd == 1)
                                          {
                                               xdo_send_keysequence_window_down(xdoMain, CURRENTWINDOW, "space", 500);
                                               //xdo_send_keysequence_window(xdoMain, CURRENTWINDOW, "space", 1000);
                                               cout << "JUMP!!\n" << endl;
                                               bTrigd = 0;

                                               cv::putText(pic, "o", cvPoint(90,25), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(100,255,100), 1, CV_AA);
                                          }
                                          else
                                               xdo_send_keysequence_window_up(xdoMain, CURRENTWINDOW, "space", 50);
                                      }
                                      else 
                                        if(bMovemode == 1 && (bTrigd == 1 || bBeyond == 1) )
                                        {
                                              beyondCoeff = 1;
                                              xdo_send_keysequence_window_down(xdoMain, CURRENTWINDOW, "space", 500);
                                        }
                                          else
                                               xdo_send_keysequence_window_up(xdoMain, CURRENTWINDOW, "space", 50);
                                }
                            }
                           break;
                          case 1: // mouse
                            {
                              mouse_to_x = disp_res_x / 2;
                              mouse_to_y = disp_res_y / 2;

                              if(bMovemode == 0)                                                         //x
                                    mouse_to_x = Percent*1800;
                                else if(bMovemode == 1)                                                  //y
                                    mouse_to_y = Percent*900;
                                else if(bMovemode == 2)                                                  //template
                                {
                                    mouse_to_x = Percent*1800;
                                    mouse_to_y = Percent*900;
                                }

                              if (bDetectMode != 0 && bM11 && bM21)
                                  {
                                      xdo_move_mouse(xdoMain,mouse_to_x,mouse_to_y,0);
                                  }
                              //int ret = system("xdotool mousemove 800 400; xdotool click 1; xdotool key space");    
                            }
                              break;
                        }

                  if(bwritetofile == 1) //!!!PELIGRO!!!!
                  {
                        myfile.open("/home/caldor/gazetrack/realtimeplot/example2.txt", std::ios_base::app);
                        myfile << (ms-ms_start)/1000 << " " << euler2[0] << " " << euler2[1] << " " << euler2[2] << "\n";
                        myfile.close();
                  }
            }//if(bM11 && bM21)
            else
                printf("N\A\n");


            //show input with augmented information and  the thresholded image
             cv::imshow("in",TheInputImageCopy);
             cv::imshow("thres",MDetector.getThresholdedImage());


            //DrawBars&etc
            DrawBar( pic, 240, 250, Range, (double) Range_min, (double) Range_max, Range_aver, thresholdDelta[3] );
            DrawBar( pic, 135, 145, euler2[0], (double) Angle_xyz_min[0], (double) Angle_xyz_max[0], Angle_aver[0], thresholdDelta[0] );
            DrawBar( pic, 150, 160, euler2[1], (double) Angle_xyz_min[1], (double) Angle_xyz_max[1], Angle_aver[1], thresholdDelta[1] );
            DrawBar( pic, 165, 175, euler2[2], (double) Angle_xyz_min[2], (double) Angle_xyz_max[2], Angle_aver[2], thresholdDelta[2] );
            DrawBar_2( pic, 180, 190, euler2[0], (double) Angle_xyz_min[0], (double) Angle_xyz_max[0], Angle_aver[0], thresholdDelta[0], color1 );
            DrawBar_2( pic, 195, 205, euler2[1], (double) Angle_xyz_min[1], (double) Angle_xyz_max[1], Angle_aver[1], thresholdDelta[1], color2 );
            DrawBar_2( pic, 210, 215, euler2[2], (double) Angle_xyz_min[2], (double) Angle_xyz_max[2], Angle_aver[2], thresholdDelta[2], color3 );
            char text_Mode[4];
            sprintf(text_Mode,"M:%d",bMovemode);
            cv::putText(pic, text_Mode, cvPoint(10,35), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255,255,255), 1, CV_AA);
            sprintf(text_Mode,"O:%d",bOutmode);
            cv::putText(pic, text_Mode, cvPoint(10,55), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255,255,255), 1, CV_AA);
            sprintf(text_Mode,"A:%d",bAdaptive);
            cv::putText(pic, text_Mode, cvPoint(10,75), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255,255,255), 1, CV_AA);
            cv::putText(pic, "0", cvPoint(135,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);
            cv::putText(pic, "1", cvPoint(150,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);
            cv::putText(pic, "2", cvPoint(165,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);
            cv::putText(pic, "R", cvPoint(240,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,255,255), 1, CV_AA);
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
/*    if (TheCameraParameters.isValid())
        for (unsigned int i=0; i<TheMarkers.size(); i++)
            CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
*/
  //  cv::imshow("in",TheInputImageCopy);
  //  cv::imshow("thres",MDetector.getThresholdedImage());
}
