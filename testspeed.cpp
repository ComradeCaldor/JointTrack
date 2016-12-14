#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <stdlib.h>

#include <time.h>
#include <sys/time.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>

extern "C" {
#include <xdo.h>
};

using namespace cv;
using namespace aruco;

int icamera = 0;
int key = 0;
int waitTime=0;
string TheIntrinsicFile;

VideoCapture TheVideoCapturer;
Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;

xdo_t* xdoMain;

int main()
{
  int ret = system("v4l2-ctl -d /dev/video0 --set-ctrl power_line_frequency=2  --set-ctrl exposure_auto=1 --set-ctrl exposure_absolute=10;");
  sleep(0.5);

  xdoMain = xdo_new(NULL);
  int  mouse_to_x = 0;
  int  mouse_to_y = 0;

  cout << "process\n";
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms_start = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  long int ms, ms_prev;
  int iNFrame = 0;

  double tmpp = 0;
  char stext[100];

  TheVideoCapturer.open(icamera);
  waitTime=10;

  if (!TheVideoCapturer.isOpened()) {
      cerr<<"Could not open video"<<endl;
      return -1;

  }

  try
  {
    //read first image to get the dimensions
    TheVideoCapturer>>TheInputImage;

    //read camera parameters if passed
    if (TheIntrinsicFile!="") {
        TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
        TheCameraParameters.resize(TheInputImage.size());
    }

//    while ( key!=27 && TheVideoCapturer.grab())
    while ( iNFrame < 500 )// && TheVideoCapturer.grab())
    {
        iNFrame++;
      /*
        if(iNFrame%10 == 0)
        {
              gettimeofday(&tp, NULL);
              ms_prev = ms;
              ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
       // if(bprint_time_detection == 1 && iNFrame%10 == 0) //cout << ((double)ms - ms_start)/1000 << endl;
                cout << "fps:" << 1000/((double)ms - ms_prev)*10 << endl;
       }*/

      tmpp += 0.005;

//      cout << tmpp*1800 << endl;
      for(int i = 0; i < 1; i++)
      {
          tmpp = (rand() % 100)*0.01;
//          sprintf(stext, "xdotool mousemove %f 450 ;",tmpp*1800);
//          int ret = system(stext);

          mouse_to_x =  tmpp*1800;
          mouse_to_y = 450;

          //xdo_move_mouse(xdoMain,mouse_to_x,mouse_to_y,0);
//          xdo_send_keysequence_window(xdoMain, CURRENTWINDOW, "space", 2000);
//          xdo_send_keysequence_window_down(xdoMain, CURRENTWINDOW, "space", 1000);
          xdo_send_keysequence_window_up(xdoMain, CURRENTWINDOW, "space", 500);

//        execl("/bin/ls", "/bin/ls", "-r", "-t", "-l", (char *) 0);
//          execl("/usr/local/bin/xdotool","/usr/local/bin/xdotool", "mousemove", "800", "450", (char *) 0);
          /*pid_t pid = fork();

          if (pid == 0)
          {
              int ret = system(stext);
              return 0;
          }
          else if (pid > 0)
          {
              ;
          }*/
      }

      key=cv::waitKey(waitTime);
    }
  } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }

  return 0;
}
