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

