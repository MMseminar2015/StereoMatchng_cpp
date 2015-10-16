#pragma once

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
//#include "opencv2/imgcodecs.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;

class StereoMatching
{
public:

	StereoMatching();
	~StereoMatching();
	int Matching(
		string img1_filepatth, string imd2_filepath, char* algorithm = "bm", int maxdisp_opt = 0, int blocksize_opt = 0, float scale_opt = 1.f, bool nodisplay = false,
		char* intrinsicfilename = 0, char* extrinsicfilename = 0, char* disparityfilename = 0, char* pointcloud_filename = 0);
	void StereoCalibrate(const vector<string>& imagelist, Size boardSize, bool displayCorners = false, bool useCalibrated = true, bool showRectified = true);
	int Calibrate(int boardwidth, int boardheight, string imglistfn, bool showRectified = true);
};

