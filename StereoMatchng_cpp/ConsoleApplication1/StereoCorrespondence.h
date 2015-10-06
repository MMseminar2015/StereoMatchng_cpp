#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
using namespace std;

class StereoCorrespondence
{
	cv::Mat LeftPic;
	cv::Mat RightPic;
public:
	StereoCorrespondence();
	StereoCorrespondence(string leftPicPath, string rightPicPath);
	~StereoCorrespondence();
};

