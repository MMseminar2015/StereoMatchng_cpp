#include "stdafx.h"
#include "StereoCorrespondence.h"


StereoCorrespondence::StereoCorrespondence()
{
}

StereoCorrespondence::StereoCorrespondence(string leftPicPath, string rightPicPath)
{
	LeftPic = cv::imread(leftPicPath, 0);
	RightPic = cv::imread(rightPicPath, 0);

	cv::Mat dispBM = cv::Mat(LeftPic.rows, RightPic.cols, CV_16S,1);
	cv::Mat dstBM = cv::Mat(LeftPic.rows, RightPic.cols, CV_8U,1);
	cv::StereoBM stateBM;
	cv::find
}

StereoCorrespondence::~StereoCorrespondence()
{
}
