// ConsoleApplication1.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "CalibrateCamera.h"

int main(int argc, const char* argv[])
{
	//opencv�̃e�X�g
	//cv::Mat redImg(cv::Size(320, 240), CV_8UC3, cv::Scalar(0, 0, 255));
	//cv::namedWindow("red", cv::WINDOW_AUTOSIZE);
	//cv::imshow("red", redImg);
	//cv::waitKey(0);
	//cv::destroyAllWindows();

	CalibrateCamera::CalibrateCamera::GetFilesFromDirectory("C:\\Users\\Ohara Kazuya\\Documents\\opencv-2.4.10\\build\\x64\\vc12\\lib\\","*.lib");

	getchar();
	return 0;
}

