#include "stdafx.h"
#include "Undistort.h"
#include "stdafx.h"
#include "CalibrateCamera.h"
#include "FileUtility.h"
#include <iostream>
#include <windows.h>
#include <filesystem>
#include <opencv2\opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

void Undistort::Undistortion(std::vector<std::string> files, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
	std::vector<cv::Mat> srcImg;
	for (int i = 0; i < files.size(); i++)
	{
		srcImg.push_back(cv::imread(files[i]));
	}


	cv::Size imageSize;
	imageSize = srcImg[0].size();
	// ˜c‚Ý•â³

	std::vector<cv::Mat> undistImg;
	for (int i = 0; i < srcImg.size(); i++) {
		
		undistImg.push_back(cv::Mat(imageSize, CV_8UC1));
		cv::undistort(srcImg[i], undistImg[i], cameraMatrix, distCoeffs);

		std::string savedir = FileUtility::GetDirPath(files[i]);
		savedir += "\\und";

		if (savedir.find_last_of("\\") != savedir.size() - 1)
			savedir.append("\\");

		cv::imwrite(savedir + FileUtility::AddSuffix(FileUtility::GetFileName(files[i]),"_und"), undistImg[i]);
	}
}
