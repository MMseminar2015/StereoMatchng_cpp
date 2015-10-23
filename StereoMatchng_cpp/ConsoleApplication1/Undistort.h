#pragma once

#include "stdafx.h"
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

class Undistort
{
public:
	static void Undistortion(std::vector<std::string> files, cv::Mat cameraMatrix, cv::Mat distCoeffs);
};

