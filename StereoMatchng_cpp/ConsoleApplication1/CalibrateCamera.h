#pragma once

#include "string.h"
#include "stdafx.h"
#include <opencv2\opencv.hpp>

/// <summary>
/// カメラキャリブレーション
/// </summary>
/// <remarks>
/// http://opencv.jp/sample/camera_calibration.html #calibration
/// </remarks>
class CalibrateCamera
{
public:
	//#define IMAGE_NUM  (25)         /* 画像数 */
	static const int PAT_ROW = 7;     /* パターンの行数 */
	static const int  PAT_COL = 10;   /* パターンの列数 */


public:
	//カメラキャリブレーション実行
	static void Calibrate();

	// カメラキャリブレーション処理
	static void Calibrate_FromDir(std::string imgdirpath);
	static void StereoCalibrate(cv::Mat *intrinsic, cv::Mat *distortion);

private:
	static void Calibrate_FromDir_Prototype(std::string imgdirpath);
	static void StereoCalibrate_Prototype();

};

