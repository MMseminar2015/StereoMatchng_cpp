#pragma once

#include "string.h"
#include "stdafx.h"
#include <opencv2\opencv.hpp>

/// <summary>
/// �J�����L�����u���[�V����
/// </summary>
/// <remarks>
/// http://opencv.jp/sample/camera_calibration.html #calibration
/// </remarks>
class CalibrateCamera
{
public:
	//#define IMAGE_NUM  (25)         /* �摜�� */
	static const int PAT_ROW = 7;     /* �p�^�[���̍s�� */
	static const int  PAT_COL = 10;   /* �p�^�[���̗� */


public:
	//�J�����L�����u���[�V�������s
	static void Calibrate();

	// �J�����L�����u���[�V��������
	static void Calibrate_FromDir(std::string imgdirpath);
	static void StereoCalibrate(cv::Mat *intrinsic, cv::Mat *distortion);

private:
	static void Calibrate_FromDir_Prototype(std::string imgdirpath);
	static void StereoCalibrate_Prototype();

};

