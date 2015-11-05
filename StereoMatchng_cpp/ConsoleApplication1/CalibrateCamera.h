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
	static const int PAT_ROW = 6;     /* �p�^�[���̍s�� */
	static const int  PAT_COL = 8;   /* �p�^�[���̗� */


public:
	//�J�����L�����u���[�V�������s
	static void Calibrate(bool forstereo);

	// �J�����L�����u���[�V��������
	static void Calibrate_FromDir(
		std::string imgdirpath,
		CvMat *intrinsic,
		CvMat *rotation,
		CvMat *translation,
		CvMat *distortion);

	static void Calibrate_FromFileNames(
		std::vector<std::string> files,
		std::string savefile,
		CvMat *intrinsic,
		CvMat *rotation,
		CvMat *translation,
		CvMat *distortion,
		int base = 0);

	static void StereoCalibrate(std::string undimgdir);

	static void CalibrateCamera::CalcExtrinsicParams(
		cv::Mat rotation1,
		cv::Mat translation1,
		cv::Mat rotation2,
		cv::Mat translation2,
		cv::Mat *rotation,
		cv::Mat *translation);
	static void CalibrateCamera::Calibrate_FromMat(
		CvMat objectPoints,
		CvMat imagePoints,
		CvMat pointCounts,
		CvSize imgsize,
		std::string savefile,
		CvMat *intrinsic,
		CvMat *rotation,
		CvMat *translation,
		CvMat *distortion);


private:
	static void Calibrate_FromDir_Prototype(std::string imgdirpath);
	static void StereoCalibrate_Prototype();

};

