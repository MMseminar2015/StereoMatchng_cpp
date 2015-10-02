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
	static const std::string IMGS_PATH;
	
	static const int IMAGE_NUM = 135;           // �摜��
	static const int PAT_ROW = 7;              // �p�^�[���̍s�� 
	static const int PAT_COL = 10;             // �p�^�[���̗� 
	static const int PAT_SIZE = PAT_ROW * PAT_COL;
	//     const int ALL_POINTS = IMAGE_NUM * PAT_SIZE;
	static const float CHESS_SIZE;     // �p�^�[��1�}�X��1�ӃT�C�Y[mm]
	static const CvSize patternSize;

public:
	/// <summary>
	/// �J�����L�����u���[�V�������s
	/// </summary>
	static void Calibrate(std::string imgdirpath);

	

};

