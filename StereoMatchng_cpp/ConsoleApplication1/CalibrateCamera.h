#pragma once

/************************
カメラキャリブレーション
************************/

#include "string.h"
#include "stdafx.h"
#include <opencv2\opencv.hpp>

class CalibrateCamera
{

public: 
	static const std::string IMGS_PATH;
	
	static const int IMAGE_NUM = 135;           // 画像数
	static const int PAT_ROW = 7;              // パターンの行数 
	static const int PAT_COL = 10;             // パターンの列数 
	static const int PAT_SIZE = PAT_ROW * PAT_COL;
	//     const int ALL_POINTS = IMAGE_NUM * PAT_SIZE;
	static const float CHESS_SIZE;     // パターン1マスの1辺サイズ[mm]
	static const CvSize patternSize;

public:
	CalibrateCamera();
	~CalibrateCamera();
	static void Calibrate(std::string imgdirpath);
	static std::vector<std::string> GetFilesFromDirectory(std::string dirpath, const std::string& filter);

};

