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

const std::string CalibrateCamera::IMGS_PATH = "imgs\\{0:D3}.JPG";
const float CalibrateCamera::CHESS_SIZE = 23.0f;
const CvSize CalibrateCamera::patternSize = cvSize(PAT_COL, PAT_ROW);



void CalibrateCamera::Calibrate(std::string imgdirpath)
{
	std::cout << "キャリブレーションを開始します。" << std::endl;

	/****************************************/
	/* (1)キャリブレーション画像の読み込み  */
	/****************************************/

	//キャリブレーション用画像のリスト
	std::vector<IplImage*> srcImg;
	std::vector<std::string> files = FileUtility::GetFilesFromDirectory(imgdirpath, "*");

	std::cout << "画像読み込み中..." << std::endl;

	for (int i = 0; i < files.size(); i++)
	{
		srcImg.push_back(cvLoadImage(files[i].c_str(), CV_LOAD_IMAGE_COLOR));
	}

	/***************************************************************/
	/* (3)チェスボード（キャリブレーションパターン）のコーナー検出 */
	/***************************************************************/
	
	int foundNum = 0;
	std::vector<CvPoint2D32f> allCorners = std::vector<CvPoint2D32f>();
	int pCount[IMAGE_NUM];
	cvNamedWindow("Calibration", CV_WINDOW_AUTOSIZE);
	for (int i = 0; i < files.size(); i++)
	{
		std::vector<CvPoint2D32f> corners;
		bool found = cv::findChessboardCorners(cv::cvarrToMat(srcImg[i]), patternSize, corners);
		if (found)
		{
			foundNum++;
		}
		else
		{
			std::cout << files[i] + " is invalid" << std::endl;
		}
		/*************************************************/
		/* (4)コーナー位置をサブピクセル精度に修正，描画 */
		/*************************************************/
		            
		IplImage* srcGray = cvCreateImage(cvSize(srcImg[i]->width, srcImg[i]->height), CV_8U, 1);

		cv::cvtColor(cv::cvarrToMat(srcImg[i]), cv::cvarrToMat(srcGray), CV_BGR2GRAY);
		cv::cornerSubPix(cv::cvarrToMat(srcImg[i]), corners, cv::Size(3, 3), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
		cv::drawChessboardCorners(cv::cvarrToMat(srcImg[i]), patternSize, corners, found);
		pCount[i] = corners.size();
		cvShowImage("Calibration", srcImg[i]);
		cv::waitKey(100);

		allCorners.insert(allCorners.end(), corners.begin(), corners.end());
	}

	int allPoints = foundNum * PAT_SIZE;

	/**************************/
	/* (2)3次元空間座標の設定 */
	/**************************/
	
	CvPoint3D32f *objects = new CvPoint3D32f[allPoints];
	for (int i = 0; i < foundNum; i++)
	{
		for (int j = 0; j < PAT_ROW; j++)
		{
			for (int k = 0; k < PAT_COL; k++)
			{
				objects[(i * PAT_SIZE) + (j * PAT_COL) + k] = cvPoint3D32f(j * CHESS_SIZE, k * CHESS_SIZE, 0.0f);
			}
		}
	}
	CvMat objectPoints = cvMat(allPoints, 3, CV_32FC1, objects);

	CvMat imagePoints = cvMat(allPoints, 1, CV_32FC2, allCorners.data());
	CvMat pointCounts = cvMat(foundNum, 1, CV_32SC1, pCount);

	/*************************************/
	/* (5)内部パラメータ，歪み係数の推定 */
	/*************************************/
	
	CvMat intrinsic = cvMat(3, 3, CV_32FC1);
	CvMat distortion = cvMat(1, 4, CV_32FC1);
	CvMat rotation = cvMat(foundNum, 3, CV_64FC1);
	CvMat translation = cvMat(foundNum, 3, CV_64FC1);
	cvCalibrateCamera2(&objectPoints, &imagePoints, &pointCounts, cv::Size(srcImg[0]->width, srcImg[0]->height), &intrinsic, &distortion, &rotation, &translation, NULL);

	/**************************************************/
	/* (6)外部パラメータの推定 (1枚目の画像hに対して) */
	/**************************************************/
	
	/*
	CvMat subImagePoints, subObjectPoints;
	Cv.GetRows(imagePoints, out subImagePoints, 0, PAT_SIZE);
	Cv.GetRows(objectPoints, out subObjectPoints, 0, PAT_SIZE);
	CvMat rotation_ = new CvMat(1, 3, MatrixType.F32C1);
	CvMat translation_ = new CvMat(1, 3, MatrixType.F32C1);

	Cv.FindExtrinsicCameraParams2(subObjectPoints, subImagePoints, intrinsic, distortion, rotation_, translation_, false);
	//Cv.FindExtrinsicCameraParams2_(subObjectPoints, subImagePoints, intrinsic, distortion, rotation_, translation_, false);

	*/

	/*******************************/
	/* ((7)XMLファイルへの書き出し */
	/*******************************/
	
	CvFileStorage *fs = cvOpenFileStorage("camera.xml", NULL, CV_STORAGE_WRITE);
	cvWrite(fs, "intrinsic", &intrinsic);
	cvWrite(fs, "distortion", &distortion);

	//  fs.Write("rotation", rotation);
	//   fs.Write("translation", translation);

	for each (IplImage* img in srcImg)
	{
		delete(img);
	}

	std::cout << "キャリブレーション終了しました。" << std::endl;

}


