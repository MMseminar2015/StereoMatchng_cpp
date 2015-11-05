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
#include "Undistort.h"
#include "StereoMatching.h"



void CalibrateCamera::Calibrate(bool forstereo)
{
	std::string imgdirpath;
	std::cout << "キャリブレーション用画像が保存されているフォルダを指定してください。" << std::endl;
	//std::cin >> imgdirpath;
	std::cin.ignore();
	std::getline(std::cin, imgdirpath);
	imgdirpath = FileUtility::Replace(imgdirpath, "\"", "");
	if (!forstereo) {
		CvMat *intrinsic = cvCreateMat(3, 3, CV_32FC1);
		CvMat *rotation = cvCreateMat(1, 3, CV_32FC1);
		CvMat *translation = cvCreateMat(1, 3, CV_32FC1);
		CvMat *distortion = cvCreateMat(1, 4, CV_32FC1);
		Calibrate_FromDir(imgdirpath, intrinsic, rotation, translation, distortion);
	}
	else
	{
		std::vector<std::string> files = FileUtility::GetFilesFromDirectory(imgdirpath, "*");
		std::vector<std::string> leftfiles;
		std::vector<std::string> rightfiles;
		for (int i = 0; i < files.size() / 2; i++)
		{
			leftfiles.push_back(files[2 * i]);
			rightfiles.push_back(files[2 * i + 1]);
		}
		CvMat *intrinsicl = cvCreateMat(3, 3, CV_32FC1);
		CvMat *rotationl = cvCreateMat(1, 3, CV_32FC1);
		CvMat *translationl = cvCreateMat(1, 3, CV_32FC1);
		CvMat *distortionl = cvCreateMat(1, 4, CV_32FC1);
		CalibrateCamera::Calibrate_FromFileNames(leftfiles, "leftcamera.xml", intrinsicl, rotationl, translationl, distortionl);
		Undistort::Undistortion(leftfiles, intrinsicl, distortionl);


		CvMat *intrinsicr = cvCreateMat(3, 3, CV_32FC1);
		CvMat *rotationr = cvCreateMat(1, 3, CV_32FC1);
		CvMat *translationr = cvCreateMat(1, 3, CV_32FC1);
		CvMat *distortionr = cvCreateMat(1, 4, CV_32FC1);
		CalibrateCamera::Calibrate_FromFileNames(rightfiles, "rightcamera.xml", intrinsicr, rotationr, translationr, distortionr);
		Undistort::Undistortion(rightfiles, intrinsicr, distortionr);

	}
	
}



void CalibrateCamera::Calibrate_FromDir(
	std::string imgdirpath,
	CvMat *intrinsic,
	CvMat *rotation,
	CvMat *translation,
	CvMat *distortion)
{
	std::vector<std::string> files = FileUtility::GetFilesFromDirectory(imgdirpath, "*");
	CalibrateCamera::Calibrate_FromFileNames(files, "camera.xml",intrinsic, rotation, translation, distortion);
	
}

void CalibrateCamera::Calibrate_FromFileNames(
	std::vector<std::string> files,
	std::string savefile,
	CvMat *intrinsic,
	CvMat *rotation,
	CvMat *translation,
	CvMat *distortion,
	int base)
{
	std::cout << "キャリブレーションを開始します。" << std::endl;

	/****************************************/
	/* (1)キャリブレーション画像の読み込み  */
	/****************************************/

	//キャリブレーション用画像のリスト
	std::vector<IplImage*> srcImg;

	int IMAGE_NUM = files.size();

	std::cout << "画像読み込み中..." << std::endl;

	for (int i = 0; i < files.size(); i++)
	{
		srcImg.push_back(cvLoadImage(files[i].c_str(), CV_LOAD_IMAGE_COLOR));
	}

	/***************************************************************/
	/* (3)チェスボード（キャリブレーションパターン）のコーナー検出 */
	/***************************************************************/

	int  PAT_SIZE = PAT_ROW*PAT_COL;
	//double  CHESS_SIZE = 24.0;      /* パターン1マスの1辺サイズ[mm] */
	double  CHESS_SIZE = 16.8;      /* パターン1マスの1辺サイズ[mm] */
	int corner_count;

	CvSize patternSize = cvSize(PAT_COL, PAT_ROW);

	int foundNum = 0;
	std::vector<CvPoint2D32f> allCorners = std::vector<CvPoint2D32f>();
	//CvPoint2D32f *corners = new CvPoint2D32f[files.size()*PAT_SIZE];
	//int* pCount=new int[IMAGE_NUM];
	//cvNamedWindow("Calibration", CV_WINDOW_AUTOSIZE);

	//std::vector<CvPoint2D32f> corners = std::vector<CvPoint2D32f>(files.size()*PAT_SIZE);
	std::vector<int> pCount;
	for (int i = 0; i < files.size(); i++)
	{
		std::vector<CvPoint2D32f> corners = std::vector<CvPoint2D32f>(PAT_SIZE);
		int found = cvFindChessboardCorners(srcImg[i], patternSize, corners.data(), &corner_count);
		if (found)
		{
			foundNum++;

			/*************************************************/
			/* (4)コーナー位置をサブピクセル精度に修正，描画 */
			/*************************************************/

			IplImage *srcGray = cvCreateImage(cvGetSize(srcImg[i]), IPL_DEPTH_8U, 1);
			cvCvtColor(srcImg[i], srcGray, CV_BGR2GRAY);
			cvFindCornerSubPix(srcGray, corners.data(), corner_count,
				cvSize(3, 3), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
			cvDrawChessboardCorners(srcImg[i], patternSize, corners.data(), corner_count, found);
			pCount.push_back(corner_count);

			allCorners.insert(allCorners.end(), corners.begin(), corners.end());
		}
		else
		{
			std::cout << files[i] + " is invalid" << std::endl;
		}


		//cvShowImage("Calibration", srcImg[i]);
		//cvWaitKey(100);

		//allCorners.insert(allCorners.end(), corners.begin(), corners.end());
	}

	int allPoints = foundNum * PAT_SIZE;

	//cvDestroyWindow("Calibration");

	//CvPoint2D32f *corners = new CvPoint2D32f[allPoints]; 
	CvMat imagePoints;
	CvMat pointCounts;

	//cvInitMatHeader(&imagePoints, allPoints, 1, CV_32FC2, corners.data);
	//cvInitMatHeader(&pointCounts, foundNum, 1, CV_32SC1, pCount);

	cvInitMatHeader(&imagePoints, allPoints, 1, CV_32FC2, allCorners.data());
	cvInitMatHeader(&pointCounts, foundNum, 1, CV_32SC1, pCount.data());

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
				objects[i * PAT_SIZE + j * PAT_COL + k].x = j * CHESS_SIZE;
				objects[i * PAT_SIZE + j * PAT_COL + k].y = k * CHESS_SIZE;
				objects[i * PAT_SIZE + j * PAT_COL + k].z = 0.0;
			}
		}
	}

	CvMat objectPoints;
	cvInitMatHeader(&objectPoints, allPoints, 3, CV_32FC1, objects);



	/*************************************/
	/* (5)内部パラメータ，歪み係数の推定 */
	//http://opencv.jp/opencv-2.1/cpp/camera_calibration_and_3d_reconstruction.html
	/*************************************/

	////内部パラメータ行列
	////fx,fy:焦点距離（ピクセル単位） cx,cy:主点（通常は画像中心）スキューs=0,アスペクト比a=1と仮定
	////[fx 0 cx]
	////[0 fy cy]
	////[0 0 1]
	//intrinsic = cvCreateMat(3, 3, CV_32FC1);

	////回転ベクトル
	////ロドリゲスの回転公式参照
	////ベクトルの方向：回転軸　ベクトルの長さ：回転量
	//rotation = cvCreateMat(1, 3, CV_32FC1);

	////並進ベクトル
	//translation = cvCreateMat(1, 3, CV_32FC1);

	////歪み係数ベクトル
	//distortion = cvCreateMat(1, 4, CV_32FC1);

	std::cout << "内部パラメータ，歪み係数推定中..." << std::endl;
	cvCalibrateCamera2(&objectPoints, &imagePoints, &pointCounts, cvSize(srcImg[0]->width, srcImg[0]->height), intrinsic, distortion);

	// (6)外部パラメータの推定
	CvMat sub_image_points, sub_object_points;
	
	std::cout << "外部パラメータ推定中..." << std::endl;
	cvGetRows(&imagePoints, &sub_image_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	cvGetRows(&objectPoints, &sub_object_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	cvFindExtrinsicCameraParams2(&sub_object_points, &sub_image_points, intrinsic, distortion, rotation, translation);

	// (7)XMLファイルへの書き出し
	CvFileStorage *fs;
	fs = cvOpenFileStorage(savefile.c_str(), 0, CV_STORAGE_WRITE);
	cvWrite(fs, "intrinsic", intrinsic);
	cvWrite(fs, "rotation", rotation);
	cvWrite(fs, "translation", translation);
	cvWrite(fs, "distortion", distortion);
	cvReleaseFileStorage(&fs);

	for (int i = 0; i < IMAGE_NUM; i++) {
		cvReleaseImage(&srcImg[i]);
	}

	std::cout << "キャリブレーション終了しました。" << std::endl;

}

void CalibrateCamera::Calibrate_FromMat(
	CvMat objectPoints,
	CvMat imagePoints,
	CvMat pointCounts,
	CvSize imgsize,
	std::string savefile,
	CvMat *intrinsic,
	CvMat *rotation,
	CvMat *translation,
	CvMat *distortion)
{
	std::cout << "キャリブレーションを開始します。" << std::endl;

	
	int  PAT_SIZE = PAT_ROW*PAT_COL;
	
	/*************************************/
	/* (5)内部パラメータ，歪み係数の推定 */
	//http://opencv.jp/opencv-2.1/cpp/camera_calibration_and_3d_reconstruction.html
	/*************************************/

	////内部パラメータ行列
	////fx,fy:焦点距離（ピクセル単位） cx,cy:主点（通常は画像中心）スキューs=0,アスペクト比a=1と仮定
	////[fx 0 cx]
	////[0 fy cy]
	////[0 0 1]
	//intrinsic = cvCreateMat(3, 3, CV_32FC1);

	////回転ベクトル
	////ロドリゲスの回転公式参照
	////ベクトルの方向：回転軸　ベクトルの長さ：回転量
	//rotation = cvCreateMat(1, 3, CV_32FC1);

	////並進ベクトル
	//translation = cvCreateMat(1, 3, CV_32FC1);

	////歪み係数ベクトル
	//distortion = cvCreateMat(1, 4, CV_32FC1);

	std::cout << "内部パラメータ，歪み係数推定中..." << std::endl;
	cvCalibrateCamera2(&objectPoints, &imagePoints, &pointCounts,imgsize, intrinsic, distortion);

	// (6)外部パラメータの推定
	CvMat sub_image_points, sub_object_points;
	int base = 0;

	std::cout << "外部パラメータ推定中..." << std::endl;
	cvGetRows(&imagePoints, &sub_image_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	cvGetRows(&objectPoints, &sub_object_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	cvFindExtrinsicCameraParams2(&sub_object_points, &sub_image_points, intrinsic, distortion, rotation, translation);

	// (7)XMLファイルへの書き出し
	CvFileStorage *fs;
	fs = cvOpenFileStorage(savefile.c_str(), 0, CV_STORAGE_WRITE);
	cvWrite(fs, "intrinsic", intrinsic);
	cvWrite(fs, "rotation", rotation);
	cvWrite(fs, "translation", translation);
	cvWrite(fs, "distortion", distortion);
	cvReleaseFileStorage(&fs);


	std::cout << "キャリブレーション終了しました。" << std::endl;

}


void CalibrateCamera::Calibrate_FromDir_Prototype(std::string imgdirpath)
{
	std::cout << "キャリブレーションを開始します。" << std::endl;

	const int IMAGE_NUM = 135;           // 画像数
	int PAT_ROW = 7;              // パターンの行数 
	int PAT_COL = 10;             // パターンの列数 
	int PAT_SIZE = PAT_ROW * PAT_COL;
	//     const int ALL_POINTS = IMAGE_NUM * PAT_SIZE;
	float CHESS_SIZE = 23.0f;     // パターン1マスの1辺サイズ[mm]
	static CvSize patternSize = cvSize(PAT_COL, PAT_ROW);

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
	CvMat rotation = cvMat(foundNum, 3, CV_32FC1);
	CvMat translation = cvMat(foundNum, 3, CV_32FC1);
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


void CalibrateCamera::CalcExtrinsicParams(
	cv::Mat rotation1,
	cv::Mat translation1,
	cv::Mat rotation2,
	cv::Mat translation2,
	cv::Mat *rotation,
	cv::Mat *translation)
{
	cv::Mat R1, R2, R, r;

	//回転ベクトルを行列に変換
	cv::Rodrigues(rotation1, R1);
	cv::Rodrigues(rotation2, R2);
	
	R = R1*R2.t();
	//cv::Rodrigues(R, r);
	//*rotation = r.t();
	*rotation = R;

	//FileStorage fs("check.xml", FileStorage::WRITE);
	//fs << "R" << R << "translation2" << translation2;
	//fs.release();
	
	cv::Mat tmp = R*translation2.t();

	*translation = translation1.t() - tmp;
}

using namespace cv;
using namespace std;

void CalibrateCamera::StereoCalibrate(std::string undimgdir)
{
	int board_w = 10;
	int board_h = 7;
	CvSize board_sz = cvSize(board_w, board_h);
	int board_n = board_w*board_h;
	int numBoards = 0;

	vector<vector<Point3f> > object_points;
	vector<vector<Point2f> > imagePoints1, imagePoints2;
	vector<Point2f> corners1, corners2;

	vector<Point3f> obj;
	Mat gray1, gray2;

	for (int j = 0; j < board_n; j++)
	{
		obj.push_back(Point3f(j / board_w, j%board_w, 0.0f));
	}

	std::vector<cv::Mat> undistImgL;
	std::vector<cv::Mat> undistImgR;

	std::vector<std::string> goodimgL;
	std::vector<std::string> goodimgR;

	std::vector<std::string> files = FileUtility::GetFilesFromDirectory(undimgdir, "*");
	std::vector<std::string> leftfiles;
	std::vector<std::string> rightfiles;
	for (int i = 0; i < files.size() / 2; i++)
	{
		leftfiles.push_back(files[2 * i]);
		rightfiles.push_back(files[2 * i + 1]);
		undistImgL.push_back(cv::imread(leftfiles[i]));
		undistImgR.push_back(cv::imread(rightfiles[i]));
	}

	int success = 0, k = 0, num = 0;
	bool found1 = false, found2 = false;


	for (int i = 0;i<files.size() / 2; i++)
	{
		cv::Mat gray1, gray2;

		cvtColor(undistImgL[i], gray1, CV_BGR2GRAY);
		cvtColor(undistImgR[i], gray2, CV_BGR2GRAY);

		//チェッカーボード探し・キャリブレーション描画
		found1 = findChessboardCorners(undistImgL[i], board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		found2 = findChessboardCorners(undistImgR[i], board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found1) {
			cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray1, board_sz, corners1, found1);
		}
		if (found2) {
			cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray2, board_sz, corners2, found2);
		}
		//キャリブレーション結果表示
		//imshow("image1", gray1);
		//imshow("image2", gray2);

		if (found1 && found2) {
			k = waitKey(0);
		}
		if (k == 27) { //ESCキーでブレイク
			break;
		}
		if (found1 != 0 && found2 != 0) {
			imagePoints1.push_back(corners1);
			imagePoints2.push_back(corners2);
			object_points.push_back(obj);

			goodimgL.push_back(leftfiles[i]);
			goodimgR.push_back(rightfiles[i]);

			numBoards++;
		}
	}

	destroyAllWindows();
	printf("Starting Calibration\n");
	Mat CM1 = Mat(3, 3, CV_64FC1);
	Mat CM2 = Mat(3, 3, CV_64FC1);
	Mat D1, D2;
	Mat R, T, E, F;

	object_points.resize(numBoards, object_points[0]);
	stereoCalibrate(object_points, imagePoints1, imagePoints2,
		CM1, D1, CM2, D2, undistImgL[0].size(), R, T, E, F,
		cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
		CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);

	FileStorage fs1("mystereocalib.txt", FileStorage::WRITE);
	fs1 << "CM1" << CM1;
	fs1 << "CM2" << CM2;
	fs1 << "D1" << D1;
	fs1 << "D2" << D2;
	fs1 << "R" << R;
	fs1 << "T" << T;
	fs1 << "E" << E;
	fs1 << "F" << F;
	printf("Done Calibration\n");
	printf("Starting Rectification\n");
	Mat R1, R2, P1, P2, Q;
	stereoRectify(CM1, D1, CM2, D2, undistImgL[0].size(), R, T, R1, R2, P1, P2, Q);
	fs1 << "R1" << R1;
	fs1 << "R2" << R2;
	fs1 << "P1" << P1;
	fs1 << "P2" << P2;
	fs1 << "Q" << Q;
	printf("Done Rectification\n");
	printf("Applying Undistort\n");
	Mat map1x, map1y, map2x, map2y;
	Mat imgU1, imgU2;
	initUndistortRectifyMap(CM1, D1, R1, P1, undistImgL[0].size(), CV_32FC1, map1x, map1y);
	initUndistortRectifyMap(CM2, D2, R2, P2, undistImgR[0].size(), CV_32FC1, map2x, map2y);
	printf("Undistort complete\n");

	

	num = 0;
	while (1)
	{
		cv::Mat img1 = undistImgL[num];
		cv::Mat img2 = undistImgR[num];
		remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
		remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
		imshow("image1", imgU1);
		imshow("image2", imgU2);
		/*if (num == 0) {
			imwrite("unsdist_rectifyL.jpg", imgU1);
			imwrite("unsdist_rectifyR.jpg", imgU2);
		}*/
		k = waitKey(0);
		if (k == 27)
		{
			break;
		}
		num++;

	}
}

void CalibrateCamera::StereoCalibrate_Prototype()
{
	//numBoardsは左・右片方だけの数
	//チェッカーボードはdata2, 3の計25枚
	int numBoards = 17;
	int board_w = 10;
	int board_h = 7;
	CvSize board_sz = cvSize(board_w, board_h);
	int board_n = board_w*board_h;
	vector<vector<Point3f> > object_points;
	vector<vector<Point2f> > imagePoints1, imagePoints2;
	vector<Point2f> corners1, corners2;
	vector<Point3f> obj;
	Mat gray1, gray2;
	vector<cv::Mat> imgL, imgR;

	for (int j = 0; j < board_n; j++)
	{
		obj.push_back(Point3f(j / board_w, j%board_w, 0.0f));
	}

	//画像読み込み,vecter<Mat>imgLにL画像, vector<Mat>imgRにR画像
	for (int i = 0; i < numBoards; i++) {
		std::stringstream stream;
		std::stringstream stream2;
		stream << "non_undistort/1280_960/l" << i + 1 << ".JPG";
		stream2 << "non_undistort/1280_960/r" << i + 1 << ".JPG";
		std::string fileName = stream.str();
		std::string fileName2 = stream2.str();
		imgL.push_back(cv::imread(fileName));
		imgR.push_back(cv::imread(fileName2));
		cout << "Load checker image: " << fileName << endl;
		cout << "Load checker image: " << fileName2 << endl;
	}

	Mat cameraMatrix(3, 3, CV_64F);
	Mat distCoeffs(1, 5, CV_64F);

	// 1280*960
	cameraMatrix.at<double>(0, 0) = 763.551851863385;
	cameraMatrix.at<double>(0, 2) = 651.0497287068629;
	cameraMatrix.at<double>(1, 1) = 760.6083431310567;
	cameraMatrix.at<double>(1, 2) = 475.4119088803083;

	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(1, 0) = 0;
	cameraMatrix.at<double>(2, 0) = 0;
	cameraMatrix.at<double>(2, 1) = 0;
	cameraMatrix.at<double>(2, 2) = 1;
	distCoeffs.at<double>(0, 0) = -0.2783040792372516;
	distCoeffs.at<double>(0, 1) = 0.1490571653850003;
	distCoeffs.at<double>(0, 2) = 0.002587035974869547;
	distCoeffs.at<double>(0, 3) = 0.0006115474139258526;
	distCoeffs.at<double>(0, 4) = -0.06305566979276371;
	
	vector<Mat> undistImgL, undistImgR;
	cv::Mat tmp(imgL[0].rows, imgL[0].cols, CV_8UC1);

	// 歪み補正(L, R)
	for (int i = 0; i < numBoards; i++) {
		undistImgL.push_back(tmp);
		undistImgR.push_back(tmp);
		undistort(imgL[i], undistImgL[i], cameraMatrix, distCoeffs);
		undistort(imgR[i], undistImgR[i], cameraMatrix, distCoeffs);
	}

	//imshow("non_undistort", imgL[0]);
	//imshow("undistort", undistImgL[0]);
	waitKey(0);

	int success = 0, k = 0, num = 0;
	bool found1 = false, found2 = false;

	while (success < numBoards)
	{
		cv::Mat img1 = undistImgL[num];
		cv::Mat img2 = undistImgR[num];

		cvtColor(img1, gray1, CV_BGR2GRAY);
		cvtColor(img2, gray2, CV_BGR2GRAY);

		//チェッカーボード探し・キャリブレーション描画
		found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found1) {
			cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray1, board_sz, corners1, found1);
		}
		if (found2) {
			cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray2, board_sz, corners2, found2);
		}
		//キャリブレーション結果表示
		//imshow("image1", gray1);
		//imshow("image2", gray2);
		k = waitKey(10);

		if (found1 && found2) {
			k = waitKey(0);
		}
		if (k == 27) { //ESCキーでブレイク
			break;
		}
		if (found1 != 0 && found2 != 0) {
			imagePoints1.push_back(corners1);
			imagePoints2.push_back(corners2);
			object_points.push_back(obj);
			printf("Corners stored\n");
			success++;
			if (success >= numBoards) {
				break;
			}
		}
		num++;
	}

	destroyAllWindows();
	printf("Starting Calibration\n");
	Mat CM1 = Mat(3, 3, CV_64FC1);
	Mat CM2 = Mat(3, 3, CV_64FC1);
	Mat D1, D2;
	Mat R, T, E, F;

	object_points.resize(numBoards, object_points[0]);
	stereoCalibrate(object_points, imagePoints1, imagePoints2,
		CM1, D1, CM2, D2, undistImgL[0].size(), R, T, E, F,
		cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
		CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);

	FileStorage fs1("mystereocalib.txt", FileStorage::WRITE);
	fs1 << "CM1" << CM1;
	fs1 << "CM2" << CM2;
	fs1 << "D1" << D1;
	fs1 << "D2" << D2;
	fs1 << "R" << R;
	fs1 << "T" << T;
	fs1 << "E" << E;
	fs1 << "F" << F;
	printf("Done Calibration\n");
	printf("Starting Rectification\n");
	Mat R1, R2, P1, P2, Q;
	stereoRectify(CM1, D1, CM2, D2, imgL[0].size(), R, T, R1, R2, P1, P2, Q);
	fs1 << "R1" << R1;
	fs1 << "R2" << R2;
	fs1 << "P1" << P1;
	fs1 << "P2" << P2;
	fs1 << "Q" << Q;
	printf("Done Rectification\n");
	printf("Applying Undistort\n");
	Mat map1x, map1y, map2x, map2y;
	Mat imgU1, imgU2;
	initUndistortRectifyMap(CM1, D1, R1, P1, imgL[0].size(), CV_32FC1, map1x, map1y);
	initUndistortRectifyMap(CM2, D2, R2, P2, imgR[0].size(), CV_32FC1, map2x, map2y);
	printf("Undistort complete\n");

	num = 0;
	while (1)
	{
		cv::Mat img1 = undistImgL[num];
		cv::Mat img2 = undistImgR[num];
		remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
		remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
		imshow("image1", imgU1);
		//imshow("image2", imgU2);
		if (num == 0) {
			imwrite("unsdist_rectifyL.jpg", imgU1);
			imwrite("unsdist_rectifyR.jpg", imgU2);
		}
		k = waitKey(0);
		if (k == 27)
		{
			break;
		}
		num++;

	}
}

