//// FlyCapOpenCV.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
////
//
#include "stdafx.h"
//#include "FlyCapture2.h"
//#include "opencv2/highgui/highgui.hpp"
//#include <iostream>
//#include <sstream>
//
//using namespace FlyCapture2;
//using namespace cv;
//using namespace std;
//
//
//int ()
//{
//	//取得する動画サイズ(モード)とフレームレート設定
//	FlyCapture2::VideoMode vMode = VIDEOMODE_320x240YUV422;
//	FlyCapture2::FrameRate fRate = FRAMERATE_30;
//
//	//保存フォルダ指定
//	string forder = "C:\tmp";
//
//	FlyCapture2::BusManager busMgr;
//	FlyCapture2::Error error;
//
//	FlyCapture2::Image cam_image, rgb_image;
//	cv::Mat cv_image;
//
//	//カメラ認識
//	unsigned int numCams;
//	error = busMgr.GetNumOfCameras(&numCams);
//	if (error != PGRERROR_OK || numCams < 1) return -1;
//	cout << "Number of cameras detected: " << numCams << endl;
//
//	//カメラの入れ物
//	FlyCapture2::Camera** ppCams = new FlyCapture2::Camera*[numCams];
//
//	for (int i = 0; i < numCams; i++)
//	{
//		ppCams[i] = new FlyCapture2::Camera();
//		FlyCapture2::PGRGuid guid;
//		//カメラとコネクションを結ぶ
//		error = busMgr.GetCameraFromIndex(i, &guid);
//		if (error != PGRERROR_OK) return -1;
//		error = ppCams[i]->Connect(&guid);
//		if (error != PGRERROR_OK) return -1;
//		//動画サイズ(モード)とフレームレート設定
//		error = ppCams[i]->SetVideoModeAndFrameRate(vMode, fRate);
//		if (error != PGRERROR_OK) return -1;
//	}
//
//	//カメラから画像の転送を開始
//	error = Camera::StartSyncCapture(numCams, (const Camera**)ppCams);
//	if (error != PGRERROR_OK) return -1;
//
//	//namedWindow("Camera_0", CV_WINDOW_AUTOSIZE);
//	//namedWindow("Camera_1", CV_WINDOW_AUTOSIZE);
//	namedWindow("Camera_0", CV_WINDOW_AUTOSIZE);
//	namedWindow("Camera_1", CV_WINDOW_AUTOSIZE);
//
//	Mat image[2];
//
//	//カメラ画像取得ループ
//	while (1)
//	{
//		for (int i = 0; i < numCams; i++)
//		{
//			//frame取得
//			error = ppCams[i]->RetrieveBuffer(&cam_image);
//			if (error != PGRERROR_OK) return -1;
//
//			//rgbに変換
//			error = cam_image.Convert(FlyCapture2::PixelFormat::PIXEL_FORMAT_BGR, &rgb_image);
//			if (error != PGRERROR_OK) return -1;
//			cv_image = cv::Mat(rgb_image.GetRows(), rgb_image.GetCols(), CV_8UC3, rgb_image.GetData());
//
//			string window_name = "Camera_" + to_string(i);
//			cv::imshow(window_name, cv_image);
//		}
//
//		int key = cv::waitKey(15);
//		if (key != -1)
//		{
//			cvDestroyAllWindows();
//			break;
//		}
//	}
//
//
//	//終了処理
//	for (int i = 0; i < numCams; i++)
//	{
//		ppCams[i]->StopCapture();
//		ppCams[i]->Disconnect();
//		delete ppCams[i];
//	}
//	delete[] ppCams;
//
//	return 0;
//}
//
