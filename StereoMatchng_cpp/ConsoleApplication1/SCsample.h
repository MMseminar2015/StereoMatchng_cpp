#pragma once

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
//#include "opencv2/imgcodecs.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;

class SCsample
{
public:

	Mat CameraMatrix[2], DistCoeffs[2];
	Mat R, T, F, E;
	Mat R1, R2, P1, P2, Q;
	Rect ValidRoi[2];
	Mat Rmap[2][2]; // イメージ変換の行列

	Size ImageSize;

	Size BoardSize;
	float SquareSize;

	vector<vector<Point3f>> ObjectPoints;
	vector<vector<Point2f>> ImagePoints[2];
	//vector<vector<Point2f>> ImagePoints[2];
	//vector<vector<Point3f>> ObjectPoints[2];

	vector<vector<Point3f>> MonoObjectPoints[2];
	vector<vector<Point2f>> MonoImagePoints[2];


	SCsample();
	~SCsample();
	SCsample(int boardwidth, int boardheight, float squaresize);


	int Matching(
		string img1_filepatth, string imd2_filepath,
		char* algorithm = "bm",
		int maxdisp_opt = 0,
		int blocksize_opt = 0,
		float scale_opt = 1.f,
		bool nodisplay = false,
		char* intrinsicfilename = 0,
		char* extrinsicfilename = 0,
		char* disparityfilename = 0,
		char* pointcloud_filename = 0);
	static void StereoCalibrate(
		const vector<string>& imagelist,
		Size boardSize,
		bool displayCorners = false,
		bool useCalibrated = true,
		bool showRectified = true);


	bool DetectObjectPoints(
		Mat img,
		Size imgSize,
		Size boardSize,		// チェッカーボードのサイズ
		float squareSize,	// チェッカーボードのスクエアサイズ
		vector<Point3f>& objectPoint,
		vector<Point2f>& corners);

	bool DetectObjectPointsForStereoCamera2(
		Mat img1, Mat img2,
		Size imgSize,
		Size boardSize,		// チェッカーボードのサイズ
		float squareSize,	// チェッカーボードのスクエアサイズ
		vector<vector<Point3f>> ObjectPoints[2],
		vector<vector<Point2f>> ImagePoints[2]);
	bool DetectObjectPointsForStereoCamera(Mat* img);
	bool CalibrateStereoCamera(vector<Mat*> &imgs, vector<Mat*>& RimgPairList);

	int MonoCalibrate(
		vector<vector<Point3f>> objectPoints,
		vector<vector<Point2f>> imagePoints,
		Size imageSize,
		Mat& cameraMatrix, Mat& distCoeffs,
		vector<Mat>& rvecs, vector<Mat>& tvecs);

	int StereoCalibrate2();
	double StereoCalibrate();

	int StereoRectify(Mat &img1, Mat &img2, Mat& rimg1, Mat& rimg2);

	int SetImageSize(Mat img);
	int SetBoardSize(int boardwidth, int boardheight);

	bool OutputExtrinsicParameter(std::string outputfile);
	bool InputExtrinsicParameter(std::string inputfile);
	bool InputIntrinsicParameter(std::string inputfile, Mat& cameraMatrix, Mat& distCoeffs);
	bool OutputIntrinsicParameter(std::string outputfile, Mat cameraMatrix, Mat distCoeffs);

	bool OutputRectifyParameter(std::string outputfile, Mat rmap0, Mat rmap1);
};