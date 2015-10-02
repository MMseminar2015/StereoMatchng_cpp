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
	// (1)�L�����u���[�V�����摜�̓ǂݍ���
	std::vector<IplImage*> srcImg;
	std::vector<std::string> files= FileUtility::GetFilesFromDirectory(imgdirpath, "*");
	
	for (int i = 0; i < files.size(); i++)
	{
		srcImg.push_back(cvLoadImage(files[i].c_str(), CV_LOAD_IMAGE_COLOR));
	}


	// (3)�`�F�X�{�[�h�i�L�����u���[�V�����p�^�[���j�̃R�[�i�[���o
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
		// (4)�R�[�i�[�ʒu���T�u�s�N�Z�����x�ɏC���C�`��                    
		IplImage* srcGray = cvCreateImage(cvSize(srcImg[i]->width, srcImg[i]->height), CV_8U, 1);

		cv::cvtColor(cv::cvarrToMat(srcImg[i]), cv::cvarrToMat(srcGray), CV_BGR2GRAY);
		cv::cornerSubPix(cv::cvarrToMat(srcImg[i]), corners,  cv::Size(3, 3), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
		cv::drawChessboardCorners(cv::cvarrToMat(srcImg[i]), patternSize, corners, found);
		pCount[i] = corners.size();
		cvShowImage("Calibration",srcImg[i]);
		cv::waitKey(100);
	
		allCorners.insert(allCorners.end(), corners.begin(), corners.end());
	}
	
	int allPoints = foundNum * PAT_SIZE;

	//// (2)3������ԍ��W�̐ݒ�
	//CvPoint3D32f[] objects = new CvPoint3D32f[allPoints];
	//for (int i = 0; i < foundNum; i++)
	//{
	//	for (int j = 0; j < PAT_ROW; j++)
	//	{
	//		for (int k = 0; k < PAT_COL; k++)
	//		{
	//			objects[(i * PAT_SIZE) + (j * PAT_COL) + k] = new CvPoint3D32f
	//			{
	//				X = j * CHESS_SIZE,
	//				Y = k * CHESS_SIZE,
	//				Z = 0.0f
	//			};
	//		}
	//	}
	//}
	//CvMat objectPoints = new CvMat(allPoints, 3, MatrixType.F32C1, objects);

	//CvMat imagePoints = new CvMat(allPoints, 1, MatrixType.F32C2, allCorners.ToArray());
	//CvMat pointCounts = new CvMat(foundNum, 1, MatrixType.S32C1, pCount);

	//// (5)�����p�����[�^�C�c�݌W���̐���
	//CvMat intrinsic = new CvMat(3, 3, MatrixType.F32C1);
	//CvMat distortion = new CvMat(1, 4, MatrixType.F32C1);
	//CvMat rotation = new CvMat(foundNum, 3, MatrixType.F64C1);
	//CvMat translation = new CvMat(foundNum, 3, MatrixType.F64C1);
	//Cv.CalibrateCamera2(objectPoints, imagePoints, pointCounts, srcImg[0].Size, intrinsic, distortion, rotation, translation, CalibrationFlag.Default);

	//// (6)�O���p�����[�^�̐��� (1���ڂ̉摜h�ɑ΂���)
	///*
	//CvMat subImagePoints, subObjectPoints;
	//Cv.GetRows(imagePoints, out subImagePoints, 0, PAT_SIZE);
	//Cv.GetRows(objectPoints, out subObjectPoints, 0, PAT_SIZE);
	//CvMat rotation_ = new CvMat(1, 3, MatrixType.F32C1);
	//CvMat translation_ = new CvMat(1, 3, MatrixType.F32C1);

	//Cv.FindExtrinsicCameraParams2(subObjectPoints, subImagePoints, intrinsic, distortion, rotation_, translation_, false);
	////Cv.FindExtrinsicCameraParams2_(subObjectPoints, subImagePoints, intrinsic, distortion, rotation_, translation_, false);

	//*/
	//// (7)XML�t�@�C���ւ̏����o��
	//using (CvFileStorage fs = new CvFileStorage("camera.xml", null, FileStorageMode.Write))
	//{
	//	fs.Write("intrinsic", intrinsic);
	//	//  fs.Write("rotation", rotation);
	//	//   fs.Write("translation", translation);
	//	fs.Write("distortion", distortion);
	//}

	//foreach(IplImage img in srcImg)
	//{
	//	img.Dispose();
	//}

	//// �������񂾃t�@�C����\��
	//Console.WriteLine(File.ReadAllText("camera.xml"));
	//Console.Read();
}


