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

std::string Replace(std::string String1, std::string String2, std::string String3)
{
	std::string::size_type  Pos(String1.find(String2));

	while (Pos != std::string::npos)
	{
		String1.replace(Pos, String2.length(), String3);
		Pos = String1.find(String2, Pos + String3.length());
	}

	return String1;
}

void CalibrateCamera::Calibrate()
{
	std::string imgdirpath;
	std::cout <<  "�L�����u���[�V�����p�摜���ۑ�����Ă���t�H���_���w�肵�Ă��������B" << std::endl;
	//std::cin >> imgdirpath;
	std::cin.ignore();
	std::getline(std::cin, imgdirpath);
	imgdirpath= Replace(imgdirpath,"\"", "");
	Calibrate_FromDir(imgdirpath);
}

void CalibrateCamera::Calibrate_FromDir(std::string imgdirpath)
{
	std::cout << "�L�����u���[�V�������J�n���܂��B" << std::endl;

	/****************************************/
	/* (1)�L�����u���[�V�����摜�̓ǂݍ���  */
	/****************************************/

	//�L�����u���[�V�����p�摜�̃��X�g
	std::vector<IplImage*> srcImg;
	std::vector<std::string> files = FileUtility::GetFilesFromDirectory(imgdirpath, "*");

	int IMAGE_NUM = files.size();

	std::cout << "�摜�ǂݍ��ݒ�..." << std::endl;

	for (int i = 0; i < files.size(); i++)
	{
		srcImg.push_back(cvLoadImage(files[i].c_str(), CV_LOAD_IMAGE_COLOR));
	}

	/***************************************************************/
	/* (3)�`�F�X�{�[�h�i�L�����u���[�V�����p�^�[���j�̃R�[�i�[���o */
	/***************************************************************/

	int  PAT_SIZE = PAT_ROW*PAT_COL;
	int  CHESS_SIZE = 24.0;      /* �p�^�[��1�}�X��1�ӃT�C�Y[mm] */
	int corner_count;

	CvSize patternSize = cvSize(PAT_COL, PAT_ROW);

	int foundNum = 0;
	std::vector<CvPoint2D32f> allCorners = std::vector<CvPoint2D32f>();
	CvPoint2D32f *corners = new CvPoint2D32f[files.size()*PAT_SIZE];
	int* pCount=new int[IMAGE_NUM];
	//cvNamedWindow("Calibration", CV_WINDOW_AUTOSIZE);
	for (int i = 0; i < files.size(); i++)
	{
		
		bool found = cvFindChessboardCorners(srcImg[i], patternSize, &corners[i * PAT_SIZE], &corner_count);
		if (found)
		{
			foundNum++;
		}
		else
		{
			std::cout << files[i] + " is invalid" << std::endl;
		}
		/*************************************************/
		/* (4)�R�[�i�[�ʒu���T�u�s�N�Z�����x�ɏC���C�`�� */
		/*************************************************/

		IplImage *srcGray = cvCreateImage(cvGetSize(srcImg[i]), IPL_DEPTH_8U, 1);
		cvCvtColor(srcImg[i], srcGray, CV_BGR2GRAY);
		cvFindCornerSubPix(srcGray, &corners[i * PAT_SIZE], corner_count,
			cvSize(3, 3), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
		cvDrawChessboardCorners(srcImg[i], patternSize, &corners[i * PAT_SIZE], corner_count, found);
		pCount[i] = corner_count;
		//cvShowImage("Calibration", srcImg[i]);
		//cvWaitKey(100);
		//cvWaitKey(0);

		//allCorners.insert(allCorners.end(), corners.begin(), corners.end());
	}

	int allPoints = foundNum * PAT_SIZE;

	//cvDestroyWindow("Calibration");

	//CvPoint2D32f *corners = new CvPoint2D32f[allPoints]; 
	CvMat imagePoints;
	CvMat pointCounts;

	cvInitMatHeader(&imagePoints, allPoints, 1, CV_32FC2, corners);
	cvInitMatHeader(&pointCounts, foundNum, 1, CV_32SC1, pCount);


	/**************************/
	/* (2)3������ԍ��W�̐ݒ� */
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
	/* (5)�����p�����[�^�C�c�݌W���̐��� */
	/*************************************/

	CvMat *intrinsic = cvCreateMat(3, 3, CV_32FC1);
	CvMat *rotation = cvCreateMat(1, 3, CV_32FC1);
	CvMat *translation = cvCreateMat(1, 3, CV_32FC1);
	CvMat *distortion = cvCreateMat(1, 4, CV_32FC1);

	std::cout << "�����p�����[�^�C�c�݌W�����蒆..." << std::endl;
	cvCalibrateCamera2(&objectPoints, &imagePoints, &pointCounts, cvSize(srcImg[0]->width, srcImg[0]->height), intrinsic, distortion);

	// (6)�O���p�����[�^�̐���
	CvMat sub_image_points, sub_object_points;
	int base = 0;

	std::cout << "�O���p�����[�^���蒆..." << std::endl;
	cvGetRows(&imagePoints, &sub_image_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	cvGetRows(&objectPoints, &sub_object_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	cvFindExtrinsicCameraParams2(&sub_object_points, &sub_image_points, intrinsic, distortion, rotation, translation);

	// (7)XML�t�@�C���ւ̏����o��
	CvFileStorage *fs;
	fs = cvOpenFileStorage("camera.xml", 0, CV_STORAGE_WRITE);
	cvWrite(fs, "intrinsic", intrinsic);
	cvWrite(fs, "rotation", rotation);
	cvWrite(fs, "translation", translation);
	cvWrite(fs, "distortion", distortion);
	cvReleaseFileStorage(&fs);

	for (int i = 0; i < IMAGE_NUM; i++) {
		cvReleaseImage(&srcImg[i]);
	}
	
	std::cout << "�L�����u���[�V�����I�����܂����B" << std::endl;

}

void CalibrateCamera::Calibrate_FromDir_Prototype(std::string imgdirpath)
{
	std::cout << "�L�����u���[�V�������J�n���܂��B" << std::endl;

	const int IMAGE_NUM = 135;           // �摜��
	int PAT_ROW = 7;              // �p�^�[���̍s�� 
	int PAT_COL = 10;             // �p�^�[���̗� 
	int PAT_SIZE = PAT_ROW * PAT_COL;
	//     const int ALL_POINTS = IMAGE_NUM * PAT_SIZE;
	float CHESS_SIZE = 23.0f;     // �p�^�[��1�}�X��1�ӃT�C�Y[mm]
	static CvSize patternSize = cvSize(PAT_COL, PAT_ROW);

	/****************************************/
	/* (1)�L�����u���[�V�����摜�̓ǂݍ���  */
	/****************************************/

	//�L�����u���[�V�����p�摜�̃��X�g
	std::vector<IplImage*> srcImg;
	std::vector<std::string> files = FileUtility::GetFilesFromDirectory(imgdirpath, "*");

	std::cout << "�摜�ǂݍ��ݒ�..." << std::endl;

	for (int i = 0; i < files.size(); i++)
	{
		srcImg.push_back(cvLoadImage(files[i].c_str(), CV_LOAD_IMAGE_COLOR));
	}

	/***************************************************************/
	/* (3)�`�F�X�{�[�h�i�L�����u���[�V�����p�^�[���j�̃R�[�i�[���o */
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
		/* (4)�R�[�i�[�ʒu���T�u�s�N�Z�����x�ɏC���C�`�� */
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
	/* (2)3������ԍ��W�̐ݒ� */
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
	/* (5)�����p�����[�^�C�c�݌W���̐��� */
	/*************************************/
	
	CvMat intrinsic = cvMat(3, 3, CV_32FC1);
	CvMat distortion = cvMat(1, 4, CV_32FC1);
	CvMat rotation = cvMat(foundNum, 3, CV_64FC1);
	CvMat translation = cvMat(foundNum, 3, CV_64FC1);
	cvCalibrateCamera2(&objectPoints, &imagePoints, &pointCounts, cv::Size(srcImg[0]->width, srcImg[0]->height), &intrinsic, &distortion, &rotation, &translation, NULL);

	/**************************************************/
	/* (6)�O���p�����[�^�̐��� (1���ڂ̉摜h�ɑ΂���) */
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
	/* ((7)XML�t�@�C���ւ̏����o�� */
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

	std::cout << "�L�����u���[�V�����I�����܂����B" << std::endl;

}


