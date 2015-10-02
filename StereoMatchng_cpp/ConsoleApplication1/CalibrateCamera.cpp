#include "stdafx.h"
#include "CalibrateCamera.h"
#include <iostream>
#include <windows.h>
#include <filesystem>
#include <opencv2\opencv.hpp>

const std::string CalibrateCamera::IMGS_PATH = "imgs\\{0:D3}.JPG";
const float CalibrateCamera::CHESS_SIZE = 23.0f;
const CvSize CalibrateCamera::patternSize = cvSize(PAT_COL, PAT_ROW);

CalibrateCamera::CalibrateCamera()
{
}


CalibrateCamera::~CalibrateCamera()
{
}

void CalibrateCamera::Calibrate(std::string imgdirpath)
{
	// (1)キャリブレーション画像の読み込み
	//IplImage *srcImg[IMAGE_NUM];
	//for (int i = 0; i < IMAGE_NUM; i++)
	//{
	//	std::string path = IMGS_PATH;// string.Format(IMGS_PATH, i);	//変更必要
	//	
	//	const char *pathchar = path.c_str();
	//	srcImg[i] = cvLoadImage(pathchar, CV_LOAD_IMAGE_COLOR);
	//}

	std::vector<IplImage*> srcImg;
	std::vector<std::string> files=GetFilesFromDirectory(imgdirpath, "*");
	
	for (int i = 0; i < files.size(); i++)
	{
		srcImg.push_back(cvLoadImage(files[i].c_str(), CV_LOAD_IMAGE_COLOR));
	}


	//// (3)チェスボード（キャリブレーションパターン）のコーナー検出
	//int foundNum = 0;
	//std::vector<CvPoint2D32f> allCorners = std::vector<CvPoint2D32f>();
	//int pCount[IMAGE_NUM];
	//using (CvWindow window = new CvWindow("Calibration", WindowMode.AutoSize))
	//{
	//	for (int i = 0; i < IMAGE_NUM; i++)
	//	{
	//		CvPoint2D32f[] corners;
	//		bool found = Cv.FindChessboardCorners(srcImg[i], patternSize, out corners);
	//		Debug.Print("{0:D2}...", i);
	//		if (found)
	//		{
	//			Debug.Print("ok");
	//			foundNum++;
	//		}
	//		else
	//		{
	//			Debug.Print("fail");
	//		}
	//		// (4)コーナー位置をサブピクセル精度に修正，描画                    
	//		using (IplImage srcGray = new IplImage(srcImg[i].Size, BitDepth.U8, 1))
	//		{
	//			Cv.CvtColor(srcImg[i], srcGray, ColorConversion.BgrToGray);
	//			Cv.FindCornerSubPix(srcGray, corners, corners.Length, new CvSize(3, 3), new CvSize(-1, -1), new CvTermCriteria(20, 0.03));
	//			Cv.DrawChessboardCorners(srcImg[i], patternSize, corners, found);
	//			pCount[i] = corners.Length;
	//			window.ShowImage(srcImg[i]);
	//			Cv.WaitKey(100);
	//		}
	//		allCorners.AddRange(corners);
	//	}
	//}
	//int allPoints = foundNum * PAT_SIZE;

	//// (2)3次元空間座標の設定
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

	//// (5)内部パラメータ，歪み係数の推定
	//CvMat intrinsic = new CvMat(3, 3, MatrixType.F32C1);
	//CvMat distortion = new CvMat(1, 4, MatrixType.F32C1);
	//CvMat rotation = new CvMat(foundNum, 3, MatrixType.F64C1);
	//CvMat translation = new CvMat(foundNum, 3, MatrixType.F64C1);
	//Cv.CalibrateCamera2(objectPoints, imagePoints, pointCounts, srcImg[0].Size, intrinsic, distortion, rotation, translation, CalibrationFlag.Default);

	//// (6)外部パラメータの推定 (1枚目の画像hに対して)
	///*
	//CvMat subImagePoints, subObjectPoints;
	//Cv.GetRows(imagePoints, out subImagePoints, 0, PAT_SIZE);
	//Cv.GetRows(objectPoints, out subObjectPoints, 0, PAT_SIZE);
	//CvMat rotation_ = new CvMat(1, 3, MatrixType.F32C1);
	//CvMat translation_ = new CvMat(1, 3, MatrixType.F32C1);

	//Cv.FindExtrinsicCameraParams2(subObjectPoints, subImagePoints, intrinsic, distortion, rotation_, translation_, false);
	////Cv.FindExtrinsicCameraParams2_(subObjectPoints, subImagePoints, intrinsic, distortion, rotation_, translation_, false);

	//*/
	//// (7)XMLファイルへの書き出し
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

	//// 書き込んだファイルを表示
	//Console.WriteLine(File.ReadAllText("camera.xml"));
	//Console.Read();
}

std::vector<std::string> CalibrateCamera::GetFilesFromDirectory(std::string dirpath, const std::string& filter)
{
	WIN32_FIND_DATAA fd;

	std::string ss = dirpath + filter;
	HANDLE hFind = FindFirstFileA(ss.c_str(), &fd);

	//// 検索失敗
	//if (hFind == INVALID_HANDLE_VALUE)
	//{
	//	throw std::exception("util::Directory::GetFiles failed");
	//}

	std::vector<std::string> fileList;
	do
	{
		// フォルダは除く
		if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
			continue;
		// 隠しファイルは除く
		if (fd.dwFileAttributes & FILE_ATTRIBUTE_HIDDEN)
			continue;

		fileList.push_back(dirpath+fd.cFileName);
		std::cout << dirpath+fd.cFileName << std::endl;
	} while (FindNextFileA(hFind, &fd));

	FindClose(hFind);

	return fileList;

}
