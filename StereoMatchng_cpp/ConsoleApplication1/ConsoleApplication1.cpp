// ConsoleApplication1.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "FileUtility.h"
#include "CalibrateCamera.h"
#include "SCsample.h"
#include "StereoMatching.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

//コマンドの入力
int inputcommand();
//文字列の整数判定
inline bool IsInteger(const std::string &str);

//コマンド一覧
std::vector<std::string> Command
{
	"Calibration",		//カメラのキャリブレーション
	"Stereo Matching",	//ステレオマッチング
	"Stereo Calibrate",
	"Set StereoCameraParameter",
	"Stereo Matching2",
	"kkk",
	"Exit"
};

int main(int argc, const char* argv[])
{
	//opencvのテスト
	//cv::Mat redImg(cv::Size(320, 240), CV_8UC3, cv::Scalar(0, 0, 255));
	//cv::namedWindow("red", cv::WINDOW_AUTOSIZE);
	//cv::imshow("red", redImg);
	//cv::waitKey(0);
	//cv::destroyAllWindows();

	int comindex = inputcommand();

	std::cout << Command[comindex] << "を実行します。" << std::endl;

	std::string imgdirpath;
	string stereoParamFilePath;
	char* stereoParamFilePath_char;
	string leftimg, rightimg;
	vector<Mat*> imglist, rimglist;
	SCsample stereo = SCsample(7, 10, 1.68f);
	vector<string> imagelist;
	int a;
	char c;
	while (1) {
		switch (comindex)
		{
		case 0:
			CalibrateCamera::Calibrate(true);
		case 1:
			CalibrateCamera::StereoCalibrate("C:\\Users\\Ohara Kazuya\\Desktop\\FlyCap_pic\\und");

		case 2:
			std::cout << "キャリブレーション用画像が保存されているフォルダを指定してください。" << std::endl;
			//std::cin >> imgdirpath;
			std::cin.ignore();
			std::getline(std::cin, imgdirpath);
			imgdirpath = FileUtility::Replace(imgdirpath, "\"", "");
			StereoMatching::StereoCalibrate(7, 10, imgdirpath, false, true);
			//StereoMatching::Matching("C:/stereo/rectify_toolbox_1029/left_rectified00.bmp", "C:/stereo/rectify_toolbox_1029/right_rectified00.bmp", "sgbm");
			break;

		case 3:
			std::cout << "ステレオカメラのパラメータxmlファイルを指定してください" << std::endl;
			std::cin.ignore();
			std::getline(std::cin, stereoParamFilePath);
			//stereoParamFilePath_char = stereoParamFilePath.c_str();

			break;

		case 4:
			std::cout << "左の画像" << std::endl;
			std::cin.ignore();
			std::getline(std::cin, leftimg);

			std::cout << "右の画像" << std::endl;
			std::cin.ignore();
			std::getline(std::cin, rightimg);

		//	StereoMatching::Matching(leftimg, rightimg, "sgbm", 0, 0, 1., false, stereoParamFilePath_char);

			break;

		case 5:
			std::cout << "キャリブレーション用画像が保存されているフォルダを指定してください。" << std::endl;
			//std::cin >> imgdirpath;
			std::cin.ignore();
			std::getline(std::cin, imgdirpath);
			imgdirpath = FileUtility::Replace(imgdirpath, "\"", "");

			imagelist = FileUtility::GetFilesFromDirectory(imgdirpath,"*.bmp");
			
			for (int i = 0; i < imagelist.size()/2; i++) {
				imglist.push_back(new Mat[2]{ cv::imread(imagelist[2 * i]),cv::imread(imagelist[2 * i + 1]) });
			}

			stereo.SetImageSize(imglist[0][0]);
			stereo.CalibrateStereoCamera(imglist, rimglist);

			for (int i = 0; i < imglist.size(); i++) {
				Mat rimg[2];
				stereo.StereoRectify(imglist[i][0], imglist[i][1], rimg[0], rimg[1]);
				cv::imshow("0o", imglist[i][0]);
				cv::imshow("0", rimg[0]);
				cv::imshow("1", rimg[1]);
				c = (char)waitKey();
			}

			for (int i = 0; i < rimglist.size(); i++) {
				cv::imshow("0", rimglist[i][0]);
				cv::imshow("1", rimglist[i][1]);
				c = (char)waitKey();
			}
			//StereoMatching::Matching("C:/stereo/rectify_toolbox_1029/left_rectified00.bmp", "C:/stereo/rectify_toolbox_1029/right_rectified00.bmp", "sgbm");
			break;

		case  5:
			return 0;

		default:
			break;
		}
	}

	getchar();
	return 0;
}

int inputcommand()
{
	std::string com;
	std::cout << "実行する処理を選択してください。" << std::endl;
	for (int i = 0; i < Command.size(); i++)
	{
		std::cout << i << " : " + Command[i] << std::endl;
	}
	std::cin >> com;

	//入力文字列の整数判定
	if (IsInteger(com))
	{
		int comindex=std::stoi(com);
		if (comindex > -1 && comindex < Command.size())
			return comindex;
		else
		{
			std::cout << "0から" << Command.size() - 1 << "までの整数値を入力してください。" << std::endl << std::endl;
			return inputcommand();
		}
	}
	else
	{
		std::cout << "入力された文字が整数値ではありません。" << std::endl << std::endl;
		return inputcommand();
	}
}

inline bool IsInteger(const std::string &str)
{
	if (str.find_first_not_of("-0123456789 \t") != std::string::npos) {
		return false;
	}

	return true;
}



