// ConsoleApplication1.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "FileUtility.h"
#include "CalibrateCamera.h"
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
	"Stereo Matching"	//ステレオマッチング
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

	std::cout << Command[comindex]<<"を実行します。" << std::endl;

	switch (comindex)
	{
	case 0:
		CalibrateCamera::Calibrate();
	default:
		break;
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



