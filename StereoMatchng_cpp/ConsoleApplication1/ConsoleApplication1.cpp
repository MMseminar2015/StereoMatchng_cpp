// ConsoleApplication1.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include "FileUtility.h"
#include "CalibrateCamera.h"
#include "SCsample.h"
#include "StereoMatching.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

//�R�}���h�̓���
int inputcommand();
//������̐�������
inline bool IsInteger(const std::string &str);

//�R�}���h�ꗗ
std::vector<std::string> Command
{
	"Calibration",		//�J�����̃L�����u���[�V����
	"Stereo Matching",	//�X�e���I�}�b�`���O
	"Stereo Calibrate",
	"Set StereoCameraParameter",
	"Stereo Matching2",
	"kkk",
	"Exit"
};

int main(int argc, const char* argv[])
{
	//opencv�̃e�X�g
	//cv::Mat redImg(cv::Size(320, 240), CV_8UC3, cv::Scalar(0, 0, 255));
	//cv::namedWindow("red", cv::WINDOW_AUTOSIZE);
	//cv::imshow("red", redImg);
	//cv::waitKey(0);
	//cv::destroyAllWindows();

	int comindex = inputcommand();

	std::cout << Command[comindex] << "�����s���܂��B" << std::endl;

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
			std::cout << "�L�����u���[�V�����p�摜���ۑ�����Ă���t�H���_���w�肵�Ă��������B" << std::endl;
			//std::cin >> imgdirpath;
			std::cin.ignore();
			std::getline(std::cin, imgdirpath);
			imgdirpath = FileUtility::Replace(imgdirpath, "\"", "");
			StereoMatching::StereoCalibrate(7, 10, imgdirpath, false, true);
			//StereoMatching::Matching("C:/stereo/rectify_toolbox_1029/left_rectified00.bmp", "C:/stereo/rectify_toolbox_1029/right_rectified00.bmp", "sgbm");
			break;

		case 3:
			std::cout << "�X�e���I�J�����̃p�����[�^xml�t�@�C�����w�肵�Ă�������" << std::endl;
			std::cin.ignore();
			std::getline(std::cin, stereoParamFilePath);
			//stereoParamFilePath_char = stereoParamFilePath.c_str();

			break;

		case 4:
			std::cout << "���̉摜" << std::endl;
			std::cin.ignore();
			std::getline(std::cin, leftimg);

			std::cout << "�E�̉摜" << std::endl;
			std::cin.ignore();
			std::getline(std::cin, rightimg);

		//	StereoMatching::Matching(leftimg, rightimg, "sgbm", 0, 0, 1., false, stereoParamFilePath_char);

			break;

		case 5:
			std::cout << "�L�����u���[�V�����p�摜���ۑ�����Ă���t�H���_���w�肵�Ă��������B" << std::endl;
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
	std::cout << "���s���鏈����I�����Ă��������B" << std::endl;
	for (int i = 0; i < Command.size(); i++)
	{
		std::cout << i << " : " + Command[i] << std::endl;
	}
	std::cin >> com;

	//���͕�����̐�������
	if (IsInteger(com))
	{
		int comindex=std::stoi(com);
		if (comindex > -1 && comindex < Command.size())
			return comindex;
		else
		{
			std::cout << "0����" << Command.size() - 1 << "�܂ł̐����l����͂��Ă��������B" << std::endl << std::endl;
			return inputcommand();
		}
	}
	else
	{
		std::cout << "���͂��ꂽ�����������l�ł͂���܂���B" << std::endl << std::endl;
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



