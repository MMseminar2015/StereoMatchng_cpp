// ConsoleApplication1.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include "StereoMatching.h"

int main()
{
	StereoMatching stereo = StereoMatching();
	int l = stereo.Calibrate(7, 10, "C:/stereo/FlyCap_pic/");
	int k = stereo.Matching("C:/stereo/data/scene1.row3.colL.png", "C:/stereo/data/scene1.row3.colR.png","sgbm");
	if (k < 0) printf("Error\n");
    return 0;
}