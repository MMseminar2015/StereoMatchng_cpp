#include "stdafx.h"
#include "StereoMatching.h"
#include "FileUtility.h"
#include "CalibrateCamera.h"

using namespace std;

StereoMatching::StereoMatching()
{
}

StereoMatching::~StereoMatching()
{
}

static int print_help_match()
{
	printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
	printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|var] [--blocksize=<block_size>]\n"
		"[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
		"[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n");
	return 0;
}

static int print_help_calib()
{
	cout <<
		" Given a list of chessboard images, the number of corners (nx, ny)\n"
		" on the chessboards, and a flag: useCalibrated for \n"
		"   calibrated (0) or\n"
		"   uncalibrated \n"
		"     (1: use cvStereoCalibrate(), 2: compute fundamental\n"
		"         matrix separately) stereo. \n"
		" Calibrate the cameras and display the\n"
		" rectified results along with the computed disparity images.   \n" << endl;
	cout << "Usage:\n ./stereo_calib -w board_width -h board_height [-nr /*dot not view results*/] <image list XML/YML file>\n" << endl;
	return 0;
}

static void saveXYZ(const char* filename, const Mat& mat)
{
	const double max_z = 1.0e4;
	FILE* fp;// = fopen(filename, "wt");
	fopen_s(&fp, filename, "wt");
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			Vec3f point = mat.at<Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
		}
	}
	fclose(fp);
}

int StereoMatching::Matching(
	string img1_filepath, string img2_filepath, char* algorithm, int numberOfDisparities, int SADWindowSize, float scale, bool no_display,
	char* intrinsic_filename, char* extrinsic_filename, char* disparity_filename, char* point_cloud_filename)
{

	const char* img1_filename = 0;
	const char* img2_filename = 0;
	//const char* intrinsic_filename = 0;
	//const char* extrinsic_filename = 0;
	//const char* disparity_filename = 0;
	//const char* point_cloud_filename = 0;

	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3 };
	int alg = STEREO_BM;

	StereoBM bm;
	StereoSGBM sgbm;
	StereoVar var;


	img1_filename = img1_filepath.c_str();
	img2_filename = img2_filepath.c_str();

	alg = strcmp(algorithm, "bm") == 0 ? STEREO_BM :
		strcmp(algorithm, "sgbm") == 0 ? STEREO_SGBM :
		strcmp(algorithm, "hh") == 0 ? STEREO_HH :
		strcmp(algorithm, "var") == 0 ? STEREO_VAR : -1;
	if (alg < 0)
	{
		printf("Command-line parameter error: Unknown stereo algorithm\n\n");
		print_help_match();
		return -1;
	}

	if (numberOfDisparities != 0 && (numberOfDisparities < 1 || numberOfDisparities % 16 != 0))
	{
		printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
		print_help_calib();
		return -1;
	}

	if (SADWindowSize != 0 && (SADWindowSize < 1 || SADWindowSize % 2 != 1))
	{
		printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
		return -1;
	}

	if (scale < 0)
	{
		printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
		return -1;
	}


	if (!img1_filename || !img2_filename)
	{
		printf("Command-line parameter error: both left and right images must be specified\n");
		return -1;
	}

	if ((intrinsic_filename != 0) ^ (extrinsic_filename != 0))
	{
		printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
		return -1;
	}

	if (extrinsic_filename == 0 && point_cloud_filename)
	{
		printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
		return -1;
	}

	int color_mode = alg == STEREO_BM ? 0 : -1;
	Mat img1 = imread(img1_filename, color_mode);
	Mat img2 = imread(img2_filename, color_mode);

	if (scale != 1.f)
	{
		Mat temp1, temp2;
		int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
		resize(img1, temp1, Size(), scale, scale, method);
		img1 = temp1;
		resize(img2, temp2, Size(), scale, scale, method);
		img2 = temp2;
	}

	Size img_size = img1.size();

	Rect roi1, roi2;
	Mat Q;

	if (intrinsic_filename)
	{
		// reading intrinsic parameters
		FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", intrinsic_filename);
			return -1;
		}

		Mat M1, D1, M2, D2;
		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;

		M1 *= scale;
		M2 *= scale;

		fs.open(extrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", extrinsic_filename);
			return -1;
		}

		Mat R, T, R1, P1, R2, P2;
		fs["R"] >> R;
		fs["T"] >> T;

		cv::stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

		Mat map11, map12, map21, map22;
		cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
		cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

		Mat img1r, img2r;
		remap(img1, img1r, map11, map12, INTER_LINEAR);
		remap(img2, img2r, map21, map22, INTER_LINEAR);

		img1 = img1r;
		img2 = img2r;
	}

	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

	bm.state->roi1 = roi1;
	bm.state->roi2 = roi2;
	bm.state->preFilterCap = 31;
	bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities = numberOfDisparities;
	bm.state->textureThreshold = 10;
	bm.state->uniquenessRatio = 15;
	bm.state->speckleWindowSize = 100;
	bm.state->speckleRange = 32;
	bm.state->disp12MaxDiff = 1;

	sgbm.preFilterCap = 63;
	sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

	int cn = img1.channels();

	sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = numberOfDisparities;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = bm.state->speckleWindowSize;
	sgbm.speckleRange = bm.state->speckleRange;
	sgbm.disp12MaxDiff = 1;
	sgbm.fullDP = alg == STEREO_HH;

	var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
	var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
	var.nIt = 25;
	var.minDisp = -numberOfDisparities;
	var.maxDisp = 0;
	var.poly_n = 3;
	var.poly_sigma = 0.0;
	var.fi = 15.0f;
	var.lambda = 0.03f;
	var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
	var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
	var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING;

	Mat disp, disp8;
	//Mat img1p, img2p, dispp;
	//copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
	//copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

	int64 t = getTickCount();
	if (alg == STEREO_BM)
		bm(img1, img2, disp);
	else if (alg == STEREO_VAR) {
		var(img1, img2, disp);
	}
	else if (alg == STEREO_SGBM || alg == STEREO_HH)
		sgbm(img1, img2, disp);
	t = getTickCount() - t;
	printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());

	//disp = dispp.colRange(numberOfDisparities, img1p.cols);
	if (alg != STEREO_VAR)
		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
	else
		disp.convertTo(disp8, CV_8U);
	if (!no_display)
	{
		namedWindow("left", 1);
		cv::imshow("left", img1);
		namedWindow("right", 1);
		cv::imshow("right", img2);
		namedWindow("disparity", 0);
		cv::imshow("disparity", disp8);
		printf("press any key to continue...");
		fflush(stdout);
		waitKey();
		printf("\n");
	}

	if (disparity_filename)
		imwrite(disparity_filename, disp8);

	if (point_cloud_filename)
	{
		printf("storing the point cloud...");
		fflush(stdout);
		Mat xyz;
		reprojectImageTo3D(disp, xyz, Q, true);
		saveXYZ(point_cloud_filename, xyz);
		printf("\n");
	}

	return 0;

}

void StereoMatching::StereoCalibrate(const vector<string>& imagelist, Size boardSize, bool displayCorners, bool useCalibrated, bool showRectified)
{
	//
	// 画像読み込み, チェッカーボード検出
	//
	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	const int maxScale = 2;
	const float squareSize = 1.68f;  // Set this to your actual square size
								   // ARRAY AND VECTOR STORAGE:

	vector<vector<Point2f>> imagePoints[2];
	vector<vector<Point3f>> objectPoints;
	Size imageSize;

	int i, j, k, nimages = (int)imagelist.size() / 2;

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	vector<string> goodImageList;

	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			const string& filename = imagelist[i * 2 + k];
			Mat img = imread(filename, 0);
			if (img.empty())
				break;
			if (imageSize == Size())
				imageSize = img.size();
			else if (img.size() != imageSize)
			{
				cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				break;
			}
			bool found = false;
			vector<Point2f>& corners = imagePoints[k][j];
			for (int scale = 1; scale <= maxScale; scale++)
			{
				Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, Size(), scale, scale);

				/* チェッカーボード検出 */
				found = findChessboardCorners(timg, boardSize, corners,
					CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
				if (found)
				{
					if (scale > 1)
					{
						Mat cornersMat(corners);
						cornersMat *= 1. / scale;
					}
					break;
				}
			}

			/* 検出されたコーナーを表示 */
			if (displayCorners)
			{
				cout << filename << endl;
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				cv::imshow("corners", cimg1);
				char c = (char)waitKey(500);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				putchar('.');

			/* チェッカーボードが検出できなかった場合次の画像ペアを処理 */
			if (!found)
				break;

			cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
					30, 0.01));
		}
		if (k == 2)
		{
			goodImageList.push_back(imagelist[i * 2]);
			goodImageList.push_back(imagelist[i * 2 + 1]);
			j++;
		}
	}
	cout << j << " pairs have been successfully detected.\n";
	nimages = j;
	if (nimages < 2)
	{
		cout << "Error: too little pairs to run the calibration\n";
		return;
	}


	//
	// objectPoints検出
	//
	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	objectPoints.resize(nimages);

	for (i = 0; i < nimages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
	}

	cout << "Running stereo calibration ...\n";

	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);

	Mat R, T, E, F;

	//
	// 左右のカメラの内部パラメータ計算
	// 
	vector<string> newimagelist[2];
	for (int i = 0; i < goodImageList.size(); i += 2) {
		newimagelist[0].push_back(goodImageList[i]);
		newimagelist[1].push_back(goodImageList[i + 1]);
	}

	CvMat
		*intrisic[2] = { cvCreateMat(3, 3, CV_32FC1), cvCreateMat(3, 3, CV_32FC1) },
		*dist[2] = { cvCreateMat(1, 4, CV_32FC1),cvCreateMat(1, 4, CV_32FC1)},
		*kn= cvCreateMat(1, 3, CV_32FC1), 
		*ln=cvCreateMat(1, 3, CV_32FC1);
	for (int i = 0; i < 2; i++) {
		string filename = "camera";
		filename += to_string(i) + ".xml";
		CalibrateCamera::Calibrate_FromFileNames(newimagelist[i], filename, intrisic[i], kn, ln, dist[i]);
		cameraMatrix[i] = intrisic[i];
		distCoeffs[i] = dist[i];
	}

	//
	// 内部、外部パラメータの計算
	// 初期値(cameraMatrix,distCoeffs)を与える必要あり
	//
	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5),
		CALIB_FIX_INTRINSIC +
		//CALIB_FIX_PRINCIPAL_POINT +
		//CALIB_FIX_ASPECT_RATIO +
		//CALIB_ZERO_TANGENT_DIST +
		CALIB_USE_INTRINSIC_GUESS +
		//CALIB_SAME_FOCAL_LENGTH +
		//CALIB_RATIONAL_MODEL +
		//CALIB_FIX_K3 +
		//CALIB_FIX_K4 +
		//CALIB_FIX_K5 +
		//CALIB_FIX_K6 +
		0);
	cout << "done with RMS error=" << rms << endl;

	// (7)XMLファイルへの書き出し
	FileStorage fs("extrinsic.xml", FileStorage::WRITE);
	if (fs.isOpened()) {
		write(fs, "rotation", R);
		write(fs, "translation", T);
		write(fs, "essential", E);
		write(fs, "fundamental", F);
		fs.release();
	}

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average epipolar err = " << err / npoints << endl;

	// save intrinsic parameters
	// カメラパラメータと歪みをyml形式で保存
	FileStorage fs1("intrinsics.yml", FileStorage::WRITE);
	if (fs1.isOpened())
	{
		fs1 << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs1.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";


	FileStorage fs2("camera0.xml", FileStorage::READ);
	if (fs2.isOpened())
	{
		fs2["intrinsic"] >> cameraMatrix[0];
		fs2["distortion"] >> distCoeffs[0];
		fs2.release();
	}

	FileStorage fs3("camera1.xml", FileStorage::READ);
	if (fs3.isOpened())
	{
		fs3["intrinsic"] >> cameraMatrix[1];
		fs3["distortion"] >> distCoeffs[1];
		fs3.release();
	}

	FileStorage fs4("extrinsic.xml", FileStorage::READ);
	if (fs4.isOpened())
	{
		fs4["rotation"] >> R;
		fs4["translation"] >> T;
	}


	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	cv::stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY
		+ 0,
		1, imageSize, &validRoi[0], &validRoi[1]);

	fs.open("extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = false;//fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	if (!showRectified)
		return;

	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)
	if (useCalibrated)
	{
		// we already computed everything
	}
	// OR ELSE HARTLEY'S METHOD
	else
		// use intrinsic parameters of each camera, but
		// compute the rectification transformation directly
		// from the fundamental matrix
	{
		vector<Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
		Mat H1, H2;
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}

	//Precompute maps for cv::remap()
	cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	for (i = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;
			remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
			if (useCalibrated)
			{
				Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
					cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
				rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
			}
		}

		if (!isVerticalStereo)
			for (j = 0; j < canvas.rows; j += 16)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (j = 0; j < canvas.cols; j += 16)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		cv::imshow("rectified", canvas);
		char c = (char)waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
}

//static bool readStringList(const string& filename, vector<string>& l)
//{
//	l.resize(0);
//	FileStorage fs(filename, FileStorage::READ);
//	if (!fs.isOpened())
//		return false;
//	FileNode n = fs.getFirstTopLevelNode();
//	if (n.type() != FileNode::SEQ)
//		return false;
//	FileNodeIterator it = n.begin(), it_end = n.end();
//	for (; it != it_end; ++it)
//		l.push_back((string)*it);
//	return true;
//}

int StereoMatching::StereoCalibrate(int boardwidth, int boardheight, string imagelistfn, bool displayCorners, bool useCalibrated, bool showRectified) {

	Size boardSize;

	if (boardwidth <= 0 || boardheight <= 0)
	{
		cout << "invalid board width and height" << endl;
		return print_help_calib();
	}

	boardSize = Size(boardwidth, boardheight);

	vector<string> imagelist;
	imagelist = FileUtility::GetFilesFromDirectory(imagelistfn, "*.bmp");
	if (imagelist.size() < 1)
		cout << "invalid filepath" << endl;

	vector<string> newimagelist;
	newimagelist.resize(imagelist.size());
	for (int i = 0; i < imagelist.size(); i++) {
		if (i < imagelist.size() / 2)
			newimagelist[2 * i] = imagelist[i];
		else
			newimagelist[2 * (i - imagelist.size() / 2) + 1] = imagelist[i];
	}
	//imagelist = newimagelist;

	StereoCalibrate(imagelist, boardSize, displayCorners, useCalibrated, showRectified);
	return 0;

}

bool StereoMatching::DetectObjectPoints(
	Mat img, Size imgSize,
	Size boardSize, float squareSize,
	vector<Point3f>& objectPoint, vector<Point2f>& corners)
{

	if (img.empty())
		return false;
	bool found = false;
	for (int scale = 1; scale <= 2; scale++)
	{
		Mat timg;
		if (scale == 1)
			timg = img;
		else
			resize(img, timg, Size(), scale, scale);

		/* チェッカーボード検出 */
		found = findChessboardCorners(timg, boardSize, corners,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		if (found)
		{
			if (scale > 1)
			{
				Mat cornersMat(corners);
				cornersMat *= 1. / scale;
			}
			break;
		}
		else
			return false;
	}

	cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
			30, 0.01));

	for (int j = 0; j < boardSize.height; j++)
		for (int k = 0; k < boardSize.width; k++)
			objectPoint.push_back(Point3f(k*squareSize, j*squareSize, 0));

	return true;
}

int StereoMatching::MonoCalibrate(
	vector<vector<Point3f>> objectPoints, 
	vector<vector<Point2f>> imagePoints,
	Size imageSize,
	Mat& cameraMatrix, Mat& distCoeffs,
	vector<Mat>& rvecs, vector<Mat>& tvecs)
{
	return cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
}

int StereoMatching::StereoCalibrate2() {

	vector<vector<Point3f>> objectPoints[2];
	vector<vector<Point2f>> imagePoints[2];
	Size imageSize;

	Mat img1, img2;

	if (img1.size() != img2.size())
		;
	imageSize = img1.size();

	DetectObjectPointsForStereoCamera2(
		img1, img2, imageSize, BoardSize, 1.68, objectPoints, imagePoints);

	for (int i = 0; i < 2; i++)
	{
		Mat rvecs, tvecs;
		cv::calibrateCamera(objectPoints[i], imagePoints[i], imageSize, CameraMatrix[i], DistCoeffs[i], rvecs, tvecs);
	}

	double rms =
		cv::stereoCalibrate(
			objectPoints[0], imagePoints[0], imagePoints[1],
			CameraMatrix[0], DistCoeffs[0],
			CameraMatrix[1], DistCoeffs[1],
			imageSize,
			R, T, E, F,
			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5),
			CALIB_FIX_INTRINSIC +
			//CALIB_FIX_PRINCIPAL_POINT +
			//CALIB_FIX_ASPECT_RATIO +
			//CALIB_ZERO_TANGENT_DIST +
			CALIB_USE_INTRINSIC_GUESS +
			//CALIB_SAME_FOCAL_LENGTH +
			//CALIB_RATIONAL_MODEL +
			//CALIB_FIX_K3 +
			//CALIB_FIX_K4 +
			//CALIB_FIX_K5 +
			//CALIB_FIX_K6 +
			0);

	//double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
	//	cameraMatrix[0], distCoeffs[0],
	//	cameraMatrix[1], distCoeffs[1],
	//	imageSize, R, T, E, F,
	//	TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5),
	//	CALIB_FIX_INTRINSIC +
	//	//CALIB_FIX_PRINCIPAL_POINT +
	//	//CALIB_FIX_ASPECT_RATIO +
	//	//CALIB_ZERO_TANGENT_DIST +
	//	CALIB_USE_INTRINSIC_GUESS +
	//	//CALIB_SAME_FOCAL_LENGTH +
	//	//CALIB_RATIONAL_MODEL +
	//	//CALIB_FIX_K3 +
	//	//CALIB_FIX_K4 +
	//	//CALIB_FIX_K5 +
	//	//CALIB_FIX_K6 +
	//	0);
	//cout << "done with RMS error=" << rms << endl;

	//// (7)XMLファイルへの書き出し
	//FileStorage fs("extrinsic.xml", FileStorage::WRITE);
	//if (fs.isOpened()) {
	//	write(fs, "rotation", R);
	//	write(fs, "translation", T);
	//	write(fs, "essential", E);
	//	write(fs, "fundamental", F);
	//	fs.release();
	//}
	return 1;
}
int StereoMatching::StereoCalibrate() {

	// チェッカーボードコーナー検出
	while (1) {
		Mat img1, img2;
		vector<Point3f> objectPoints;
		vector<Point2f> imagePoints[2];
		bool found =
			DetectObjectPointsForStereoCamera(
				img1, img2, ImageSize, BoardSize, SquareSize,
				objectPoints, imagePoints[0], imagePoints[1]);
		if (found) {
			ObjectPoints.push_back(objectPoints);
			for (int i = 0; i < 2; i++)
				ImagePoints[i].push_back(imagePoints[i]);
		}
	}

	// 内部、外部パラメータの計算
	// 初期値(cameraMatrix,distCoeffs)を与える必要あり
	double rms =
		stereoCalibrate(
			ObjectPoints, ImagePoints[0], ImagePoints[1],
			CameraMatrix[0], DistCoeffs[0],
			CameraMatrix[1], DistCoeffs[1],
			ImageSize, R, T, E, F,
			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5),
			CALIB_FIX_INTRINSIC +
			//CALIB_FIX_PRINCIPAL_POINT +
			//CALIB_FIX_ASPECT_RATIO +
			//CALIB_ZERO_TANGENT_DIST +
			CALIB_USE_INTRINSIC_GUESS +
			//CALIB_SAME_FOCAL_LENGTH +
			//CALIB_RATIONAL_MODEL +
			//CALIB_FIX_K3 +
			//CALIB_FIX_K4 +
			//CALIB_FIX_K5 +
			//CALIB_FIX_K6 +
			0);

	cv::stereoRectify(CameraMatrix[0], DistCoeffs[0],
		CameraMatrix[1], DistCoeffs[1],
		ImageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY
		+ 0,
		1, ImageSize, &ValidRoi[0], &ValidRoi[1]);

	cv::initUndistortRectifyMap(CameraMatrix[0], DistCoeffs[0], R1, P1, ImageSize, CV_16SC2, Rmap[0][0], Rmap[0][1]);
	cv::initUndistortRectifyMap(CameraMatrix[1], DistCoeffs[1], R2, P2, ImageSize, CV_16SC2, Rmap[1][0], Rmap[1][1]);

}

bool StereoMatching::DetectObjectPointsForStereoCamera2(
	Mat img1, Mat img2,
	Size imgSize,
	Size boardSize,		// チェッカーボードのサイズ
	float squareSize,	// チェッカーボードのスクエアサイズ
	vector<vector<Point3f>> objectPoints[2],
	vector<vector<Point2f>> imagePoints[2]
	)
{
	if (img1.size() != img2.size())
		return false;

	bool found = true;

	for (int i = 0; i < 2; i++) {
		vector<Point3f> tempOP;
		vector<Point2f> tempC;
		if (DetectObjectPoints(img1, imgSize, boardSize, squareSize, tempOP, tempC)) {
			objectPoints[i].push_back(tempOP);
			imagePoints[i].push_back(tempC);
		}
		else
			found = false;
	}
	// 両方の画像からチェッカーボードが検出できた場合
	if (found)
		;
}

bool StereoMatching::DetectObjectPointsForStereoCamera(
	Mat img1, Mat img2,
	Size imgSize,
	Size boardSize,		// チェッカーボードのサイズ
	float squareSize,	// チェッカーボードのスクエアサイズ
	vector<Point3f>& ObjectPoints,
	vector<Point2f>& ImagePoints1, vector<Point2f>& ImagePoints2
	)
{
	if (img1.size() != img2.size())
		return false;

	vector<Point3f> tempOP;
	vector<Point2f> tempIP[2];
	for (int i = 0; i < 2; i++) {
		if (!DetectObjectPoints(img1, imgSize, boardSize, squareSize, tempOP, tempIP[i]))
			return false;
	}

	ObjectPoints = tempOP;
	ImagePoints1 = tempIP[0];
	ImagePoints2 = tempIP[1];
}

int StereoMatching::StereoRectify(Mat img1, Mat img2, Mat& rimg1, Mat& rimg2) {

	remap(img1, rimg1, Rmap[0][0], Rmap[0][1], INTER_LINEAR);
	remap(img2, rimg2, Rmap[1][0], Rmap[1][1], INTER_LINEAR);
	return 1;
}

int StereoMatching::SetImageSize(Mat img) {
	ImageSize = img.size();
	return 1;
}
int StereoMatching::SetBoardSize(int boardwidth, int boardheight) {
	if (boardwidth <= 0 || boardheight <= 0)
		return 0;
	BoardSize = Size(boardwidth, boardheight);
	return 1;
}


bool StereoMatching::OutputExtrinsicParameter(std::string outputfile) {
	// (7)XMLファイルへの書き出し
	FileStorage fs(outputfile, FileStorage::WRITE);
	if (fs.isOpened()) {
		write(fs, "rotation", R);
		write(fs, "translation", T);
		write(fs, "essential", E);
		write(fs, "fundamental", F);
		fs.release();
		return true;
	}
	return false;
}
bool StereoMatching::InputExtrinsicParameter(std::string inputfile) {
	FileStorage fs(inputfile, FileStorage::READ);
	if (fs.isOpened())
	{
		fs["rotation"] >> R;
		fs["translation"] >> T;
		fs["essential"] >> E;
		fs["fundamental"] >> F;
		fs.release();

		cv::stereoRectify(
			CameraMatrix[0], DistCoeffs[0],
			CameraMatrix[1], DistCoeffs[1],
			ImageSize, R, T,
			R1, R2, P1, P2, Q,
			CALIB_ZERO_DISPARITY
			+ 0,
			1, ImageSize, &ValidRoi[0], &ValidRoi[1]);

		//Precompute maps for cv::remap()
		cv::initUndistortRectifyMap(CameraMatrix[0], DistCoeffs[0], R1, P1, ImageSize, CV_16SC2, Rmap[0][0], Rmap[0][1]);
		cv::initUndistortRectifyMap(CameraMatrix[1], DistCoeffs[1], R2, P2, ImageSize, CV_16SC2, Rmap[1][0], Rmap[1][1]);

		return true;
	}
	return false;
}
bool StereoMatching::OutputIntrinsicParameter(std::string outputfile, Mat cameraMatrix, Mat distCoeffs) {
	FileStorage fs(outputfile, FileStorage::WRITE);
	if (fs.isOpened()) {
		write(fs, "cameraMatrix", cameraMatrix);
		write(fs, "distCoeffs", distCoeffs);
		fs.release();
		return true;
	}
	return false;
}
bool StereoMatching::InputIntrinsicParameter(std::string inputfile, Mat& cameraMatrix, Mat& distCoeffs) {
	FileStorage fs(inputfile, FileStorage::READ);
	if (fs.isOpened())
	{
		fs["cameraMatrix"] >> cameraMatrix;
		fs["distCoeffs"] >> distCoeffs;
		fs.release();
		return true;
	}
	return false;

}

bool StereoMatching::OutputRectifyParameter(std::string outputfile, Mat rmap0, Mat rmap1) {
	FileStorage fs(outputfile, FileStorage::WRITE);
	if (fs.isOpened()) {
		write(fs, "rmap0", rmap0);
		write(fs, "rmap1", rmap1);
		fs.release();
		return true;
	}
	return false;
}
