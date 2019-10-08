#include <stdio.h>
#include <iostream>
#include <sstream>
#include <windows.h>
#include <MMsystem.h>
#include <random>
#include <Kinect.h>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#define FILENAME "tes.csv"
#define FILENAME3D "test3D.csv"
//#define display

// #define OBJECT //物体検出

#define B_MAX 50
#define B_MIN 0
#define G_MAX 50
#define G_MIN 0
#define R_MAX 255
#define R_MIN 150

#define H_MAX 90
#define H_MIN 70
#define S_MAX 255
#define S_MIN 50
#define V_MAX 205
#define V_MIN 80

// Visual Studio Professional以上を使う場合はCComPtrの利用を検討してください。
#include "ComPtr.h"
//#include <atlbase.h>
#define CRT_SECURE_NO_WARNINGS
#pragma warning(disable:4996)

#pragma comment( lib, "Winmm.lib" )
// 次のように使います
// ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );
// 書籍での解説のためにマクロにしています。実際には展開した形で使うことを検討してください。
#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss; \
        ss << "failed " #ret " " << std::hex << ret << std::endl;   \
        throw std::runtime_error( ss.str().c_str() );   \
		    }
using namespace cv;
using namespace std;

long getAllTime(int h, int m, int s, int ms)
{
	return h * 3600 * 1000 + m * 60 * 1000 + s * 1000 + ms;
}


class KinectApp
{
private:

	IKinectSensor* kinect = nullptr;
	IBodyFrameReader* bodyFrameReader = nullptr;
	IColorFrameReader* colorFrameReader = nullptr;
	IBody* bodies[6];

	std::vector<BYTE> colorBuffer;
	int colorWidth;
	int colorHeight;
	int saveflag = 0;
	unsigned int colorBytesPerPixel;

public:


	vector<vector<int>> inst;
	// 初期化
	void initialize()
	{
		// デフォルトのKinectを取得する
		ERROR_CHECK(::GetDefaultKinectSensor(&kinect));

		// Kinectを開く
		ERROR_CHECK(kinect->Open());

		BOOLEAN isOpen = false;
		ERROR_CHECK(kinect->get_IsOpen(&isOpen));
		if (!isOpen) {
			throw std::runtime_error("Kinectが開けません");
		}

		// ボディリーダーを取得する
		ComPtr<IBodyFrameSource> bodyFrameSource;
		ERROR_CHECK(kinect->get_BodyFrameSource(&bodyFrameSource));
		ERROR_CHECK(bodyFrameSource->OpenReader(&bodyFrameReader));
		/////////color

		// カラーリーダーを取得する
		ComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

		// カラー画像のサイズを取得する
		ComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK(colorFrameSource->CreateFrameDescription(
			ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription));
		ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
		ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));
		ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));

		// バッファーを作成する
		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);

		////////color

		// create instruction list

		//inst.reserve(setnum * 12);

		vector<int> L10 = { 1, 0 };
		vector<int> L11 = { 1, 1 };
		vector<int> L12 = { 1, 2 };
		vector<int> L20 = { 2, 0 };
		vector<int> L21 = { 2, 1 };
		vector<int> L22 = { 2, 2 };
		vector<int> L30 = { 3, 0 };
		vector<int> L31 = { 3, 1 };
		vector<int> L32 = { 3, 2 };
		vector<int> L40 = { 4, 0 };
		vector<int> L41 = { 4, 1 };
		vector<int> L42 = { 4, 2 };

		for (int i = 0; i < setnum; i++) {
			inst.push_back(L10);
			inst.push_back(L11);
			inst.push_back(L12);
			inst.push_back(L20);
			inst.push_back(L21);
			inst.push_back(L22);
			inst.push_back(L30);
			inst.push_back(L31);
			inst.push_back(L32);
			inst.push_back(L40);
			inst.push_back(L41);
			inst.push_back(L42);
		}
	}

	ColorSpacePoint pointH;
	ColorSpacePoint pointN;
	ColorSpacePoint pointSS;
	ColorSpacePoint pointSM;
	ColorSpacePoint pointSB;
	ColorSpacePoint pointSR;
	ColorSpacePoint pointER;
	ColorSpacePoint pointWR;
	ColorSpacePoint pointHR;
	ColorSpacePoint pointTR;
	ColorSpacePoint pointHTR;
	ColorSpacePoint pointSL;
	ColorSpacePoint pointEL;
	ColorSpacePoint pointWL;
	ColorSpacePoint pointHL;
	ColorSpacePoint pointTL;
	ColorSpacePoint pointHTL;
	ColorSpacePoint pointHiR;
	ColorSpacePoint pointKR;
	ColorSpacePoint pointAR;
	ColorSpacePoint pointFR;
	ColorSpacePoint pointHiL;
	ColorSpacePoint pointKL;
	ColorSpacePoint pointAL;
	ColorSpacePoint pointFL;
	ColorSpacePoint pointHtmp;
	ColorSpacePoint pointNtmp;
	ColorSpacePoint pointSStmp;
	ColorSpacePoint pointSMtmp;
	ColorSpacePoint pointSBtmp;
	ColorSpacePoint pointSRtmp;
	ColorSpacePoint pointERtmp;
	ColorSpacePoint pointWRtmp;
	ColorSpacePoint pointHRtmp;
	ColorSpacePoint pointTRtmp;
	ColorSpacePoint pointHTRtmp;
	ColorSpacePoint pointSLtmp;
	ColorSpacePoint pointELtmp;
	ColorSpacePoint pointWLtmp;
	ColorSpacePoint pointHLtmp;
	ColorSpacePoint pointTLtmp;
	ColorSpacePoint pointHTLtmp;
	ColorSpacePoint pointHiRtmp;
	ColorSpacePoint pointKRtmp;
	ColorSpacePoint pointARtmp;
	ColorSpacePoint pointFRtmp;
	ColorSpacePoint pointHiLtmp;
	ColorSpacePoint pointKLtmp;
	ColorSpacePoint pointALtmp;
	ColorSpacePoint pointFLtmp;
	CameraSpacePoint PointH;
	CameraSpacePoint PointN;
	CameraSpacePoint PointSS;
	CameraSpacePoint PointSM;
	CameraSpacePoint PointSB;
	CameraSpacePoint PointSR;
	CameraSpacePoint PointER;
	CameraSpacePoint PointWR;
	CameraSpacePoint PointHR;
	CameraSpacePoint PointTR;
	CameraSpacePoint PointHTR;
	CameraSpacePoint PointSL;
	CameraSpacePoint PointEL;
	CameraSpacePoint PointWL;
	CameraSpacePoint PointHL;
	CameraSpacePoint PointTL;
	CameraSpacePoint PointHTL;
	CameraSpacePoint PointHiR;
	CameraSpacePoint PointKR;
	CameraSpacePoint PointAR;
	CameraSpacePoint PointFR;
	CameraSpacePoint PointHiL;
	CameraSpacePoint PointKL;
	CameraSpacePoint PointAL;
	CameraSpacePoint PointFL;
	CameraSpacePoint PointHtmp;
	CameraSpacePoint PointNtmp;
	CameraSpacePoint PointSStmp;
	CameraSpacePoint PointSMtmp;
	CameraSpacePoint PointSBtmp;
	CameraSpacePoint PointSRtmp;
	CameraSpacePoint PointERtmp;
	CameraSpacePoint PointWRtmp;
	CameraSpacePoint PointHRtmp;
	CameraSpacePoint PointTRtmp;
	CameraSpacePoint PointHTRtmp;
	CameraSpacePoint PointSLtmp;
	CameraSpacePoint PointELtmp;
	CameraSpacePoint PointWLtmp;
	CameraSpacePoint PointHLtmp;
	CameraSpacePoint PointTLtmp;
	CameraSpacePoint PointHTLtmp;
	CameraSpacePoint PointHiRtmp;
	CameraSpacePoint PointKRtmp;
	CameraSpacePoint PointARtmp;
	CameraSpacePoint PointFRtmp;
	CameraSpacePoint PointHiLtmp;
	CameraSpacePoint PointKLtmp;
	CameraSpacePoint PointALtmp;
	CameraSpacePoint PointFLtmp;
	DepthSpacePoint dpointH;
	DepthSpacePoint dpointN;
	DepthSpacePoint dpointSS;
	DepthSpacePoint dpointSM;
	DepthSpacePoint dpointSB;
	DepthSpacePoint dpointSR;
	DepthSpacePoint dpointER;
	DepthSpacePoint dpointWR;
	DepthSpacePoint dpointHR;
	DepthSpacePoint dpointTR;
	DepthSpacePoint dpointHTR;
	DepthSpacePoint dpointSL;
	DepthSpacePoint dpointEL;
	DepthSpacePoint dpointWL;
	DepthSpacePoint dpointHL;
	DepthSpacePoint dpointTL;
	DepthSpacePoint dpointHTL;
	DepthSpacePoint dpointHiR;
	DepthSpacePoint dpointKR;
	DepthSpacePoint dpointAR;
	DepthSpacePoint dpointFR;
	DepthSpacePoint dpointHiL;
	DepthSpacePoint dpointKL;
	DepthSpacePoint dpointAL;
	DepthSpacePoint dpointFL;
	float COG3D[3];
	float COG3Dtmp[3];
	vector<float> prev;
	int fonttype = 3;
	int fontsize = 9;

	int font = cv::FONT_HERSHEY_COMPLEX;

	cv::Mat bodyImage;
	cv::Mat colorImage2;

	int cnt;
	int flag = 0;
	int setnum = 5;






	// mission accomplished

	void run()
	{
		// 関節の座標をDepth座標系で表示する
		cv::Mat bodyImage = cv::Mat::zeros(424, 512, CV_8UC4);
		cv::Mat colorImage2(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
		//VideoWriter writer("bodyImage.avi", CV_FOURCC_DEFAULT, 30, Size(424, 512), true);
		//VideoWriter writer2("ColorImage2.avi", CV_FOURCC_DEFAULT, 30, Size(colorWidth, colorHeight), true);
		/*if (!writer.isOpened()) {
		std::cout << "Could not open the output video." << std::endl;
		return ;
		}*/

		cnt = 0;
		FILE *fp, *fp2, *fp3, *fp4; //fp4にデータ、fp5に矢印を出した時間、方向が記録される
		const char *fname = FILENAME;
		const char *fname3D = FILENAME3D;
		char fname4[10];
		int imcount, key;
		int number = 0;
		int count = 0; //何回音を鳴らしたかを格納する変数
		int testflag = 0; //0のときKinectの画像を表示し(記録しない)、1のとき表示しない(ファイルへの書き込みをする)
		LARGE_INTEGER nFreq, nBefore, nAfter; //時間を格納する変数
		DWORD dwTime; //1回の試行で経過した時間を計測しておく変数
		//fp = fopen(fname, "w");
		//fp2 = fopen("speed.csv", "w");
		//fp3 = fopen(fname3D, "w");
		//fp4 = fopen("test.csv", "w"); //捨てるデータ
		COG3Dtmp[0] = 0;
		COG3Dtmp[1] = 0;
		COG3Dtmp[2] = 0;
		//   fprintf(fp, "pointH.X, pointN.X, pointSS.X, pointSM.X, pointSB.X, pointSR.X, pointER.X, pointWR.X, pointHR.X, pointTR.X, pointHTR.X, pointSL.X, pointEL.X, pointWL.X, pointHL.X, pointTL.X, pointHTL.X, pointHiR.X, pointKR.X, pointAR.X, pointFR.X, pointHiL.X, pointKL.X, pointAL.X, pointFL.X, pointH.Y, pointN.Y, pointSS.Y, pointSM.Y, pointSB.Y, pointSR.Y, pointER.Y, pointWR.Y, pointHR.Y, pointTR.Y, pointHTR.Y, pointSL.Y, pointEL.Y, pointWL.Y, pointHL.Y, pointTL.Y, pointHTL.Y, pointHiR.Y, pointKR.Y, pointAR.Y, pointFR.Y, pointHiL.Y, pointKL.Y, pointAL.Y, pointFL.Y,time\n");
		//if (fp == NULL) {
		//printf("%sファイルが開けません¥n", fname);
		//  return -1;
		//}
		//時間変数の初期化
		memset(&nFreq, 0x00, sizeof nFreq);
		memset(&nBefore, 0x00, sizeof nBefore);
		memset(&nAfter, 0x00, sizeof nAfter);
		dwTime = 0;



		string folder_name;
		printf("Enter the file name。\n");
		cin >> folder_name;



		printf("kinect.csv ready go!\n");

		const char* folder_name_s = folder_name.c_str();
		//const char* folder_name_s = "kinect.csv";
		// excel output header
		fp4 = fopen(folder_name_s, "w");
		fprintf(fp4, "COG.X,COG.Y,COG.Z,PointH.X, PointN.X, PointSS.X, PointSM.X, PointSB.X, PointSR.X, PointER.X, PointWR.X, PointHR.X, PointTR.X, PointHTR.X, PointSL.X, PointEL.X, PointWL.X, PointHL.X, PointTL.X, PointHTL.X, PointHiR.X, PointKR.X, PointAR.X, PointFR.X, PointHiL.X, PointKL.X, PointAL.X, PointFL.X, PointH.Y, PointN.Y, PointSS.Y, PointSM.Y, PointSB.Y, PointSR.Y, PointER.Y, PointWR.Y, PointHR.Y, PointTR.Y, PointHTR.Y, PointSL.Y, PointEL.Y, PointWL.Y, PointHL.Y, PointTL.Y, PointHTL.Y, PointHiR.Y, PointKR.Y, PointAR.Y, PointFR.Y, PointHiL.Y, PointKL.Y, PointAL.Y, PointFL.Y,PointH.Z, PointN.Z, PointSS.Z, PointSM.Z, PointSB.Z, PointSR.Z, PointER.Z, PointWR.Z, PointHR.Z, PointTR.Z, PointHTR.Z, PointSL.Z, PointEL.Z, PointWL.Z, PointHL.Z, PointTL.Z, PointHTL.Z, PointHiR.Z, PointKR.Z, PointAR.Z, PointFR.Z, PointHiL.Z, PointKL.Z, PointAL.Z, PointFL.Z, flag, time, class\n");

		int halfflag = 0;
		int exp_count = 0;

		//時間の計測の開始
		clock_t start = clock();
		QueryPerformanceFrequency(&nFreq);
		QueryPerformanceCounter(&nBefore);

		//cout << "press a to continue! " << endl;

		char trigger;
		/*cin >> trigger;
		if (trigger != 'a')
		{
			return;
		}*/

		while (1) {
			update();
			//draw(fp, fp2, fp3, fp4, start, dwTime, testflag);
			draw(fp4, start, dwTime, testflag);
			QueryPerformanceCounter(&nAfter);
			dwTime = (DWORD)((nAfter.QuadPart - nBefore.QuadPart) * 1000 / nFreq.QuadPart);
			key = cv::waitKey(10);
			if (key == 's' || key == 'q' || dwTime > 10000) break; //sを押すか20秒経過で開始
		}
		while (number < setnum * 12) {
			//****inst random choose 
			std::mt19937 mt{ std::random_device{}() };
			std::uniform_int_distribution<int> dist(0, inst.size() - 1);//random the random num
			int index = dist(mt);// choose random index
			vector<int> valuelist = inst[index];
			inst.erase(inst.begin() + index);// erase the random index  

			//hosuu = valuelist[0];
			//sizi = valuelist[1];
			exp_count = exp_count + 1;
			string ex_count = to_string(exp_count);

			//string s1 = to_string(hosuu);
			/*
			string s2;
			if (sizi == 0) {
				s2 = " ";
			}
			if (sizi == 1) {
				s2 = "STOP";
			}
			if (sizi == 2) {
				s2 = " ";
			}*/

			//string hokousizi = "go on";
			cnt = 0;
			if (key == 'q') break;

			//時間の計測の開始
			start = clock();
			QueryPerformanceFrequency(&nFreq);
			QueryPerformanceCounter(&nBefore);
			//imcount = 0;
			while (count < 15) {

				update();
				QueryPerformanceCounter(&nAfter);
				dwTime = (DWORD)((nAfter.QuadPart - nBefore.QuadPart) * 1000 / nFreq.QuadPart);

				saveflag = 1;//////////////////////
				draw(fp4, start, dwTime, testflag);
				if (dwTime > count * 500) {
					//count = metronome(count);
					count++;
					//MessageBeep(MB_OK);
					//PlaySound(TEXT("C:\\Users\\bluek\\OneDrive\\ドキュメント\\Visual Studio 2013\\Projects\\kinectV2\\beep-07.wav"), NULL, (SND_ASYNC | SND_FILENAME));
					//PlaySound(TEXT("beep-07.wav"), NULL, SND_ASYNC);
					if (count == 8) {//after 4s, flag=1 
						flag = 1;
					}
					if (count == 14) {//after 7s, flag=0
						flag = 0;
					}
				}
				switch (count) {
				case(1):
					cv::putText(colorImage2, "ready", cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					cv::putText(colorImage2, ex_count, cv::Point(300, 700), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;
				case(2):
					cv::putText(colorImage2, "ready", cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					cv::putText(colorImage2, ex_count, cv::Point(300, 700), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;

				case(3):
					cv::putText(colorImage2, "2", cv::Point(750, 550), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					//cv::putText(colorImage2, hokousizi, cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;

				case(4):
					//cv::putText(colorImage2, hokousizi, cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;

				case(5):
					cv::putText(colorImage2, "1", cv::Point(750, 550), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					//cv::putText(colorImage2, hokousizi, cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;

				case(6):
					//cv::putText(colorImage2, hokousizi, cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;
				case(7):
					cv::putText(colorImage2, "start", cv::Point(750, 550), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					//cv::putText(colorImage2, hokousizi, cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;
				case(8):
					cv::putText(colorImage2, "3", cv::Point(750, 550), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;
				case(9):
					//cv::putText(colorImage2, hokousizi, cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					cv::putText(colorImage2, "3", cv::Point(750, 550), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;
				case(10):
					cv::putText(colorImage2, "2", cv::Point(750, 550), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;
				case(11):
					cv::putText(colorImage2, "go", cv::Point(750, 550), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					//cv::putText(colorImage2, hokousizi, cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;
				case(12):
					cv::putText(colorImage2, "go", cv::Point(750, 550), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					//cv::putText(colorImage2, hokousizi, cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;
				case(13):
					cv::putText(colorImage2, "go", cv::Point(750, 550), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					//cv::putText(colorImage2, hokousizi, cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;
				case(14):
					cv::putText(colorImage2, "End", cv::Point(750, 550), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					//cv::putText(colorImage2, hokousizi, cv::Point(300, 250), font, fontsize, cv::Scalar(0, 0, 255), 5, CV_AA);
					break;

				}
				//draw(fp4, start, dwTime, testflag);
				key = cv::waitKey(1);
				if (key == 'q') break;
			}
			if (key == 'q') break;
			count = 0;
			cnt = 0;
			number++;

		}
		fclose(fp4);


		// finish updater

		cout << "finish" << endl;

	}

private:

	// データの更新処理
	void update()
	{
		updateBodyFrame();
	}

	// ボディフレームの更新
	void updateBodyFrame()
	{
		// フレームを取得する
		ComPtr<IBodyFrame> bodyFrame;
		auto ret = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
		if (ret == S_OK) {
			// データを取得する
			ERROR_CHECK(bodyFrame->GetAndRefreshBodyData(6, &bodies[0]));

			// スマートポインタを使ってない場合は、自分でフレームを解放する
			// bodyFrame->Release();
		}
	}
	// カラーフレームの更新
	void updateColorFrame()
	{
		// フレームを取得する
		ComPtr<IColorFrame> colorFrame;
		auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if (ret == S_OK) {
			// BGRAの形式でデータを取得する
			ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(
				colorBuffer.size(), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra));

			// カラーデータを表示する
			cv::Mat colorImage(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
			// cv::imshow("Color Image", colorImage);

			// スマートポインタを使ってない場合は、自分でフレームを解放する
			// colorFrame->Release();
		}
	}

	//void draw(FILE *fp, FILE *fp2, FILE *fp3, FILE *fp4, clock_t start, DWORD dwTime, int testflag)
	void draw(FILE *fp4, clock_t start, DWORD dwTime, int testflag)
	{
		//drawBodyIndexFrame(fp, fp2, fp3, fp4, start, dwTime, testflag);
		drawBodyIndexFrame(fp4, start, dwTime, testflag);
		updateColorFrame();
	}

	//void drawBodyIndexFrame(FILE *fp, FILE *fp2, FILE *fp3, FILE *fp4, clock_t start, DWORD dwTime, int testflag)
	void drawBodyIndexFrame(FILE *fp4, clock_t start, DWORD dwTime, int testflag)
	{

		// 関節の座標をDepth座標系で表示する
		cv::Mat bodyImage = cv::Mat::zeros(424, 512, CV_8UC4);
		cv::Mat colorImage2(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
		cv::Mat maskImage2;


		for (auto body : bodies) {
			if (body == nullptr) {
				continue;
			}

			BOOLEAN isTracked = false;
			ERROR_CHECK(body->get_IsTracked(&isTracked));
			if (!isTracked) {
				continue;
			}

			// 関節の位置を表示する
			Joint joints[JointType::JointType_Count];
			body->GetJoints(JointType::JointType_Count, joints);

			//bodyimageに重心
			ComPtr<ICoordinateMapper> mapper4;
			ERROR_CHECK(kinect->get_CoordinateMapper(&mapper4));
			mapper4->MapCameraPointToDepthSpace(joints[0].Position, &dpointSB);
			mapper4->MapCameraPointToDepthSpace(joints[1].Position, &dpointSM);
			mapper4->MapCameraPointToDepthSpace(joints[2].Position, &dpointN);
			mapper4->MapCameraPointToDepthSpace(joints[3].Position, &dpointH);
			mapper4->MapCameraPointToDepthSpace(joints[4].Position, &dpointSL);
			mapper4->MapCameraPointToDepthSpace(joints[5].Position, &dpointEL);
			mapper4->MapCameraPointToDepthSpace(joints[6].Position, &dpointWL);
			mapper4->MapCameraPointToDepthSpace(joints[7].Position, &dpointHL);
			mapper4->MapCameraPointToDepthSpace(joints[8].Position, &dpointSR);
			mapper4->MapCameraPointToDepthSpace(joints[9].Position, &dpointER);
			mapper4->MapCameraPointToDepthSpace(joints[10].Position, &dpointWR);
			mapper4->MapCameraPointToDepthSpace(joints[11].Position, &dpointHR);
			mapper4->MapCameraPointToDepthSpace(joints[12].Position, &dpointHiL);
			mapper4->MapCameraPointToDepthSpace(joints[13].Position, &dpointKL);
			mapper4->MapCameraPointToDepthSpace(joints[14].Position, &dpointAL);
			mapper4->MapCameraPointToDepthSpace(joints[15].Position, &dpointFL);
			mapper4->MapCameraPointToDepthSpace(joints[16].Position, &dpointHiR);
			mapper4->MapCameraPointToDepthSpace(joints[17].Position, &dpointKR);
			mapper4->MapCameraPointToDepthSpace(joints[18].Position, &dpointAR);
			mapper4->MapCameraPointToDepthSpace(joints[19].Position, &dpointFR);
			mapper4->MapCameraPointToDepthSpace(joints[20].Position, &dpointSS);
			mapper4->MapCameraPointToDepthSpace(joints[21].Position, &dpointHTL);
			mapper4->MapCameraPointToDepthSpace(joints[22].Position, &dpointTL);
			mapper4->MapCameraPointToDepthSpace(joints[23].Position, &dpointHTR);
			mapper4->MapCameraPointToDepthSpace(joints[24].Position, &dpointTR);

			cv::line(bodyImage, cv::Point(dpointH.X, dpointH.Y), cv::Point(dpointN.X, dpointN.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointN.X, dpointN.Y), cv::Point(dpointSS.X, dpointSS.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointSS.X, dpointSS.Y), cv::Point(dpointSM.X, dpointSM.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointSB.X, dpointSB.Y), cv::Point(dpointSM.X, dpointSM.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointSS.X, dpointSS.Y), cv::Point(dpointSR.X, dpointSR.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointER.X, dpointER.Y), cv::Point(dpointSR.X, dpointSR.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointER.X, dpointER.Y), cv::Point(dpointWR.X, dpointWR.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointHR.X, dpointHR.Y), cv::Point(dpointWR.X, dpointWR.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointHR.X, dpointHR.Y), cv::Point(dpointTR.X, dpointTR.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointHR.X, dpointHR.Y), cv::Point(dpointHTR.X, dpointHTR.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointSS.X, dpointSS.Y), cv::Point(dpointSL.X, dpointSL.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointEL.X, dpointEL.Y), cv::Point(dpointSL.X, dpointSL.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointEL.X, dpointEL.Y), cv::Point(dpointWL.X, dpointWL.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointHL.X, dpointHL.Y), cv::Point(dpointWL.X, dpointWL.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointHL.X, dpointHL.Y), cv::Point(dpointTL.X, dpointTL.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointHL.X, dpointHL.Y), cv::Point(dpointHTL.X, dpointHTL.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointHiR.X, dpointHiR.Y), cv::Point(dpointSB.X, dpointSB.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointHiR.X, dpointHiR.Y), cv::Point(dpointKR.X, dpointKR.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointAR.X, dpointAR.Y), cv::Point(dpointKR.X, dpointKR.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointAR.X, dpointAR.Y), cv::Point(dpointFR.X, dpointFR.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointHiL.X, dpointHiL.Y), cv::Point(dpointSB.X, dpointSB.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointHiL.X, dpointHiL.Y), cv::Point(dpointKL.X, dpointKL.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointAL.X, dpointAL.Y), cv::Point(dpointKL.X, dpointKL.Y), cv::Scalar(0, 255, 0), 3);
			cv::line(bodyImage, cv::Point(dpointAL.X, dpointAL.Y), cv::Point(dpointFL.X, dpointFL.Y), cv::Scalar(0, 255, 0), 3);

			for (auto joint : joints)
			{
				// 手の位置が追跡状態
				if (joint.TrackingState == TrackingState::TrackingState_Tracked) {
					drawEllipse(bodyImage, colorImage2, joint, 10, cv::Scalar(255, 0, 0));
					// 左手を追跡していたら、手の状態を表示する
					if (joint.JointType == JointType::JointType_HandLeft) {
						HandState handState;
						TrackingConfidence handConfidence;
						body->get_HandLeftState(&handState);
						body->get_HandLeftConfidence(&handConfidence);
						// a = joint.Position;
						//drawHandState(bodyImage, joint, handConfidence, handState);
					}
					// 右手を追跡していたら、手の状態を表示する
					else if (joint.JointType == JointType::JointType_HandRight) {
						HandState handState;
						TrackingConfidence handConfidence;
						body->get_HandRightState(&handState);
						body->get_HandRightConfidence(&handConfidence);

						//drawHandState(bodyImage, joint, handConfidence, handState);
					}
				}
				// 手の位置が推測状態
				else if (joint.TrackingState == TrackingState::TrackingState_Inferred) {
					drawEllipse(bodyImage, colorImage2, joint, 10, cv::Scalar(255, 255, 0));
				}
			}



			// カメラ座標系をDepth座標系に変換する
			ComPtr<ICoordinateMapper> mapper3;
			ERROR_CHECK(kinect->get_CoordinateMapper(&mapper3));
			ColorSpacePoint point3;
			ColorSpacePoint point4;
			mapper3->MapCameraPointToColorSpace(joints[10].Position, &point3);
			mapper3->MapCameraPointToColorSpace(joints[20].Position, &point3);
			mapper3->MapCameraPointToColorSpace(joints[0].Position, &pointSB);
			mapper3->MapCameraPointToColorSpace(joints[1].Position, &pointSM);
			mapper3->MapCameraPointToColorSpace(joints[2].Position, &pointN);
			mapper3->MapCameraPointToColorSpace(joints[3].Position, &pointH);
			mapper3->MapCameraPointToColorSpace(joints[4].Position, &pointSL);
			mapper3->MapCameraPointToColorSpace(joints[5].Position, &pointEL);
			mapper3->MapCameraPointToColorSpace(joints[6].Position, &pointWL);
			mapper3->MapCameraPointToColorSpace(joints[7].Position, &pointHL);
			mapper3->MapCameraPointToColorSpace(joints[8].Position, &pointSR);
			mapper3->MapCameraPointToColorSpace(joints[9].Position, &pointER);
			mapper3->MapCameraPointToColorSpace(joints[10].Position, &pointWR);
			mapper3->MapCameraPointToColorSpace(joints[11].Position, &pointHR);
			mapper3->MapCameraPointToColorSpace(joints[12].Position, &pointHiL);
			mapper3->MapCameraPointToColorSpace(joints[13].Position, &pointKL);
			mapper3->MapCameraPointToColorSpace(joints[14].Position, &pointAL);
			mapper3->MapCameraPointToColorSpace(joints[15].Position, &pointFL);
			mapper3->MapCameraPointToColorSpace(joints[16].Position, &pointHiR);
			mapper3->MapCameraPointToColorSpace(joints[17].Position, &pointKR);
			mapper3->MapCameraPointToColorSpace(joints[18].Position, &pointAR);
			mapper3->MapCameraPointToColorSpace(joints[19].Position, &pointFR);
			mapper3->MapCameraPointToColorSpace(joints[20].Position, &pointSS);
			mapper3->MapCameraPointToColorSpace(joints[21].Position, &pointHTL);
			mapper3->MapCameraPointToColorSpace(joints[22].Position, &pointTL);
			mapper3->MapCameraPointToColorSpace(joints[23].Position, &pointHTR);
			mapper3->MapCameraPointToColorSpace(joints[24].Position, &pointTR);
			/*cv::line(colorImage2, cv::Point(pointH.X, pointH.Y), cv::Point(pointN.X, pointN.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointN.X, pointN.Y), cv::Point(pointSS.X, pointSS.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointSS.X, pointSS.Y), cv::Point(pointSM.X, pointSM.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointSB.X, pointSB.Y), cv::Point(pointSM.X, pointSM.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointSS.X, pointSS.Y), cv::Point(pointSR.X, pointSR.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointER.X, pointER.Y), cv::Point(pointSR.X, pointSR.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointER.X, pointER.Y), cv::Point(pointWR.X, pointWR.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointHR.X, pointHR.Y), cv::Point(pointWR.X, pointWR.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointHR.X, pointHR.Y), cv::Point(pointTR.X, pointTR.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointHR.X, pointHR.Y), cv::Point(pointHTR.X, pointHTR.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointSS.X, pointSS.Y), cv::Point(pointSL.X, pointSL.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointEL.X, pointEL.Y), cv::Point(pointSL.X, pointSL.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointEL.X, pointEL.Y), cv::Point(pointWL.X, pointWL.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointHL.X, pointHL.Y), cv::Point(pointWL.X, pointWL.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointHL.X, pointHL.Y), cv::Point(pointTL.X, pointTL.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointHL.X, pointHL.Y), cv::Point(pointHTL.X, pointHTL.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointHiR.X, pointHiR.Y), cv::Point(pointSB.X, pointSB.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointHiR.X, pointHiR.Y), cv::Point(pointKR.X, pointKR.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointAR.X, pointAR.Y), cv::Point(pointKR.X, pointKR.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointAR.X, pointAR.Y), cv::Point(pointFR.X, pointFR.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointHiL.X, pointHiL.Y), cv::Point(pointSB.X, pointSB.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointHiL.X, pointHiL.Y), cv::Point(pointKL.X, pointKL.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointAL.X, pointAL.Y), cv::Point(pointKL.X, pointKL.Y), cv::Scalar(0, 0, 255), 1.8);
			cv::line(colorImage2, cv::Point(pointAL.X, pointAL.Y), cv::Point(pointFL.X, pointFL.Y), cv::Scalar(0, 0, 255), 1.8);*/
			float COG[2];
			float dCOG[2];
			//float COG3D[3];
			float segments[8] = { 10.1, 48.6, 3.5, 1.7, 1.8, 8.3, 3.5, 1.9 };
			COG[0] = pointH.X * segments[0]
				+ (pointSS.X + pointSB.X + pointSM.X) *segments[1] / 3
				+ (pointSR.X + pointER.X) *segments[2] / 2
				+ (pointER.X + pointWR.X) *segments[3] / 2
				+ (pointSL.X + pointEL.X) *segments[2] / 2
				+ (pointEL.X + pointWL.X) *segments[3] / 2
				+ (pointHR.X) *segments[4]
				+ (pointHL.X) *segments[4]
				+ (pointHiR.X + pointKR.X) *segments[5] / 2
				+ (pointHiL.X + pointKL.X) *segments[5] / 2
				+ (pointKR.X + pointAR.X) *segments[6] / 2
				+ (pointKL.X + pointAL.X) *segments[6] / 2
				+ (pointFR.X) *segments[7]
				+ (pointFL.X) *segments[7];

			COG[1] = pointH.Y * segments[0]
				+ (pointSS.Y + pointSB.Y + pointSM.Y) *segments[1] / 3
				+ (pointSR.Y + pointER.Y) *segments[2] / 2
				+ (pointER.Y + pointWR.Y) *segments[3] / 2
				+ (pointSL.Y + pointEL.Y) *segments[2] / 2
				+ (pointEL.Y + pointWL.Y) *segments[3] / 2
				+ (pointHR.Y) *segments[4]
				+ (pointHL.Y) *segments[4]
				+ (pointHiR.Y + pointKR.Y) *segments[5] / 2
				+ (pointHiL.Y + pointKL.Y) *segments[5] / 2
				+ (pointKR.Y + pointAR.Y) *segments[6] / 2
				+ (pointKL.Y + pointAL.Y) *segments[6] / 2
				+ (pointFR.Y) *segments[7]
				+ (pointFL.Y) *segments[7];

			dCOG[0] = dpointH.X * segments[0]
				+ (dpointSS.X + dpointSB.X + dpointSM.X) *segments[1] / 3
				+ (dpointSR.X + dpointER.X) *segments[2] / 2
				+ (dpointER.X + dpointWR.X) *segments[3] / 2
				+ (dpointSL.X + dpointEL.X) *segments[2] / 2
				+ (dpointEL.X + dpointWL.X) *segments[3] / 2
				+ (dpointHR.X) *segments[4]
				+ (dpointHL.X) *segments[4]
				+ (dpointHiR.X + dpointKR.X) *segments[5] / 2
				+ (dpointHiL.X + dpointKL.X) *segments[5] / 2
				+ (dpointKR.X + dpointAR.X) *segments[6] / 2
				+ (dpointKL.X + dpointAL.X) *segments[6] / 2
				+ (dpointFR.X) *segments[7]
				+ (dpointFL.X) *segments[7];

			dCOG[1] = dpointH.Y * segments[0]
				+ (dpointSS.Y + dpointSB.Y + dpointSM.Y) *segments[1] / 3
				+ (dpointSR.Y + dpointER.Y) *segments[2] / 2
				+ (dpointER.Y + dpointWR.Y) *segments[3] / 2
				+ (dpointSL.Y + dpointEL.Y) *segments[2] / 2
				+ (dpointEL.Y + dpointWL.Y) *segments[3] / 2
				+ (dpointHR.Y) *segments[4]
				+ (dpointHL.Y) *segments[4]
				+ (dpointHiR.Y + dpointKR.Y) *segments[5] / 2
				+ (dpointHiL.Y + dpointKL.Y) *segments[5] / 2
				+ (dpointKR.Y + dpointAR.Y) *segments[6] / 2
				+ (dpointKL.Y + dpointAL.Y) *segments[6] / 2
				+ (dpointFR.Y) *segments[7]
				+ (dpointFL.Y) *segments[7];

			COG3D[0] = PointH.X * segments[0]
				+ (PointSS.X + PointSB.X + PointSM.X) *segments[1] / 3
				+ (PointSR.X + PointER.X) *segments[2] / 2
				+ (PointER.X + PointWR.X) *segments[3] / 2
				+ (PointSL.X + PointEL.X) *segments[2] / 2
				+ (PointEL.X + PointWL.X) *segments[3] / 2
				+ (PointHR.X) *segments[4]
				+ (PointHL.X) *segments[4]
				+ (PointHiR.X + PointKR.X) *segments[5] / 2
				+ (PointHiL.X + PointKL.X) *segments[5] / 2
				+ (PointKR.X + PointAR.X) *segments[6] / 2
				+ (PointKL.X + PointAL.X) *segments[6] / 2
				+ (PointFR.X) *segments[7]
				+ (PointFL.X) *segments[7];




			COG3D[1] = PointH.Y * segments[0]
				+ (PointSS.Y + PointSB.Y + PointSM.Y) *segments[1] / 3
				+ (PointSR.Y + PointER.Y) *segments[2] / 2
				+ (PointER.Y + PointWR.Y) *segments[3] / 2
				+ (PointSL.Y + PointEL.Y) *segments[2] / 2
				+ (PointEL.Y + PointWL.Y) *segments[3] / 2
				+ (PointHR.Y) *segments[4]
				+ (PointHL.Y) *segments[4]
				+ (PointHiR.Y + PointKR.Y) *segments[5] / 2
				+ (PointHiL.Y + PointKL.Y) *segments[5] / 2
				+ (PointKR.Y + PointAR.Y) *segments[6] / 2
				+ (PointKL.Y + PointAL.Y) *segments[6] / 2
				+ (PointFR.Y) *segments[7]
				+ (PointFL.Y) *segments[7];

			COG3D[2] = PointH.Z * segments[0]
				+ (PointSS.Z + PointSB.Z + PointSM.Z) *segments[1] / 3
				+ (PointSR.Z + PointER.Z) *segments[2] / 2
				+ (PointER.Z + PointWR.Z) *segments[3] / 2
				+ (PointSL.Z + PointEL.Z) *segments[2] / 2
				+ (PointEL.Z + PointWL.Z) *segments[3] / 2
				+ (PointHR.Z) *segments[4]
				+ (PointHL.Z) *segments[4]
				+ (PointHiR.Z + PointKR.Z) *segments[5] / 2
				+ (PointHiL.Z + PointKL.Z) *segments[5] / 2
				+ (PointKR.Z + PointAR.Z) *segments[6] / 2
				+ (PointKL.Z + PointAL.Z) *segments[6] / 2
				+ (PointFR.Z) *segments[7]
				+ (PointFL.Z) *segments[7];
			//重心プロット
			cv::circle(colorImage2, cv::Point(COG[0] / 100, COG[1] / 100), 10, cv::Scalar(0, 0, 255), -1);
			cv::circle(bodyImage, cv::Point(dCOG[0] / 100, dCOG[1] / 100), 10, cv::Scalar(0, 0, 255), -1);

			break;
		}

		if (testflag == 0) {
			cv::imshow("Body Image", bodyImage);
			cv::imshow("Color Image2", colorImage2);
			/*if (dwTime > cnt * 500) {
			if (cnt == 6) {
			imwrite("body0.jpg", bodyImage);
			imwrite("color0.jpg", colorImage2);
			}
			if (cnt == 7) {
			imwrite("body0_5.jpg", bodyImage);
			imwrite("color0_5.jpg", colorImage2);
			}
			if (cnt == 8) {
			imwrite("body1.jpg", bodyImage);
			imwrite("color1.jpg", colorImage2);
			}
			cnt += 1;

			}*/
		}
		if (saveflag == 1) {
			//printf("insave");
			//clock_t end = clock();
			//double now = (double)(end - start) / 1000;
			SYSTEMTIME sys;
			GetLocalTime(&sys);
			long t = getAllTime(sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds);

			int c = 2;


			vector<float> nowin = { COG3D[0], COG3D[1], COG3D[2], PointH.X, PointN.X, PointSS.X, PointSM.X, PointSB.X, PointSR.X, PointER.X, PointWR.X, PointHR.X, PointTR.X, PointHTR.X, PointSL.X, PointEL.X, PointWL.X, PointHL.X, PointTL.X, PointHTL.X, PointHiR.X, PointKR.X, PointAR.X, PointFR.X, PointHiL.X, PointKL.X, PointAL.X, PointFL.X, PointH.Y, PointN.Y, PointSS.Y, PointSM.Y, PointSB.Y, PointSR.Y, PointER.Y, PointWR.Y, PointHR.Y, PointTR.Y, PointHTR.Y, PointSL.Y, PointEL.Y, PointWL.Y, PointHL.Y, PointTL.Y, PointHTL.Y, PointHiR.Y, PointKR.Y, PointAR.Y, PointFR.Y, PointHiL.Y, PointKL.Y, PointAL.Y, PointFL.Y, PointH.Z, PointN.Z, PointSS.Z, PointSM.Z, PointSB.Z, PointSR.Z, PointER.Z, PointWR.Z, PointHR.Z, PointTR.Z, PointHTR.Z, PointSL.Z, PointEL.Z, PointWL.Z, PointHL.Z, PointTL.Z, PointHTL.Z, PointHiR.Z, PointKR.Z, PointAR.Z, PointFR.Z, PointHiL.Z, PointKL.Z, PointAL.Z, PointFL.Z };
			if (prev != nowin) {
				fprintf(fp4, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%ld,%d\n", COG3D[0], COG3D[1], COG3D[2], PointH.X, PointN.X, PointSS.X, PointSM.X, PointSB.X, PointSR.X, PointER.X, PointWR.X, PointHR.X, PointTR.X, PointHTR.X, PointSL.X, PointEL.X, PointWL.X, PointHL.X, PointTL.X, PointHTL.X, PointHiR.X, PointKR.X, PointAR.X, PointFR.X, PointHiL.X, PointKL.X, PointAL.X, PointFL.X, PointH.Y, PointN.Y, PointSS.Y, PointSM.Y, PointSB.Y, PointSR.Y, PointER.Y, PointWR.Y, PointHR.Y, PointTR.Y, PointHTR.Y, PointSL.Y, PointEL.Y, PointWL.Y, PointHL.Y, PointTL.Y, PointHTL.Y, PointHiR.Y, PointKR.Y, PointAR.Y, PointFR.Y, PointHiL.Y, PointKL.Y, PointAL.Y, PointFL.Y, PointH.Z, PointN.Z, PointSS.Z, PointSM.Z, PointSB.Z, PointSR.Z, PointER.Z, PointWR.Z, PointHR.Z, PointTR.Z, PointHTR.Z, PointSL.Z, PointEL.Z, PointWL.Z, PointHL.Z, PointTL.Z, PointHTL.Z, PointHiR.Z, PointKR.Z, PointAR.Z, PointFR.Z, PointHiL.Z, PointKL.Z, PointAL.Z, PointFL.Z, flag, t, c);
				//fprintf(fp4, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,", (PointH.X - PointHtmp.X) / (now), (PointN.X - PointNtmp.X) / (now), (PointSS.X - PointSStmp.X) / (now), (PointSM.X - PointSMtmp.X) / (now), (PointSB.X - PointSBtmp.X) / (now), (PointSR.X - PointSRtmp.X) / (now), (PointER.X - PointERtmp.X) / (now), (PointWR.X - PointWRtmp.X) / (now), (PointHR.X - PointHRtmp.X) / (now), (PointTR.X - PointTRtmp.X) / (now), (PointHTR.X - PointHTRtmp.X) / (now), (PointSL.X - PointSLtmp.X) / (now), (PointEL.X - PointELtmp.X) / (now), (PointWL.X - PointWLtmp.X) / (now), (PointHL.X - PointHLtmp.X) / (now), (PointTL.X - PointTLtmp.X) / (now), (PointHTL.X - PointHTLtmp.X) / (now), (PointHiR.X - PointHiRtmp.X) / (now), (PointKR.X - PointKRtmp.X) / (now), (PointAR.X - PointARtmp.X) / (now), (PointFR.X - PointFRtmp.X) / (now), (PointHiL.X - PointHiLtmp.X) / (now), (PointKL.X - PointKLtmp.X) / (now), (PointAL.X - PointALtmp.X) / (now), (PointFL.X - PointFLtmp.X) / (now), (PointH.Y - PointHtmp.Y) / (now), (PointN.Y - PointNtmp.Y) / (now), (PointSS.Y - PointSStmp.Y) / (now), (PointSM.Y - PointSMtmp.Y) / (now), (PointSB.Y - PointSBtmp.Y) / (now), (PointSR.Y - PointSRtmp.Y) / (now), (PointER.Y - PointERtmp.Y) / (now), (PointWR.Y - PointWRtmp.Y) / (now), (PointHR.Y - PointHRtmp.Y) / (now), (PointTR.Y - PointTRtmp.Y) / (now), (PointHTR.Y - PointHTRtmp.Y) / (now), (PointSL.Y - PointSLtmp.Y) / (now), (PointEL.Y - PointELtmp.Y) / (now), (PointWL.Y - PointWLtmp.Y) / (now), (PointHL.Y - PointHLtmp.Y) / (now), (PointTL.Y - PointTLtmp.Y) / (now), (PointHTL.Y - PointHTLtmp.Y) / (now), (PointHiR.Y - PointHiRtmp.Y) / (now), (PointKR.Y - PointKRtmp.Y) / (now), (PointAR.Y - PointARtmp.Y) / (now), (PointFR.Y - PointFRtmp.Y) / (now), (PointHiL.Y - PointHiLtmp.Y) / (now), (PointKL.Y - PointKLtmp.Y) / (now), (PointAL.Y - PointALtmp.Y) / (now), (PointFL.Y - PointFLtmp.Y) / (now), (PointH.Z - PointHtmp.Z) / (now), (PointN.Z - PointNtmp.Z) / (now), (PointSS.Z - PointSStmp.Z) / (now), (PointSM.Z - PointSMtmp.Z) / (now), (PointSB.Z - PointSBtmp.Z) / (now), (PointSR.Z - PointSRtmp.Z) / (now), (PointER.Z - PointERtmp.Z) / (now), (PointWR.Z - PointWRtmp.Z) / (now), (PointHR.Z - PointHRtmp.Z) / (now), (PointTR.Z - PointTRtmp.Z) / (now), (PointHTR.Z - PointHTRtmp.Z) / (now), (PointSL.Z - PointSLtmp.Z) / (now), (PointEL.Z - PointELtmp.Z) / (now), (PointWL.Z - PointWLtmp.Z) / (now), (PointHL.Z - PointHLtmp.Z) / (now), (PointTL.Z - PointTLtmp.Z) / (now), (PointHTL.Z - PointHTLtmp.Z) / (now), (PointHiR.Z - PointHiRtmp.Z) / (now), (PointKR.Z - PointKRtmp.Z) / (now), (PointAR.Z - PointARtmp.Z) / (now), (PointFR.Z - PointFRtmp.Z) / (now), (PointHiL.Z - PointHiLtmp.Z) / (now), (PointKL.Z - PointKLtmp.Z) / (now), (PointAL.Z - PointALtmp.Z) / (now), (PointFL.Z - PointFLtmp.Z) / (now));
			}
			prev = nowin;
		}
		printf("X=%f,Y=%f,Z=%f\n", PointFL.X, PointFL.Y, PointFL.Z);

	}

	void drawEllipse(cv::Mat& bodyImage, cv::Mat& colorImage2, const Joint& joint, int r, const cv::Scalar& color)
	{
		// カメラ座標系をDepth座標系に変換する
		ComPtr<ICoordinateMapper> mapper;
		ERROR_CHECK(kinect->get_CoordinateMapper(&mapper));
		DepthSpacePoint point;
		mapper->MapCameraPointToDepthSpace(joint.Position, &point);

		// カメラ座標系をDepth座標系に変換する
		ComPtr<ICoordinateMapper> mapper2;
		ERROR_CHECK(kinect->get_CoordinateMapper(&mapper2));
		ColorSpacePoint point2;
		mapper2->MapCameraPointToColorSpace(joint.Position, &point2);

		cv::circle(bodyImage, cv::Point(point.X, point.Y), 6, color, -1);
		cv::circle(colorImage2, cv::Point(point2.X, point2.Y), r, color, -1);

		//joudgeState(joint, point2);//colorspace  
		joudgeState2(joint, joint.Position);//colorspace
	}

	void joudgeState(Joint joint, ColorSpacePoint &point2) {
		if (joint.JointType == JointType::JointType_Head) { pointH = point2; }
		else if (joint.JointType == JointType::JointType_Neck) { pointN = point2; }
		else if (joint.JointType == JointType::JointType_SpineShoulder) { pointSS = point2; }
		else if (joint.JointType == JointType::JointType_SpineMid) { pointSM = point2; }
		else if (joint.JointType == JointType::JointType_SpineBase) { pointSB = point2; }
		else if (joint.JointType == JointType::JointType_ShoulderRight) { pointSR = point2; }
		else if (joint.JointType == JointType::JointType_ElbowRight) { pointER = point2; }
		else if (joint.JointType == JointType::JointType_WristRight) { pointWR = point2; }
		else if (joint.JointType == JointType::JointType_HandRight) { pointHR = point2; }
		else if (joint.JointType == JointType::JointType_ThumbRight) { pointTR = point2; }
		else if (joint.JointType == JointType::JointType_HandTipRight) { pointHTR = point2; }
		else if (joint.JointType == JointType::JointType_ShoulderLeft) { pointSL = point2; }
		else if (joint.JointType == JointType::JointType_ElbowLeft) { pointEL = point2; }
		else if (joint.JointType == JointType::JointType_WristLeft) { pointWL = point2; }
		else if (joint.JointType == JointType::JointType_HandLeft) { pointHL = point2; }
		else if (joint.JointType == JointType::JointType_ThumbLeft) { pointTL = point2; }
		else if (joint.JointType == JointType::JointType_HandTipLeft) { pointHTL = point2; }
		else if (joint.JointType == JointType::JointType_HipRight) { pointHiR = point2; }
		else if (joint.JointType == JointType::JointType_KneeRight) { pointKR = point2; }
		else if (joint.JointType == JointType::JointType_AnkleRight) { pointAR = point2; }
		else if (joint.JointType == JointType::JointType_FootRight) { pointFR = point2; }
		else if (joint.JointType == JointType::JointType_HipLeft) { pointHiL = point2; }
		else if (joint.JointType == JointType::JointType_KneeLeft) { pointKL = point2; }
		else if (joint.JointType == JointType::JointType_AnkleLeft) { pointAL = point2; }
		else if (joint.JointType == JointType::JointType_FootLeft) { pointFL = point2; }
	}
	void joudgeState2(Joint joint, CameraSpacePoint Point2) {
		if (joint.JointType == JointType::JointType_Head) { PointH = Point2; }
		else if (joint.JointType == JointType::JointType_Neck) { PointN = Point2; }
		else if (joint.JointType == JointType::JointType_SpineShoulder) { PointSS = Point2; }
		else if (joint.JointType == JointType::JointType_SpineMid) { PointSM = Point2; }
		else if (joint.JointType == JointType::JointType_SpineBase) { PointSB = Point2; }
		else if (joint.JointType == JointType::JointType_ShoulderRight) { PointSR = Point2; }
		else if (joint.JointType == JointType::JointType_ElbowRight) { PointER = Point2; }
		else if (joint.JointType == JointType::JointType_WristRight) { PointWR = Point2; }
		else if (joint.JointType == JointType::JointType_HandRight) { PointHR = Point2; }
		else if (joint.JointType == JointType::JointType_ThumbRight) { PointTR = Point2; }
		else if (joint.JointType == JointType::JointType_HandTipRight) { PointHTR = Point2; }
		else if (joint.JointType == JointType::JointType_ShoulderLeft) { PointSL = Point2; }
		else if (joint.JointType == JointType::JointType_ElbowLeft) { PointEL = Point2; }
		else if (joint.JointType == JointType::JointType_WristLeft) { PointWL = Point2; }
		else if (joint.JointType == JointType::JointType_HandLeft) { PointHL = Point2; }
		else if (joint.JointType == JointType::JointType_ThumbLeft) { PointTL = Point2; }
		else if (joint.JointType == JointType::JointType_HandTipLeft) { PointHTL = Point2; }
		else if (joint.JointType == JointType::JointType_HipRight) { PointHiR = Point2; }
		else if (joint.JointType == JointType::JointType_KneeRight) { PointKR = Point2; }
		else if (joint.JointType == JointType::JointType_AnkleRight) { PointAR = Point2; }
		else if (joint.JointType == JointType::JointType_FootRight) { PointFR = Point2; }
		else if (joint.JointType == JointType::JointType_HipLeft) { PointHiL = Point2; }
		else if (joint.JointType == JointType::JointType_KneeLeft) { PointKL = Point2; }
		else if (joint.JointType == JointType::JointType_AnkleLeft) { PointAL = Point2; }
		else if (joint.JointType == JointType::JointType_FootLeft) { PointFL = Point2; }

	}

	void drawHandState(cv::Mat& bodyImage, Joint joint, TrackingConfidence handConfidence, HandState handState)
	{
		const int R = 40;

		if (handConfidence != TrackingConfidence::TrackingConfidence_High) {
			return;
		}

		// カメラ座標系をDepth座標系に変換する
		ComPtr<ICoordinateMapper> mapper;
		ERROR_CHECK(kinect->get_CoordinateMapper(&mapper));

		DepthSpacePoint point;
		mapper->MapCameraPointToDepthSpace(joint.Position, &point);

		// 手が開いている(パー)
		if (handState == HandState::HandState_Open) {
			cv::circle(bodyImage, cv::Point(point.X, point.Y), R, cv::Scalar(0, 255, 255), R / 4);
		}
		// チョキのような感じ
		else if (handState == HandState::HandState_Lasso) {
			cv::circle(bodyImage, cv::Point(point.X, point.Y), R, cv::Scalar(255, 0, 255), R / 4);
		}
		// 手が閉じている(グー)
		else if (handState == HandState::HandState_Closed) {
			cv::circle(bodyImage, cv::Point(point.X, point.Y), R, cv::Scalar(255, 255, 0), R / 4);
		}
	}

	////メトロノーム
	//int metronome(int count)
	//{
	//	count++;
	//	Beep(880, 500);
	//	return count;
	//}

};

int main(void)
{

	try {
		KinectApp app;
		app.initialize();
		app.run();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}