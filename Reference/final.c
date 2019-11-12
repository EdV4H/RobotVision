//region include
#include <math.h>
#include <ctype.h>
#include <curses.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include "get_contour.h"
#include "serial2016.h"
//endregion

//region define
#define DEBUG 1
#define PORT "/dev/ttyACM0" //適宜変更のこと

#define CAMERA_CENTER_H 102 //カメラサーボの垂直方向中央値（キャリブレーションに利用）
#define CAMERA_CENTER_V 86  //カメラサーボの垂直方向中央値（キャリブレーションに利用）
#define MOTOR_DEFAULT_L 122 //左モータのデフォルト値（キャリブレーションに利用）
#define MOTOR_DEFAULT_R 134 //右モータのデフォルト値（キャリブレーションに利用）
#define CAMERA_INIT_V 75    //カメラサーボの垂直方向初期値
#define CAMERA_INIT_H 90    //カメラサーボの水平方向初期値

#define STRAIGHT_SPEED_CM 1.47 // (140, 140)での値（cm）
//#define STRAIGHT_SPEED_CM 2.25 // (140, 140)での値（cm）
#define STRAIGHT_SPEED_PX 3.5 // (140, 140)での値（cm）
//#define STRAIGHT_SPEED_PX 5.25 // (140, 140)での値（cm）

//#define SLOW_SPIN_SPEED_DEG 3.0
#define SLOW_SPIN_SPEED_DEG 3.05
//#define SLOW_SPIN_SPEED_DEG 4.5
#define FAST_SPIN_SPEED_DEG 10.0

#define STRAIGHT_MOTOR 140
//#define STRAIGHT_MOTOR 146
#define BACK_MOTOR 116
#define LOWSPIN_MOTOR_HIGH 134
//#define LOWSPIN_MOTOR_HIGH 158
#define LOWSPIN_MOTOR_LOW 122
//#define LOWSPIN_MOTOR_LOW 108
#define HIGHSPIN_MOTOR_HIGH 140
#define HIGHSPIN_MOTOR_LOW 116

#define PX_PER_CM (STRAIGHT_SPEED_PX/STRAIGHT_SPEED_CM)
#define CM_PER_PX (1.0 / PX_PER_CM)

#define GO_STRAIGHT do {motor(STRAIGHT_MOTOR, STRAIGHT_MOTOR);} while(0)
#define GO_BACK do {motor(BACK_MOTOR, BACK_MOTOR);} while(0)
#define SLOW_TURN_RIGHT do {motor(LOWSPIN_MOTOR_HIGH, LOWSPIN_MOTOR_LOW);} while(0)
#define SLOW_TURN_LEFT do {motor(LOWSPIN_MOTOR_LOW, LOWSPIN_MOTOR_HIGH);} while(0)
#define SLOW_TURN(turnRight) do {if(turnRight) SLOW_TURN_RIGHT; else SLOW_TURN_LEFT;} while(0)
#define FAST_TURN_RIGHT do {motor(HIGHSPIN_MOTOR_HIGH, HIGHSPIN_MOTOR_LOW);} while(0)
#define FAST_TURN_LEFT do {motor(HIGHSPIN_MOTOR_LOW, HIGHSPIN_MOTOR_HIGH);} while(0)
#define FAST_TURN(turnRight) do {if(turnRight) FAST_TURN_RIGHT; else FAST_TURN_LEFT;} while(0)

#define MID_X 119
#define MAX_Y 255

#define INVISIBLE_DISTANCE_CM 40 // cm
#define ROBOT_X MID_X
#define ROBOT_Y (MAX_Y + INVISIBLE_DISTANCE_CM * PX_PER_CM)

#define TILE_AREA 1000 // 色タイルの面積の閾値

#define RED_MIN_H 110
#define RED_MAX_H 140
#define RED_MIN_S 100
#define RED_MAX_S 255
#define RED_MIN_V 60
#define RED_MAX_V 255

#define GREEN_MIN_H 40
#define GREEN_MAX_H 70
#define GREEN_MIN_S 50
#define GREEN_MAX_S 255
#define GREEN_MIN_V 60
#define GREEN_MAX_V 255

#define BLUE_MIN_H 10
#define BLUE_MAX_H 30
#define BLUE_MIN_S 50
#define BLUE_MAX_S 255
#define BLUE_MIN_V 60
#define BLUE_MAX_V 255

#define PI 3.141592653589793238462643383279

#define SWAP(type, x, y) do {type _tmp = x; x = y; y = _tmp;} while(0)
#define DEG_TO_RAD(deg) ((deg) * PI / 180.0)
#define RAD_TO_DEG(rad) ((rad) * 180.0 / PI)

#define HOUGH_EPS 2
#define HOUGH_STRAIGHT_LIMIT 4
//endregion

//region enum

typedef enum
{
	FINDING,
	FIRST_ROTATING,
	MOVING_TO_FRONT,
	SECOND_ROTATING,
	STRAIGHT_VISIBLE,
	HOUGH,
	STRAIGHT_INVISIBLE,
	ARRIVE,
} State;
//endregion

//region global

CvCapture *capture = NULL;
IplImage *frame;      // キャプチャ画像 (RGB)
IplImage *frameHSV;   // キャプチャ画像 (HSV)
IplImage *framePT;    // 透視変換画像 (RGB)
IplImage *framePTHSV; // 透視変換画像 (HSV)

IplImage *mask; // 指定値によるmask (１チャネル)
IplImage *contour; // GetLargestContour() の結果
IplImage **frames[] = {&frame, &frameHSV};
IplImage **framesPT[] = {&framePT, &framePTHSV};
contourInfo topContoursInfo[CONTOURS];
int key;

//hough
IplImage *frameGray = NULL;
CvMemStorage *storage;
CvSeq *circles = NULL;
float *p;
int hough_count = 0,hough_turn = 0;
double avex = 0,avey = 0,linenum = 0;
int ctotal = 0;
//endregion

void on_mouse(int event, int x, int y, int flags, void *frames)
{
	CvScalar BGR, HSV;
	if (event == CV_EVENT_MOUSEMOVE)
	{
		IplImage *rgb_image = *(((IplImage ***)frames)[0]);
		IplImage *hsv_image = *(((IplImage ***)frames)[1]);
		if (y < rgb_image->height && x < rgb_image->width && y < hsv_image->height && x < hsv_image->width)
		{
			BGR = cvGet2D(rgb_image, y, x);
			HSV = cvGet2D(hsv_image, y, x);
			printf("(%3d,%3d): RGB=(%3.0f,%3.0f,%3.0f) HSV=(%3.0f,%3.0f,%3.0f)\n", x, y, BGR.val[2], BGR.val[1],
			       BGR.val[0], HSV.val[0], HSV.val[1], HSV.val[2]);
		}
	}
}

int main(int argc, char **argv)
{
	init();

	//region prepare

	// 実習項目5.0で計測したモーターの中央値をmotor_onに、サーボの中央値をcamera_onにそれぞれセットする
	motor_on(MOTOR_DEFAULT_L, MOTOR_DEFAULT_R); // モーター静止パルス幅のキャリブレーション
	camera_on(CAMERA_CENTER_V, CAMERA_CENTER_H); // カメラアングルキャリブレーション

	camera_horizontal(CAMERA_INIT_H); // 水平方向のカメラ角度を初期値に
	camera_vertical(CAMERA_INIT_V); // 垂直方向のカメラ角度を初期値に

	// 赤系のHSV色．各自チューニングすること
	uchar minH = RED_MIN_H, maxH = RED_MAX_H;
	uchar minS = RED_MIN_S, maxS = RED_MAX_S;
	uchar minV = RED_MIN_V, maxV = RED_MAX_V;
	CvMat *map_matrix;
	CvPoint2D32f src_pnt[4], dst_pnt[4];

	src_pnt[0] = cvPoint2D32f(181.0, 199.0);
	src_pnt[1] = cvPoint2D32f(110.5, 199.0);
	src_pnt[2] = cvPoint2D32f(104.7, 240.0);
	src_pnt[3] = cvPoint2D32f(184.2, 240.0);
	dst_pnt[0] = cvPoint2D32f(132.5, 240.0);
	dst_pnt[1] = cvPoint2D32f(107.5, 240.0);
	dst_pnt[2] = cvPoint2D32f(107.5, 260.0);
	dst_pnt[3] = cvPoint2D32f(132.5, 260.0);
	map_matrix = cvCreateMat(3, 3, CV_32FC1);
	cvGetPerspectiveTransform(src_pnt, dst_pnt, map_matrix);

	if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
		capture = cvCaptureFromCAM(argc == 2 ? argv[1][0] - '0' : -1);
	if (capture == NULL)
	{
		printf("not find camera\n");
		return -1;
	}

	// 解析速度向上のために画像サイズを下げる
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 320);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 240);

	frame = cvQueryFrame(capture);
	frameHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	framePT = cvCreateImage(cvSize(240, 270), IPL_DEPTH_8U, 3);
	framePTHSV = cvCreateImage(cvGetSize(framePT), IPL_DEPTH_8U, 3);
	mask = cvCreateImage(cvGetSize(framePT), IPL_DEPTH_8U, 1);
	contour = cvCreateImage(cvGetSize(framePT), IPL_DEPTH_8U, 3);

	cvNamedWindow("src", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("dst", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("contour", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("src", 60, 480);
	cvMoveWindow("dst", 380, 480);
	cvMoveWindow("contour", 700, 480);
	cvSetMouseCallback("src", on_mouse, (void *)frames);
	cvSetMouseCallback("dst", on_mouse, (void *)framesPT);
	cvSetMouseCallback("contour", on_mouse, (void *)framesPT);

	//hough
	frameGray = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

	//endregion

	//region main

	CvBox2D oblique; // square
	int x, y;
	double X, Y, w, h, ang, d, d0, d1, d2;
	double theta, alpha, phi;
	int straightcnt = 0, angcnt = 0;
	double targetAng = 0, targetDis = 0;
	int turnRight = 1; // bool
	State state = FINDING;
	for (int j = 0; j < 3; ++j)
	{
		while (1)
		{
			//region prepare

			frame = cvQueryFrame(capture);
			frame = cvQueryFrame(capture);
			frame = cvQueryFrame(capture);
			frame = cvQueryFrame(capture);
			cvCvtColor(frame, frameHSV, CV_RGB2HSV);
			cvWarpPerspective(frame, framePT, map_matrix, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(100));

			cvCvtColor(framePT, framePTHSV, CV_RGB2HSV);
			cvShowImage("src", frame);
			cvShowImage("dst", framePT);
			GetMaskHSV(framePT, mask, minH, maxH, minS, maxS, minV, maxV);
			GetLargestContour(framePT, mask, contour, topContoursInfo);
			cvShowImage("contour", contour);
			key = cvWaitKey(1);

			//endregion

			oblique = topContoursInfo[0].oblique; // square
			x = oblique.center.x;
			y = oblique.center.y;
			X = (double)(x - ROBOT_X);
			Y = (double)(ROBOT_Y - y); // 正になる
			d = sqrt(X * X + Y * Y);
			theta = fabs(atan(Y / X));
			ang = oblique.angle;
			w = oblique.size.width;
			h = oblique.size.height;
			if (DEBUG)
			{
				printf("state:%d\n", state);
				printf("x:%d,y:%d,ang:%f,w:%f,h:%f\n", x, y, ang, w, h);
			}
			printf("area:%f\n",topContoursInfo[0].area);
			switch (state)
			{
			case FINDING:
				if (topContoursInfo[0].area < TILE_AREA)
				{
					//FAST_TURN_LEFT;
					//SLOW_TURN_RIGHT;
					FAST_TURN_RIGHT;
					break;
				}
				if(minH != BLUE_MIN_H)
					for(int ii=0;ii<7;ii++) FAST_TURN_LEFT;
				
				motor_stop();

				//
				key = cvWaitKey(1);
				frame = cvQueryFrame(capture);
				frame = cvQueryFrame(capture);
				frame = cvQueryFrame(capture);
				frame = cvQueryFrame(capture);
				cvCvtColor(frame, frameHSV, CV_RGB2HSV);
				cvWarpPerspective(frame, framePT, map_matrix, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(100));

				cvCvtColor(framePT, framePTHSV, CV_RGB2HSV);
				cvShowImage("src", frame);
				cvShowImage("dst", framePT);
				GetMaskHSV(framePT, mask, minH, maxH, minS, maxS, minV, maxV);
				GetLargestContour(framePT, mask, contour, topContoursInfo);
				cvShowImage("contour", contour);

				oblique = topContoursInfo[0].oblique; // square
				x = oblique.center.x;
				y = oblique.center.y;
				X = (double)(x - ROBOT_X);
				Y = (double)(ROBOT_Y - y); // 正になる
				d = sqrt(X * X + Y * Y);
				theta=fabs(atan(Y/X));
				ang = oblique.angle;
				w=oblique.size.width;
				h=oblique.size.height;
				//

				if (ang + 60 > 0) alpha = copysign(ang, -X);
				else alpha = copysign(90 + ang, X);
				alpha = DEG_TO_RAD(alpha);

				if ((ang + 60) * (h - w) > 0) // 縦長に見える場合、直進->回転->直進
				{
					targetDis = d * cos(theta) * (tan(theta) + tan(alpha)) / STRAIGHT_SPEED_PX;
					phi = -alpha; // moving to frontのときの計算で結果が合うように設定
					puts("================");
					printf("phi:%f\n",RAD_TO_DEG(phi));
					printf("phi:%f\n",RAD_TO_DEG(phi));
					printf("phi:%f\n",RAD_TO_DEG(phi));
					puts("================");
					d2 = d * cos(theta) / cos(alpha); // second rotatingのときの計算で結果が合うように設定
					state = MOVING_TO_FRONT;
				}
				else // 横長に見える場合、回転->直進->回転->直進
				{
					d2 = d * sin(theta + alpha) / 2;
					d0 = d * cos(theta + alpha);
					d1 = sqrt(d0 * d0 + d2 * d2);
					phi = atan(d2 / d0);
					targetAng = (90 + RAD_TO_DEG(alpha - phi)) / SLOW_SPIN_SPEED_DEG;
					printf("alpha:%f,phi:%f,theta:%f\n",RAD_TO_DEG(alpha),RAD_TO_DEG(phi),RAD_TO_DEG(theta));
					targetDis = d1 / STRAIGHT_SPEED_PX;
					state = FIRST_ROTATING;
				}

				turnRight = X > 0;
				break;
			case FIRST_ROTATING:
				if (++angcnt < targetAng) SLOW_TURN(turnRight);
				else
				{
					angcnt = 0;
					state = MOVING_TO_FRONT;
					targetDis = d1 / STRAIGHT_SPEED_PX;
					turnRight = !turnRight;
				}
				break;
			case MOVING_TO_FRONT:
				if (straightcnt++ < targetDis) GO_STRAIGHT;
				else
				{
					targetDis = 0;
					straightcnt = 0;
					state = SECOND_ROTATING;
					targetAng = (90 - RAD_TO_DEG(phi)) / SLOW_SPIN_SPEED_DEG;
				}
				break;
			case SECOND_ROTATING:
				cvCvtColor(frame, frameGray, CV_RGB2GRAY);
				cvSmooth(frameGray, frameGray, CV_GAUSSIAN, 11, 0, 0, 0);
				storage = cvCreateMemStorage(0);
				circles = cvHoughCircles(frameGray, storage, CV_HOUGH_GRADIENT,
				                         1, 3.0, 20.0, 70.0, 10,
				                         MAX (frameGray->width, frameGray->height));
				
				avex = avey = 0;
				ctotal = circles->total;
	
				if (++angcnt < targetAng && ctotal == 0) SLOW_TURN(turnRight);
				else
				{
					if (topContoursInfo[0].area > TILE_AREA) state = STRAIGHT_VISIBLE;
					else
					{
						hough_count = 0;
						state = HOUGH;
						targetDis = d2 / STRAIGHT_SPEED_PX;
						angcnt=0;
					}
				}
				break;
			case STRAIGHT_VISIBLE:
				if (ang <= -88.0 || -2.0 <= ang) GO_STRAIGHT;
				else
					SLOW_TURN(ang < -45);
				if (topContoursInfo[0].area <= TILE_AREA)
				{
					state = HOUGH;
					targetDis = INVISIBLE_DISTANCE_CM / STRAIGHT_SPEED_CM;
				}
				break;
			case HOUGH:
				//houghの準備(情報の取得)
				cvCvtColor(frame, frameGray, CV_RGB2GRAY);
				cvSmooth(frameGray, frameGray, CV_GAUSSIAN, 11, 0, 0, 0);
				storage = cvCreateMemStorage(0);
				circles = cvHoughCircles(frameGray, storage, CV_HOUGH_GRADIENT,
				                         1, 3.0, 20.0, 70.0, 10,
				                         MAX (frameGray->width, frameGray->height));
				//円の中心値の座標を計算する(見つかった円のうち最大3つの座標の平均を取る)
				avex = avey = 0;
				ctotal = circles->total;
				if(ctotal > 4)ctotal = 3;
				for (int i = 0; i < ctotal; ++i)
				{
					p = (float *)cvGetSeqElem(circles, i);
					cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), 3,
					         CV_RGB(0, 255, 0), -1, 8, 0);
					cvCircle(frame, cvPoint(cvRound(p[0]), cvRound(p[1])), cvRound(p[2]),
					         CV_RGB(255, 0, 0), 6 - 2 * i, 8, 0);
					avex += p[0];
					avey += p[1];
				}
				if (ctotal != 0) avex /= ctotal,avey /= ctotal;
				cvShowImage("src", frame);
				//中心になるように回転、閾値を超えたらしばらく直進、を繰り返す
				if (++hough_count >= HOUGH_STRAIGHT_LIMIT  && ctotal != 0)
				{
					linenum = -2.7 * avex + 322;
					if (avey > linenum  + HOUGH_EPS) {
						if(hough_turn != 2 )SLOW_TURN_RIGHT;
						else hough_count = 0;
						hough_turn = 1;
					}
					else if (avey < linenum - HOUGH_EPS) {
						if(hough_turn != 1)SLOW_TURN_LEFT;
						else hough_count = 0;
						hough_turn = 2;					
					}
					else hough_count = 0;
				}
				else
				{
					//ラストラン判定(円が見えなくなったら直進して的を倒す)
					if (ctotal == 0){
						state = STRAIGHT_INVISIBLE;
          				targetDis = 8;
						motor_stop();
					}
					else{
						hough_turn = 0;					
						GO_STRAIGHT;
					}
			 	}
				break;
			case STRAIGHT_INVISIBLE:
				if (straightcnt++ < targetDis) GO_STRAIGHT;
				else
				{
					motor_stop();
					straightcnt = 0;
					targetDis = 0;
					state = ARRIVE;
				}
				break;
			case ARRIVE:
				motor_stop();
				state = FINDING;
				goto NEXT_TILE;
			default:
				printf("err");
				key = 'q';
				break;
			}

			if (key == 'q') break;
		}

		NEXT_TILE:

		if (j == 0)
		{
			minH = GREEN_MIN_H, maxH = GREEN_MAX_H;
			minS = GREEN_MIN_S, maxS = GREEN_MAX_S;
			minV = GREEN_MIN_V, maxV = GREEN_MAX_V;
			for (int k = 0; k < 80; ++k) GO_BACK;
		}
		else if (j == 1)
		{
			minH = BLUE_MIN_H, maxH = BLUE_MAX_H;
			minS = BLUE_MIN_S, maxS = BLUE_MAX_S;
			minV = BLUE_MIN_V, maxV = BLUE_MAX_V;
			for (int k = 0; k < 80; ++k) GO_BACK;
		}
	}
	//endregion

	//region finish

	finalize();
	cvDestroyWindow("src");
	cvDestroyWindow("contour");
	cvReleaseImage(&frameHSV);
	cvReleaseImage(&framePTHSV);
	cvReleaseImage(&mask);
	cvReleaseImage(&contour);
	cvReleaseCapture(&capture);
	//endregion

	return 0;
}
