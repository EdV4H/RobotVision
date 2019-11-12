#include <stdio.h>
#include <ctype.h>
#include <cv.h>
#include <highgui.h>
#include <curses.h>
#include "get_contour.h"

void on_mouse(int event, int x, int y, int flags, void *param);

int main(int argc, char **argv)
{
  CvCapture* capture = NULL;
  IplImage* frame;     // キャプチャ画像 (RGB)
  IplImage* framep;    // 座標変換画像 (RGB)
  IplImage* frameHSV;  // 座標変換画像 (HSV)
  IplImage* mask;      // 指定値によるmask (１チャネル)
  IplImage* contour;   // GetLargestContour() の結果
  IplImage** frames[] = {&framep, &frameHSV};
  contourInfo topContoursInfo[CONTOURS];

  CvMat *map_matrix;
  CvPoint2D32f src_pnt[4], dst_pnt[4];

  src_pnt[0] = cvPoint2D32f(95.0, 70.0);
  src_pnt[1] = cvPoint2D32f(50.0, 170.0);
  src_pnt[2] = cvPoint2D32f(270.0, 170.0);
  src_pnt[3] = cvPoint2D32f(225.0, 70.0);
  dst_pnt[0] = cvPoint2D32f(124, 110);
  dst_pnt[1] = cvPoint2D32f(124, 208.0);
  dst_pnt[2] = cvPoint2D32f(196, 208.0);
  dst_pnt[3] = cvPoint2D32f(196, 110);
  map_matrix = cvCreateMat (3, 3, CV_32FC1);
  cvGetPerspectiveTransform (src_pnt, dst_pnt, map_matrix);

  // 赤系のHSV色．各自チューニングすること
  uchar minH = 110, maxH = 140;
  uchar minS = 100, maxS = 220;
  uchar minV = 120, maxV = 220;
/*uchar minH =  30, maxH =  50;
  uchar minS =  90, maxS = 190;
  uchar minV =  60, maxV = 180;
  uchar minH =  10, maxH =  30;
  uchar minS = 100, maxS = 255;
  uchar minV = 100, maxV = 200;*/

  if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
    capture = cvCaptureFromCAM(argc == 2 ? argv[1][0] - '0' : -1);
  if (capture == NULL) {
    printf("カメラが見つかりません\n");
    return -1;
  }

  // 解析速度向上のために画像サイズを下げる
  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_WIDTH, 320);
  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_HEIGHT, 240);

  frame = cvQueryFrame(capture);
  framep = cvCloneImage(frame);
  frameHSV = cvCreateImage(cvGetSize(framep), IPL_DEPTH_8U, 3);
  mask = cvCreateImage(cvGetSize(framep), IPL_DEPTH_8U, 1);
  contour = cvCreateImage(cvGetSize(framep), IPL_DEPTH_8U, 3);

  cvNamedWindow("src", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("dst", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("contour", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("src", 60,480);
  cvMoveWindow("dst", 380,480);
  cvMoveWindow("contour", 700,480);
  cvSetMouseCallback("src", on_mouse, (void *)frames);
  cvSetMouseCallback("dst", on_mouse, (void *)frames);
  cvSetMouseCallback("contour", on_mouse, (void *)frames);
  
  initscr();

  while (1) {
    //    cvGrabFrame(capture);
    frame = cvQueryFrame(capture);
    cvWarpPerspective (frame, framep, map_matrix, 
        CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (100));
    cvCvtColor(framep, frameHSV, CV_RGB2HSV);
    GetMaskHSV(framep, mask, minH, maxH, minS, maxS, minV, maxV);
    GetLargestContour(framep, mask, contour, topContoursInfo);
    mvprintw(0,0,"yellow");
    mvprintw(1,0,"center:x=%d  ",(int)topContoursInfo[0].oblique.center.x);
    mvprintw(2,7,"y=%d  ",(int)topContoursInfo[0].oblique.center.y);
    mvprintw(3,0,"width :%d   ",(int)topContoursInfo[0].oblique.size.width);
    mvprintw(4,0,"height:%d   ",(int)topContoursInfo[0].oblique.size.height);
    mvprintw(5,0,"angle :%d  ",(int)topContoursInfo[0].oblique.angle);
    mvprintw(6,0,"area  :%d",(int)topContoursInfo[0].obliqueArea);
    mvprintw(8,0,"green");
    mvprintw(9,0,"width :%d    ",(int)topContoursInfo[0].horizontal.width);
    mvprintw(10,0,"height:%d    ",(int)topContoursInfo[0].horizontal.height);
    mvprintw(11,0,"area  :%d",(int)topContoursInfo[0].horizontalArea);
    refresh();
    cvShowImage("src", frame);
    cvShowImage("dst", framep);
    cvShowImage("contour", contour);
  
    //    usleep(500000);
    if (cvWaitKey(25) == 'q') break;
  }

  endwin();
  cvDestroyWindow("src");
  cvDestroyWindow("dst");
  cvDestroyWindow("contour");
  cvReleaseImage(&frameHSV);
  cvReleaseImage(&mask);
  cvReleaseImage(&contour);
  cvReleaseCapture(&capture);
  return 0;
}

void on_mouse(int event, int x, int y, int flags, void *frames)
{
  CvScalar BGR, HSV;
  if (event == CV_EVENT_MOUSEMOVE) {
      BGR = cvGet2D(*(((IplImage***)frames)[0]), y, x);
      HSV = cvGet2D(*(((IplImage***)frames)[1]), y, x);
      mvprintw(13,0,"(%3d,%3d): RGB=(%3.0f,%3.0f,%3.0f) HSV=(%3.0f,%3.0f,%3.0f)\n",
             x, y, BGR.val[2], BGR.val[1], BGR.val[0],
             HSV.val[0], HSV.val[1], HSV.val[2]);
    }
}
