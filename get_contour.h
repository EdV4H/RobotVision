#ifndef GET_CONTOUR_H_
#define GET_CONTOUR_H_

#include <stdio.h>
#include <cv.h>
#include <highgui.h>

/* for cvThreshold */
#define THRESHOLD 10        // 2値化の際の閾値

/* for cvLine, cvDrawContours and cvRectangle */
#define LINE_THICKNESS  2
#define LINE_TYPE 8

#define CONTOURS 3

typedef struct contourInfo {
  CvBox2D oblique;        // 最小包囲長方形
  CvRect  horizontal;     // 水平最小包囲長方形
  double  area;           // 元の輪郭の面積
  double  obliqueArea;    // oblique の面積
  double  horizontalArea; // horizontal の面積
} contourInfo;

/*
 * HSVによる二値化画像 (1 channel) の取得
 */
void GetMaskHSV(IplImage* src, IplImage* mask, uchar minH, uchar maxH,
                uchar minS, uchar maxS, uchar minV, uchar maxV)
{
  IplImage *tmp = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
  IplImage *tmpH = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  IplImage *tmpS = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  IplImage *tmpV = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  IplImage *tmp1 = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  IplImage *tmp2 = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);

  cvSmooth (src, tmp, CV_GAUSSIAN, 7, 0, 0, 0);
  cvCvtColor(tmp, tmp, CV_RGB2HSV);
  cvSplit(tmp, tmpH, tmpS, tmpV, NULL);
  cvThreshold(tmpH, tmp1, minH, 255, CV_THRESH_BINARY);
  cvThreshold(tmpH, tmp2, maxH, 255, CV_THRESH_BINARY_INV);
  cvAnd(tmp1, tmp2, tmpH, NULL);
  cvThreshold(tmpS, tmp1, minS, 255, CV_THRESH_BINARY);
  cvThreshold(tmpS, tmp2, maxS, 255, CV_THRESH_BINARY_INV);
  cvAnd(tmp1, tmp2, tmpS, NULL);
  cvThreshold(tmpV, tmp1, minV, 255, CV_THRESH_BINARY);
  cvThreshold(tmpV, tmp2, maxV, 255, CV_THRESH_BINARY_INV);
  cvAnd(tmp1, tmp2, tmpV, NULL);
  cvAnd(tmpH, tmpS, mask, NULL);
  cvAnd(mask, tmpV, mask, NULL);
  cvReleaseImage(&tmp);
  cvReleaseImage(&tmpH);
  cvReleaseImage(&tmpS);
  cvReleaseImage(&tmpV);
  cvReleaseImage(&tmp1);
  cvReleaseImage(&tmp2);
}

/*
 * 最大輪郭と外接矩形の計算，描画
 */
void GetLargestContour(IplImage *frame, IplImage *src, IplImage *ret,
                       contourInfo topContoursInfo[CONTOURS])
{
  IplImage *bin = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1); // 2値画像
  
  CvMemStorage *storage = cvCreateMemStorage(0);  // 抽出された輪郭の保存領域
  CvSeq *contours = NULL;   // 輪郭へのポインタ
  CvSeq *it;
  CvSeq *topC[CONTOURS+2];
  double topA[CONTOURS+2];
  double area;
  CvPoint pt, pt0;
  CvPoint2D32f box_vtx[4];
  int i,j;

  // 輪郭抽出
  cvThreshold(src, bin, THRESHOLD, 255, CV_THRESH_BINARY);
  cvCvtColor(src, ret, CV_GRAY2BGR);
  cvAnd(frame, ret, ret, NULL);
  cvFindContours(bin, storage, &contours, sizeof(CvContour),
                 CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

  for (i = 1; i <= CONTOURS; i++) {
    topC[i] = NULL;
    topA[i] = 0.0;
  }
  it = contours;
  while (it != NULL) {       // insertion sort for top CONTOURS contours
    area = cvContourArea(it, CV_WHOLE_SEQ, 0);
    topA[0] = area;          // topA[0] for sentinel
    i = CONTOURS;
    while (area > topA[i]) {
      topC[i+1] = topC[i];   // topC[CONTOURS+1] is for spill
      topA[i+1] = topA[i];   // topA[CONTOURS+1] is for spill
      i--;
    } 
    topC[i+1] = it;
    topA[i+1] = area;
    it = it->h_next;
  }

  for (i=0; i<CONTOURS; i++) {
    contourInfo *ci = &topContoursInfo[i];
    ci->area = topA[i+1];
    if (topA[i+1] == 0.0) break;
    ci->oblique = cvMinAreaRect2 (topC[i+1], NULL);
    ci->obliqueArea = ci->oblique.size.height * ci->oblique.size.width;
    cvBoxPoints(ci->oblique, box_vtx);
    ci->horizontal = cvBoundingRect (topC[i+1], 0);
    ci->horizontalArea = ci->horizontal.height * ci->horizontal.width;
    pt0 = cvPointFrom32f (box_vtx[3]);
    for (j = 0; j < 4; j++) {
      pt = cvPointFrom32f (box_vtx[j]);
      cvLine (ret, pt0, pt, CV_RGB(255,255,0),
	      LINE_THICKNESS, LINE_TYPE, 0);
      pt0 = pt;
    }
    cvDrawContours(ret, topC[i+1], CV_RGB(0,0,255), CV_RGB(0,0,255),
		   0, LINE_THICKNESS, LINE_TYPE, cvPoint(0, 0));
    cvRectangle(ret, cvPoint(ci->horizontal.x, ci->horizontal.y),
		cvPoint(ci->horizontal.x + ci->horizontal.width,
			ci->horizontal.y + ci->horizontal.height),
		CV_RGB(0,255,0), LINE_THICKNESS, LINE_TYPE, 0);
  }

  cvReleaseImage(&bin);
  cvReleaseMemStorage(&storage);
}

#endif /* GET_CONTOUR_H_ */
