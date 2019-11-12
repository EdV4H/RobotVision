#include <stdio.h>
#include <cv.h>
#include <highgui.h>

int main (int argc, char **argv)
{
  int i, key;
  float *p;

  CvCapture *capture;        // カメラキャプチャ用の構造体
  IplImage *frame;           // キャプチャ画像用IplImage
  IplImage *frameGray = NULL;
  CvMemStorage *storage;
  CvSeq *circles = NULL;

  //カメラを初期化
  if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
    capture = cvCaptureFromCAM(argc == 2 ? argv[1][0] - '0' : -1);
  if (capture == NULL) {
    printf("カメラが見つかりません\n");
    return -1;
  }

  // 解析速度向上のために画像サイズをカメラの能力よりも下げる
  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_WIDTH, 320);
  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_HEIGHT, 240);

  cvNamedWindow("src", CV_WINDOW_AUTOSIZE);  // ウィンドウ生成
  cvNamedWindow("circles", CV_WINDOW_AUTOSIZE);

  frame = cvQueryFrame(capture);
  frameGray = cvCreateImage (cvGetSize(frame), IPL_DEPTH_8U, 1);

  while (1) {
    // (1) カメラからの入力画像1フレームをframeに格納
    frame = cvQueryFrame(capture);
    cvCvtColor(frame, frameGray, CV_RGB2GRAY);

    // (2)ハフ変換のための前処理（画像を平滑化しないと誤検出が出やすい）
    cvSmooth (frameGray, frameGray, CV_GAUSSIAN, 11, 0, 0, 0);
    storage = cvCreateMemStorage (0);

    // (3)ハフ変換による円の検出と検出した円の描画
    circles = cvHoughCircles (frameGray, storage, CV_HOUGH_GRADIENT, 
                              1, 3.0, 20.0, 70.0, 10,
                              MAX (frameGray->width, frameGray->height));
    for (i = 0; i < MIN (3,circles->total); i++) {
      p = (float *) cvGetSeqElem (circles, i);
      cvCircle (frame, cvPoint (cvRound (p[0]), cvRound (p[1])),
		3, CV_RGB (0,255,0), -1, 8, 0);
      cvCircle (frame, cvPoint (cvRound (p[0]), cvRound (p[1])),
		cvRound (p[2]), CV_RGB (255,0,0), 6-2*i, 8, 0);
    }

    // (4) 検出結果表示用のウィンドウを確保し表示する
    cvShowImage ("circles", frame);
    cvShowImage ("src", frameGray);
    key = cvWaitKey(1);       // 'q'キーが入力されたら
    if (key == 'q') break;      // ループ脱出
  }

  cvDestroyWindow ("circles");
  cvReleaseImage (&frameGray);
  cvReleaseMemStorage (&storage);

  cvReleaseCapture(&capture);   // キャプチャ用構造体を解放
  cvDestroyWindow("src");       // ウィンドウを破棄

  return 0;
}
