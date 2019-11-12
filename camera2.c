//---------------------------------------------------------
// 概要 : カメラからの入力画像をそのまま表示
//---------------------------------------------------------

#include <stdio.h>
#include <highgui.h>

int main(int argc, char** argv){
  int key;                        // キー入力用の変数
  CvCapture *capture1, *capture2;  // カメラキャプチャ用の構造体
  IplImage *frameImage, *frameImage2;  // キャプチャ画像用IplImage
  char windowName[] = "Capture";  // キャプチャ画像を表示するウィンドウの名前
  char windowName2[] = "Capture2"; // キャプチャ画像を表示するウィンドウの名前

  //カメラを初期化
  capture1 = cvCaptureFromCAM(argc == 3 ? argv[1][0] - '0' : -1);
  capture2 = cvCaptureFromCAM(argc == 3 ? argv[2][0] - '0' : -1);
  if (capture1 == NULL) {
    printf("カメラ１が見つかりません\n");
    return -1;
  }
  if (capture2 == NULL) {
    printf("カメラ２が見つかりません\n");
    return -1;
  }

  cvSetCaptureProperty (capture1, CV_CAP_PROP_FRAME_WIDTH, 320);
  cvSetCaptureProperty (capture1, CV_CAP_PROP_FRAME_HEIGHT, 240);
  cvSetCaptureProperty (capture2, CV_CAP_PROP_FRAME_WIDTH, 320);
  cvSetCaptureProperty (capture2, CV_CAP_PROP_FRAME_HEIGHT, 240);

  cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);  // ウィンドウ生成
  cvNamedWindow(windowName2, CV_WINDOW_AUTOSIZE);  // ウィンドウ生成

  while (1) {
    // カメラからの入力画像1フレームをframeImageに格納
    frameImage = cvQueryFrame(capture1);
    frameImage2 = cvQueryFrame(capture2);
    cvShowImage(windowName, frameImage); // 画像を表示
    cvShowImage(windowName2, frameImage2); // 画像を表示

    key = cvWaitKey(1);     // 'q'キーが入力されたら
    if (key == 'q') break;  // ループ脱出
  }

  cvReleaseCapture(&capture1);   // キャプチャ用構造体を解放
  cvReleaseCapture(&capture2);   // キャプチャ用構造体を解放
  cvDestroyWindow(windowName);  // ウィンドウを破棄
  return 0;
}
