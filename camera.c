//---------------------------------------------------------
// 概要 : カメラからの入力画像をそのまま表示
//---------------------------------------------------------

#include <stdio.h>
#include <highgui.h>

int main(int argc, char** argv){
  int key;                        // キー入力用の変数
  CvCapture *capture;             // カメラキャプチャ用の構造体
  IplImage *frameImage;           // キャプチャ画像用IplImage
  char windowName[] = "Capture";  // キャプチャ画像を表示するウィンドウの名前

  //カメラを初期化
  if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
    capture = cvCaptureFromCAM(argc == 2 ? argv[1][0] - '0' : -1);
  if (capture == NULL) {
    printf("カメラが見つかりません\n");
    return -1;
  }

  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_WIDTH, 320);
  cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_HEIGHT, 240);
  cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);  // ウィンドウ生成

  while (1) {
    // カメラからの入力画像1フレームをframeImageに格納
    frameImage = cvQueryFrame(capture);
    cvShowImage(windowName, frameImage); // 画像を表示

    key = cvWaitKey(1);     // 1ms の間に'q'キーが入力されたら
    if (key == 'q') break;  // ループ脱出
  }

  cvReleaseCapture(&capture);   // キャプチャ用構造体を解放
  cvDestroyWindow(windowName);  // ウィンドウを破棄
  return 0;
}
