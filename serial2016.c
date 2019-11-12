#include <stdio.h>
#define PORT "/dev/ttyACM0"  // 適宜変更のこと
#include "serial2016.h"

int main(void)
{
  init();
  motor_on(135,121); // モーター静止パルス幅調整（左，右）
  motor_stop();
  camera_on(90,89);  // カメラ中心アングル調整（縦，横）
  camera_vertical(80);
  camera_horizontal(90);
  usleep(500000);

  // モーターは128を中心に大きな値は正回転，小さな値は逆回転
  int i,k;
  for (i=0; i<2; i++) {
  for (k=5; k<60; k+=3) {
    motor(128-k, 128-k);
    usleep(50000);
    printf("%5d\n", k);
  }
  //  usleep(1000000);
  for (k=60; k>0; k-=3) {
    motor(128-k, 128-k);
    usleep(50000);
  }
  motor(128+20,128-20);
  usleep(2700000);
  }
  motor_stop();
  /*
  for (k=0; k<60; k+=10) {
    motor(128+k, 128+k);
    usleep(100000);
  }
  usleep(1000000);
  for (k=60; k>0; k-=10) {
    motor(128+k, 128+k);
    usleep(100000);
  }
  motor(178,178);    //前進
  usleep(1000000);
  motor(78,78);      //後退
  usleep(100000);
  motor(128,150);    //左折
  usleep(100000);
  motor(138,118);    //その場右回転
  usleep(100000);
  */
  motor_stop();           //停止
  camera_horizontal(120);
  usleep(500000);
  camera_horizontal(60);
  usleep(500000);
  camera_horizontal(90);
  camera_vertical(80);
  usleep(500000);

  finalize();
  return 0;
}