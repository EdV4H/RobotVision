/* serial2016.h, version 2016-08-23 */
/* to be used with robot_driver2016.ino */
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#define BAUDRATE B115200

int fd;
int n;
struct termios oldtio;
unsigned char buf[6] = {255, 0, 0, 0, 0, 0};
                 // buf[0] の 255 はコマンド開始目印

//モーター電源オン，静止パルス幅設定
void motor_on(int pwL, int pwR) {
  buf[1] = 11;   // モーター電源オンコマンド
  buf[2] = 0;    // コマンド終了目印 (0固定)
  n = write(fd, buf, 3);
  usleep(500000);
  buf[1] = 13;   // モーター静止パルス幅設定コマンド
  buf[2] = 256 - pwL;  // 左モーターパルス幅調整値、中心は128
  buf[3] = 256 - pwR;  // 右モーターパルス幅調整値、中心は128
  buf[4] = 0;    // コマンド終了目印 (0固定)
  n = write(fd, buf, 5);
  usleep(50000);
}

//モーター電源オフ（脱力）
void motor_release() {
  buf[1] = 12;   // モーター脱力コマンド
  buf[2] = 0;    // コマンド終了目印 (0固定)
  n = write(fd, buf, 3);
  usleep(500000);
}

//モーター駆動
//vl, vr: 128を中心とする速度（およそ 78〜178）
void motor(int vl, int vr) {
  buf[1] = 14;   // モーター駆動コマンド
  buf[2] = 256 - vl;   // 左モーター速度
  buf[3] = 256 - vr;   // 右モーター速度
  buf[4] = 0;    // コマンド終了目印 (0固定)
  n = write(fd, buf, 5);
  usleep(50000);
}

//モーター停止（脱力はしない）
void motor_stop() {
  motor(128,128);
}

//カメラサーボ電源オン，中心アングル設定
void camera_on(int thV, int thH) {
  buf[1] = 21;   // カメラサーボ電源オンコマンド
  buf[2] = 0;    // コマンド終了目印 (0固定)
  n = write(fd, buf, 3);
  usleep(500000);
  buf[1] = 23;   // カメラサーボ中心位置設定コマンド
  buf[2] = thV;  // 真正面を向くための縦アングル値（90を中心に微調整）
  buf[3] = thH;  // 真正面を向くための横アングル値（90を中心に微調整）
  buf[4] = 0;    // コマンド終了目印 (0固定)
  n = write(fd, buf, 5);
  usleep(50000);
}

//カメラサーボ電源オフ（脱力）
void camera_release() {
  buf[1] = 22;   // カメラサーボ脱力コマンド
  buf[2] = 0;    // コマンド終了目印 (0固定)
  n = write(fd, buf, 3);
  usleep(500000);
}

//カメラサーボ縦移動
void camera_vertical(int th) {
  buf[1] = 24;   // 縦サーボ移動コマンド
  buf[2] = th;   // 角度（30〜105、真正面は90）
  buf[3] = 0;    // コマンド終了目印 (0固定)
  n = write(fd, buf, 4);
  usleep(50000);
}

//カメラサーボ横移動
void camera_horizontal(int th) {
  buf[1] = 25;   // 横サーボ移動コマンド
  buf[2] = th;   // 角度（30〜150、真正面は90）
  buf[3] = 0;    // コマンド終了目印 (0固定)
  n = write(fd, buf, 4);
  usleep(50000);
}

//LED点滅
void LED_blink(int n) {
  buf[1] = 99;   // LED点滅コマンド
  buf[2] = n;    // LED点滅回数
  buf[3] = 0;    // コマンド終了目印 (0固定)
  n = write(fd, buf, 4);
  usleep(50000);
}

// serial通信
void open_serial_port(const char *serialport) {
  struct termios toptions;

  fd = open(serialport, O_RDWR | O_NONBLOCK );
  if (fd == -1)  {
    perror("serialport_init: Unable to open port ");
    return;
  }
  /* Get currently set options for the tty */
  tcgetattr(fd,&oldtio);
  tcgetattr(fd,&toptions);
  /* Set custom options */
  cfsetispeed(&toptions, BAUDRATE);
  cfsetospeed(&toptions, BAUDRATE);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  /* no hardware flow control */
  toptions.c_cflag &= ~CRTSCTS;
  //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset    
  /* enable receiver, ignore status lines */
  toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines    
  /* disable input/output flow control, disable restart chars */
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl      
  /* disable canonical input, disable echo,
     disable visually erase chars,
     disable terminal-generated signals */
  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /* disable output processing */
  toptions.c_oflag &= ~OPOST;
  toptions.c_cc[VMIN]  = 0;
  toptions.c_cc[VTIME] = 0;

  tcsetattr(fd, TCSANOW, &toptions);
  if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
    perror("init_serialport: Couldn't set term attributes");
    return;
  }
  sleep(1);  //required to make flush work, for some reason
  tcflush(fd, TCIOFLUSH);
}

void close_serial_port(void) {
  tcsetattr(fd, TCSANOW, &oldtio);
  close(fd);
}

void finalize() {
  /*  motor_release();
      camera_release();*/
  motor_stop();
  close_serial_port();
}

void sigcatch(int sig) {
  printf("signal catch %d\n", sig);
  finalize();
  fflush(stdout);
  exit(1);
}

void init() {
  //シグナル処理
  if (SIG_ERR == signal(SIGHUP, sigcatch)
      || SIG_ERR == signal(SIGILL, sigcatch)
      || SIG_ERR == signal(SIGABRT, sigcatch)
      || SIG_ERR == signal(SIGSEGV, sigcatch)
      || SIG_ERR == signal(SIGTERM, sigcatch)
      || SIG_ERR == signal(SIGTSTP, sigcatch)
      || SIG_ERR == signal(SIGINT, sigcatch)) {
    printf("failed to set signal handler.n");
    exit(1);
  }
  open_serial_port(PORT);
}