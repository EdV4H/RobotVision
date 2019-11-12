#include <stdio.h>
#include <curses.h>

#define PORT "/dev/ttyACM0"
#define MOTOR_DEFAULT_L 135
#define MOTOR_DEFAULT_R 120
#define SERVO_DEFAULT_V 86
#define SERVO_DEFAULT_H 92
#define STEP 12
#include "serial2016.h"

typedef enum {inc, dec, left, right} motor_operation_t;
typedef enum {v_inc, v_dec, h_inc, h_dec} servo_operation_t;

void change_speed(motor_operation_t so, int *lp, int *rp,
		  int step)
{
  switch(so) {
  case inc: *lp -= step; *rp -= step; break;
  case dec: *lp += step; *rp += step; break;
  case left: *lp += step; *rp -= step; break;
  case right: *lp -= step; *rp += step; break;
  }
  motor(*lp, *rp);
  mvprintw(4, 0, "L = %3d, R = %3d\n", *lp, *rp);
}

void s_swing(servo_operation_t so, int *vp, int *hp)
{
  switch(so) {
  case v_inc:
    *vp += 1;
    camera_vertical(*vp);
    break;
  case v_dec:
    *vp -= 1;
    camera_vertical(*vp);
    break;
  case h_inc:
    *hp += 1;
    camera_horizontal(*hp);
    break;
  case h_dec:
    *hp -= 1;
    camera_horizontal(*hp);
    break;
  }
  mvprintw(5, 0, "V = %3d, H = %3d\n", *vp, *hp);
}

int main()
{
  int dir;
  int l = 128;
  int r = 128;
  int step = STEP;
  int v = 72;
  int h = 90;

  init();
  motor_on(MOTOR_DEFAULT_L, MOTOR_DEFAULT_R);
  motor_stop();
  camera_on(SERVO_DEFAULT_V,SERVO_DEFAULT_H);
  //  usleep(500000);    //temp
  camera_vertical(72);
  camera_horizontal(90);

  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  printw("Control speed/turn by arrow keys\n");
  printw("and servo orientaion by h/j/k/l keys.\n");
  printw("Press q to quit.\n");

  int loop = 1;

  while(loop) {
    dir = getch();
    switch(dir) {
    case KEY_UP:
      change_speed(inc, &l, &r, step);
      break;
    case KEY_DOWN:
      change_speed(dec, &l, &r, step);
      break;
    case KEY_LEFT:
      change_speed(left, &l, &r, step);
      break;
    case KEY_RIGHT:
      change_speed(right, &l, &r, step);
      break;
    case 'z':
      l = 128 + step;
      r = 128 + step;
      change_speed(inc, &l, &r, step);
      break;
    case 'k':
      s_swing(v_inc, &v, &h);
      break;
    case 'j':
      s_swing(v_dec, &v, &h);
      break;
    case 'h':
      s_swing(h_inc, &v, &h);
      break;
    case 'l':
      s_swing(h_dec, &v, &h);
      break;
    case 'q':
      loop = 0;
      break;
    default:
      break;
    }
  }
  endwin();
  motor_stop();
  finalize();
  return 0;
}
