#include <stdio.h>
#include <curses.h>

#define PORT "/dev/ttyACM0"
#define MOTOR_DEFAULT_L 128
#define MOTOR_DEFAULT_R 128
#include "serial2016.h"

typedef enum {l_inc, l_dec, r_inc, r_dec} servo_operation_t;

void s_swing(servo_operation_t so, int *current_l, int *current_r)
{
  switch(so) {
  case l_inc: *current_l += 1; break;
  case l_dec: *current_l -= 1; break;
  case r_inc: *current_r += 1; break;
  case r_dec: *current_r -= 1; break;
  }
  motor(*current_l, *current_r);
  mvprintw(6, 0, "L = %d, R = %d\n", *current_l, *current_r);
}

int main()
{
  int dir;
  int current_l = MOTOR_DEFAULT_L;
  int current_r = MOTOR_DEFAULT_R;

  init();
  motor_on(128, 128);
  motor_stop();
  initscr();
  cbreak();
  noecho();
//  motor(MOTOR_DEFAULT_L, MOTOR_DEFAULT_R);
  keypad(stdscr, TRUE);
  printw("Adjust orientation by   h / Leftarrow\n");
  printw("                     or j / Downarrow\n");
  printw("                     or k / Uparrow\n");
  printw("                     or l / Rightarrow.\n");
  printw("Press q to quit.\n");

  int loop = 1;

  while(loop) {
    dir = getch();
    switch(dir) {
    case 'k':
    case KEY_UP:
      s_swing(r_inc, &current_l, &current_r);
      break;
    case 'j':
    case KEY_DOWN:
      s_swing(r_dec, &current_l, &current_r);
      break;
    case 'h':
    case KEY_LEFT:
      s_swing(l_inc, &current_l, &current_r);
      break;
    case 'l':
    case KEY_RIGHT:
      s_swing(l_dec, &current_l, &current_r);
      break;
    case 'q':
      loop = 0;
      break;
    default:
      break;
    }
  }
  endwin();
  finalize();
  return 0;
}