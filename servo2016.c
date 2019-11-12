#include <stdio.h>
#include <curses.h>

#define PORT "/dev/ttyACM0"
#define SERVO_DEFAULT_V 90
#define SERVO_DEFAULT_H 90
#include "serial2016.h"

typedef enum {v_inc, v_dec, h_inc, h_dec} servo_operation_t;

void s_swing(servo_operation_t so, int *current_v, int *current_h)
{
  switch(so) {
  case v_inc:
    *current_v += 1;
    camera_vertical(*current_v);
    break;
  case v_dec:
    *current_v -= 1;
    camera_vertical(*current_v);
    break;
  case h_inc:
    *current_h += 1;
    camera_horizontal(*current_h);
    break;
  case h_dec:
    *current_h -= 1;
    camera_horizontal(*current_h);
    break;
  }
  mvprintw(6, 0, "V = %d, H = %d\n", *current_v, *current_h);
}

int main()
{
  int dir;
  int current_v = 90;
  int current_h = 90;

  init();
  camera_on(SERVO_DEFAULT_V, SERVO_DEFAULT_H);
  camera_vertical(current_v);
  camera_horizontal(current_h);
  initscr();
  cbreak();
  noecho();
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
      s_swing(v_inc, &current_v, &current_h);
      break;
    case 'j':
    case KEY_DOWN:
      s_swing(v_dec, &current_v, &current_h);
      break;
    case 'h':
    case KEY_LEFT:
      s_swing(h_inc, &current_v, &current_h);
      break;
    case 'l':
    case KEY_RIGHT:
      s_swing(h_dec, &current_v, &current_h);
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
