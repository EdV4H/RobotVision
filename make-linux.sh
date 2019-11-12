#!/bin/sh
g++ -O3 -Wno-unused-result $* `pkg-config --cflags --libs opencv` -I/usr/include/ncurses -lncurses
