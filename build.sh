#!/bin/sh
echo $1 | sed -r "s/(.*)\.[a-zA-Z]*/\1/" | xargs g++ -O3 -Wno-unused-result $* `pkg-config --cflags --libs opencv` -I/usr/include/ncurses -lncurses -o
