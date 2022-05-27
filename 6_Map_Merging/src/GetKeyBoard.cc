//
// Created by hazyparker on 22-5-27.
//

#include "GetKeyBoard.h"
#include <cstdio>
#include <termios.h>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <iostream>

using namespace std;

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42


int kfd = 0;
struct termios cooked, raw;

int main() {
    char c;
    bool dirty = false;


    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");


    for (;;) {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }

        switch (c) {
            case KEYCODE_L:
                cout << "left is pressed" << endl;
                dirty = true;
                break;
            case KEYCODE_R:
                cout << "right is pressed" << endl;
                dirty = true;
                break;
            case KEYCODE_U:
                cout << "up is pressed" << endl;
                dirty = true;
                break;
            case KEYCODE_D:
                cout << "down is pressed" << endl;
                dirty = true;
                break;
        }
        return 0;
    }
}