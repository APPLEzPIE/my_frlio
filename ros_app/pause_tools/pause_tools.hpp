#include <iostream>
#include <string>
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
//新添加
#include <boost/thread/thread.hpp>
// #include <pcl/visualization/cloud_viewer.h>

#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_W 0x77
#define KEYCODE_U 0x75
#define KEYCODE_V 0x76
#define KEYCODE_SPACE 0x20

int kfd = 0;
struct termios cooked, raw;
bool done;
double rate = 1;
bool go = true;
bool step = false;
bool system_shoutdown = false;

void SigHandle(int sig)
{
    system_shoutdown = true;
}

bool pause_detect()
{
    if (!go && !step) {
        return true;
    } else if (!go && step) {
        step = false;
        return false;
    } else if (go && !step) {
        step = true;
        return false;
    }
}
void keyboardLoop()
{
    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    while (!system_shoutdown) {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0) {
            perror("poll():");
            return;
        } else if (num > 0) {
            if (read(kfd, &c, 1) < 0) {
                perror("read():");
                return;
            }
        } else {
            if (dirty == true) {
                // stopRobot();
                dirty = false;
            }

            continue;
        }

        switch (c) {
        case KEYCODE_SPACE:
            go = !go;
            step = !step;
            if (!go) {
                std::cout << "stop" << std::endl;
            } else {
                std::cout << "coninue" << std::endl;
            }
            break;
        case KEYCODE_Z:
            if (rate <= 1.1) {
                rate /= 2;
            } else {
                rate -= 1;
            }
            std::cout << "rate:" << rate << std::endl;
            dirty = true;
            break;
        case KEYCODE_X:
            if (rate <= 1.1) {
                rate *= 2;
            } else {
                rate += 1;
            }
            std::cout << "rate:" << rate << std::endl;
            dirty = true;
            break;
        case KEYCODE_W:
            step = true;
            break;
        default:
            std::cout << "undefined !" << std::endl;
        }
    }
}