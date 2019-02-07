#ifndef SOCKETCAN_H
#define SOCKETCAN_H

#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <unistd.h>

#include <thread>
#include <mutex>
#include <queue>

class socketCAN
{
public:
    socketCAN();
    ~socketCAN();

    //int open_port(const char *port);
    int openCAN(const std::string &channel);
    //int close_port();
    int closeCAN();
    //int send_port(struct can_frame *frame);
    int writeCAN(unsigned int id, unsigned short len, unsigned char *data);

    int readCAN(unsigned int *id, unsigned short *len, unsigned char *data);


    // This function runs of thread
    void read_port();

    //int readCAN(unsigned int *id, unsigned short *len, unsigned char *data);


private:
    int soc;
    int read_can_port;

    std::thread readThread;
    std::mutex mutex;

    std::queue<can_frame> buffer;

};

#endif // SOCKETCAN_H
