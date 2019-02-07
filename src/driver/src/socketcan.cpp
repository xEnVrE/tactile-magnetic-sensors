#include "socketcan.h"
#include <iostream>
#include <unistd.h>

socketCAN::socketCAN()
{

}

socketCAN::~socketCAN()
{
    mutex.lock();
    read_can_port = false;
    mutex.unlock();
    readThread.join();
}


//int openCAN(const std::string &channel);

//int socketCAN::open_port(const char *port)
int socketCAN::openCAN(const std::string &channel)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(soc < 0)
    {
        return (-1);
    }

    addr.can_family = AF_CAN;
    //strcpy(ifr.ifr_name, port);
    strcpy(ifr.ifr_name, channel.c_str());
    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
    {

        return (-1);
    }

    addr.can_ifindex = ifr.ifr_ifindex;

    fcntl(soc, F_SETFL, O_NONBLOCK);

    if (bind(soc, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {

        return (-1);
    }

    readThread = std::thread(&socketCAN::read_port,this);

    return 0;
}


//int socketCAN::send_port(struct can_frame *frame)
int socketCAN::writeCAN(unsigned int id, unsigned short len, unsigned char *data)
{
    can_frame frame;

    frame.can_id = id;
    frame.can_dlc = static_cast<unsigned char>(len);
    for (unsigned i = 0; i<len; i++)
        frame.data[i] = data[i];

    long retval;
    retval = write(soc, &frame, sizeof(struct can_frame));
    if (retval != sizeof(struct can_frame))
    {
        //std::cout << "CAN: Error" << std::endl;
        return (-1);
    }
    else
    {
        //std::cout << "CAN: Ok" << std::endl;
        return (0);
    }
}

/* this is just an example, run in a thread */
void socketCAN::read_port()
{
    struct can_frame frame_rd;
    long recvbytes = 0;

    read_can_port = 1;
    while(read_can_port)
    {
        struct timeval timeout = {1, 0};
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(soc, &readSet);

        if (select((soc + 1), &readSet, nullptr, nullptr, &timeout) >= 0)
        {
            if (!read_can_port)
            {
                break;
            }
            if (FD_ISSET(soc, &readSet))
            {
                recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
                if(recvbytes)
                {
                    //printf("id = %d, dlc = %d, data = %s\n", frame_rd.can_id, frame_rd.can_dlc,frame_rd.data);

                    mutex.lock();
                    buffer.push(frame_rd);
                    mutex.unlock();

                }
            }
        }

    }

}


// Take a frame from FIFO buffer
int socketCAN::readCAN(unsigned int *id, unsigned short *len, unsigned char *data){

    if(buffer.size()){

        //remove a frame from buffer
        mutex.lock();
        can_frame frame = buffer.front();
        buffer.pop();
        mutex.unlock();


        *id = frame.can_id;
        *len = frame.can_dlc;

        for (unsigned short i=0; i<*len; i++)
            data[i] = frame.data[i];

        return 0;
    }else

        return -1;


}

//int socketCAN::close_port()
int socketCAN::closeCAN()
{
    close(soc);
    return 0;
}
