#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <stdlib.h>

#include "xbee/dataIn.h"
#include "xbee/dataSend.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

ros::Publisher info_XBee_pub;
struct termios optionsDeBase;
int serial_fd;

unsigned char write_buf[] =
        {0x7E, 0x00, 0x16, 0x10, 0x01, 0x00, 0x13,
         0xA2, 0x00, 0x40, 0xE4, 0x22, 0x64, 0xFF,
         0xFE, 0x00, 0x00, 0x41, 0x42, 0x43, 0x44,
         0x41, 0x42, 0x43, 0x43, 0x7F};

//Initialize serial port
int initport(int fd)
{
    int portstatus = 0;

    struct termios optionsPSC;

    // Get the current options for the port...
    tcgetattr(fd, &optionsDeBase);
    tcgetattr(fd, &optionsPSC);
    // Set the baud rates to 9600...
    cfsetispeed(&optionsPSC, B9600);
    cfsetospeed(&optionsPSC, B9600);

    // Enable the receiver and set local mode...
    optionsPSC.c_cflag |= (CLOCAL | CREAD);

    optionsPSC.c_cflag &= ~PARENB;
    optionsPSC.c_cflag &= ~CSTOPB;
    optionsPSC.c_cflag &= ~CSIZE;
    optionsPSC.c_cflag |= CS8;
    //options.c_cflag |= SerialDataBitsInterp(8);     /* CS8 - Selects 8 data bits */
    optionsPSC.c_cflag &= ~CRTSCTS;                      // Disable hardware flow control
    optionsPSC.c_iflag &= ~(IXON | IXOFF | IXANY);       // Disable XON XOFF (for transmit and receive)
    //options.c_cflag |= CRTSCTS;                     /* Enable hardware flow control */

    optionsPSC.c_cc[VMIN] = 1;   //Minimum characters to be read
    optionsPSC.c_cc[VTIME] = 2;    //Time to wait for data (tenths of seconds)
    optionsPSC.c_oflag &=~OPOST;
    optionsPSC.c_iflag &=~(ICANON | ECHO | ECHOE | ISIG);
    // Set the new options for the port...
    tcsetattr(fd, TCSANOW, &optionsPSC);

    //Set the new options for the port...
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &optionsPSC) == -1)
    {
        perror("On tcsetattr:");
        portstatus = -1;
    }
    else
        portstatus = 1;
    return (portstatus);
}

int open_port(void)
{
    int fd; /* File descriptor for the port */
    fd = open("/dev/ttyUSB2", O_RDWR | O_NOCTTY | O_NONBLOCK);   //Attention à modifier à la main ttyUSB

    if (fd == -1)
    {
        /*      Could not open the port.        */
        perror("Open_port: Unable to open /dev/ttyUSB2 --- \n");
    }

    return (fd);
}

//Fonction pour resst le termios
void reset (int fd)
{
    // Set the new options for the port...
    tcsetattr(fd, TCSANOW, &optionsDeBase);

    //Set the new options for the port...
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &optionsDeBase) == -1)
    {
        perror("On tcsetattr:");
    }
}

//Traiter le fait que le message n'est pas transmis via un topic
void envoyerData(const xbee::dataSend& msg)
{
    unsigned char msg_buf[]=msg;
    int n = write(serial_fd, &msg_buf, sizeof msg_buf);
    if (n < 0)
        fputs("write() failed!\n", stderr);
    else
    {
        printf("Successfully wrote %d bytes\n", (int) sizeof (write_buf));
        for (i=0; i<n; i++)
        {
            printf("%c ",write_buf[i]);
        }
    }
}

//Réordonner le code la c'est de la merde
int main(int argc, char **argv)
{
    int i;
    serial_fd = open_port();

    if (serial_fd == -1)
        printf("Error opening serial port /dev/ttyUSB0 \n");
    else
    {
        printf("Serial Port /dev/ttyUSB0 is Open\n");
        if (initport(serial_fd) == -1)
        {
            printf("Error Initializing port");
            reset(serial_fd);
            close(serial_fd);
            return EXIT_SUCCESS;
        }
        ros::init(argc, argv, "xbee");
        ros::NodeHandle n;
        info_XBee_pub = n.advertise<xbee::dataIn>("dataReceived", 100);
        ros::Subscriber info_XBee_sub = n.subscribe("dataSend", 100, envoyerData);
        ros::spin();
        while (ros::ok())
        {
            xbee::dataIn msg;

            char read_buf[128];
            int n1 = read(serial_fd, read_buf, sizeof read_buf);
            if (n1 < 0)
                fputs("Read failed!\n", stderr);
            else
            {
                printf("Successfully read from serial port -- %s\n With %d Bytes", read_buf,n1);
            }
            msg.mes=read_buf;

            info_XBee_pub.publish(msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
        printf("\n\nNow closing serial port /dev/ttyUSB0 \n\n");
        close(serial_fd);

        return 0;
    }
    return 0;
}
