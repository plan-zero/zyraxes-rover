#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <time.h>
#include <termios.h>

int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    return -1;
}

size_t get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

size_t get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    int a = 1;
    if(x<0) {
        a = -1;
        x = -x;
    }
    if(x<in_min) return 0;

    return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)*a;
}

int set_interface_attribs (int fd, int speed, int parity) {
    struct termios tty;
    if (tcgetattr (fd, &tty) != 0) {
        printf("error %d from tcgetattr", errno);
        return -1;
    }
    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        printf("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0) {
            printf("error %d from tggetattr", errno);
            return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
            printf("error %d setting term attributes", errno);
}

int init_serial() {
    char *portname = "/dev/ttyS0";
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("error %d opening %s: %s", errno, portname, strerror (errno));
        exit(-1);
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking
    return fd;
}

int main(int argc, char *argv[]) {
    struct timespec start, stop;
    const char *device;
    int js;
    struct js_event event;
    int deadzone = 2000;
    int jx,jy = 0;
    char changed = 0;
    double elapsed = 0;
    int ser_fd;
    char mes[4] = "(..)";

    if (argc > 1)
        device = argv[1];
    else
        device = "/dev/input/js0";

    js = open(device, O_RDONLY|O_NONBLOCK);

    if (js == -1) {
        perror("Could not open joystick");
        return 1;
    }

    printf("joystick open\n");

    ser_fd = init_serial();
    //write(ser_fd,"x",1);

    clock_gettime(CLOCK_REALTIME, &start);
    char buf [100];
    int bytes;
    int count = 0;
    char fw=0;
    char bk=0;
    char lf = 0;
    char ri = 0;
    char cmd[2] = { 0x1B, 0x00 };

    /* This loop will exit if the controller is unplugged. */
    while (1) {
	ioctl(ser_fd, FIONREAD, &bytes);
	if(bytes>0) {
	    int n = read (ser_fd, buf, sizeof buf);
	    printf("%s\n",buf);
	}
	if(bk==1) {
	    cmd[1] = 0x42;
	    write(ser_fd,cmd,2);
	    //printf("bk\n");
	}
	if(fw==1) {
	    cmd[1] = 0x41;
	    write(ser_fd,cmd,2);
	    //printf("fw\n");
	}
	if(lf==1) {
	    cmd[1] = 0x44;
	    write(ser_fd,cmd,2);
	    //printf("lf\n");
	}
	if(ri==1) {
	    cmd[1] = 0x43;
	    write(ser_fd,cmd,2);
	    //printf("ri\n");
	}
	//int i = read_event(js, &event);
	//printf("i este %d\n",i);
	usleep(100000);
	if(read_event(js, &event)>=0) { 
	printf("event: %d\n",event.type);
        switch (event.type) {
            case JS_EVENT_BUTTON:
                printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
		if(event.number == 0 && event.value==1) bk = 1;
		if(event.number == 0 && event.value==0) bk = 0;
		if(event.number == 1 && event.value==1) ri = 1;
		if(event.number == 1 && event.value==0) ri = 0;
		if(event.number == 2 && event.value==1) fw = 1;
		if(event.number == 2 && event.value==0) fw = 0;
		if(event.number == 3 && event.value==1) lf = 1;
		if(event.number == 3 && event.value==0) lf = 0;
                break;
            case JS_EVENT_AXIS:
                if(event.number < 2) {
                    if(event.value<-deadzone||event.value>deadzone) {
                        if(event.number == 0) {
                            jx = event.value;
                        } else {
                            jy = event.value;
                        }
                        changed = 1;
                    } else {
                        if(event.number == 0) {
                            if(jx!=0) { jx = 0; changed = 1; }
                        } else {
                            if(jy!=0) { jy = 0; changed = 1; }
                        }
                    }
                    if(changed) {
                        clock_gettime(CLOCK_REALTIME, &stop);
                        elapsed = (stop.tv_sec - start.tv_sec) + (double)(stop.tv_nsec - start.tv_nsec) / (double)1000000000L;
                        if(elapsed > 0.1 || (jx==0 && jy==0)) {
                            clock_gettime(CLOCK_REALTIME, &start);
                            //printf("changed x=%d y=%d, last %lf \n",jx,jy,elapsed);
                            printf("changed x=%d y=%d, last %lf \n",map(jx,2000,32767,0,127),map(jy,2000,32767,0,127),elapsed);
                            mes[1] = (char)map(jx,2000,32767,0,127);
                            mes[2] = (char)map(jy,2000,32767,0,127);
                            write(ser_fd,mes,4);
                            changed = 0;
                        }
                    }
                }
                break;
            default:
                /* Ignore init events. */
                break;
        }
	}

        fflush(stdout);
    }

    close(js);
    return 0;
}
