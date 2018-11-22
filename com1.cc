
#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/stat.h>
#include <nmea.h>
#include <nmea/gpgll.h>
#include <nmea/gpgga.h>
#include <nmea/gprmc.h>

//#include <fcnt1.h>
//#define NMEA_END_CHAR_1		'\r'
//#define NMEA_END_CHAR_2		'\n'


int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}



int main()
{
 
   int gps_fd;



//    sigset_t block_mask;
    char *portname = "/dev/ttyS14";
    int read_bytes; 
    int wlen;
   // char start, *end;
    gps_fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (gps_fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(gps_fd, B9600);
    set_mincount(gps_fd, 10);                /* set to pure timed read */

    /* simple output */
    wlen = write(gps_fd, "Hello!\n", 7);
    if (wlen != 7) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(gps_fd);    /* delay for output */

	char buffer[4096];
//	int gps_fd = 0; // stdin
//	int read_bytes = 0;
	int total_bytes = 0;
	char *start, *end;
	sigset_t block_mask;

	while (1) {
		char buf[255];
		nmea_s *data;

		read_bytes = read(gps_fd, buffer + total_bytes, 20);
		if (-1 == read_bytes) {
			perror("read stdin");
			exit(EXIT_FAILURE);
		}
		if (0 == read_bytes) {
			break;
 		}

		total_bytes += read_bytes;

		/* find start (a dollar $ign) */
		start = (char *) memchr(buffer, '$', total_bytes);
		if (NULL == start) {
			total_bytes = 0;
			continue;
		}

		/* find end of line */
		end = (char*) memchr(start, NMEA_END_CHAR_1, total_bytes - (start - buffer));
		if (NULL == end || NMEA_END_CHAR_2 != *(++end)) {
			continue;
		}

		/* handle data */
		data = nmea_parse(start, end - start + 1, 0);
		if (NULL != data) {
			if (0 < data->errors) {
				printf("{ type: 'GPWRN', data: { message:'The following sentence contains parse errors!' } }\n");
			}

			if (NMEA_GPGGA == data->type) {
				nmea_gpgga_s *gpgga = (nmea_gpgga_s *) data;
				printf("{ type: 'GPGGA', data: { satellites: %d, altitude: '%d%c' } }\n", gpgga->n_satellites, gpgga->altitude, gpgga->altitude_unit);
			}

			if (NMEA_GPGLL == data->type) {
				nmea_gpgll_s *pos = (nmea_gpgll_s *) data;
				strftime(buf, sizeof(buf), "%H:%M:%S", &pos->time);
				printf("{ type: 'GPGLL', data: { long: '%c%d:%f', lat: '%c%d:%f', time: '%s' } }\n",\
					(char) pos->longitude.cardinal, pos->longitude.degrees, pos->longitude.minutes,\
					(char) pos->latitude.cardinal, pos->latitude.degrees, pos->latitude.minutes,\
					buf);
			}

			if (NMEA_GPRMC == data->type) {
				nmea_gprmc_s *pos = (nmea_gprmc_s *) data;
				strftime(buf, sizeof(buf), "%H:%M:%S", &pos->time);
				printf("{ type: 'GPRMC', data: { long: '%c%d:%f', lat: '%c%d:%f', time: '%s' } }\n",\
					(char) pos->longitude.cardinal, pos->longitude.degrees, pos->longitude.minutes,\
					(char) pos->latitude.cardinal, pos->latitude.degrees, pos->latitude.minutes,\
					buf);
			}

			nmea_free(data);
		}

		/* buffer empty? */
		if (end == buffer + total_bytes) {
			total_bytes = 0;
			continue;
		}

		/* copy rest of buffer to beginning */
		if (buffer != memmove(buffer, end, total_bytes - (end - buffer))) {
			total_bytes = 0;
			continue;
		}

		total_bytes -= end - buffer;
	}

	return 0;
}


/*
    /* simple noncanonical input 
	char *start, *end;
	int total_bytes;
	int i = 0; 
	char buffer [4096];
  do { 
        int rdlen;
	char buf[4096];
	read_bytes = read(fd, buf, sizeof(buf) - 1);

	if (0 == read_bytes)
	{
		printf("0 bytes");
//		sig_quit(SIGINT);
	}


	total_bytes += read_bytes;
//	sigprocmask(SIG_BLOCK, &block_mask, NULL);
		
	start = (char *) memchr(buf, '$', total_bytes);
		if (NULL == start) {
			total_bytes = 0;
		
			continue;
		}

        	if (i>100){
			 close(fd);
			break;
		}

	end = (char *) memchr(start, NMEA_END_CHAR_1, total_bytes - (start - buf));
		if (NULL == end || NMEA_END_CHAR_2 != *(++end)) {
			continue;
//			printf("end \n");
		}

	nmea_parse (start, end - start+1,0);
 	printf("\n  GETTING  FOR NOW \n\n \n %s", end);
//	printf(end);

//	if (rdlen > 0) {
//	printf("buf not NULL");
//		start = (char*) memchr (buf, '$', rdlen);
//		if (NULL==start)
//		{
//			total_bytes=0;
//			continue;
//		}
//		printf("start");
//		end = (char*) memchr(buf, NMEA_END_CHAR_1, rdlen - (start-buf));
//		if (NULL== end || NMEA_END_CHAR_2 != *(++end)){
//		//printf ("here2");
//		continue;
//		} 
//		printf("end \n");
//            buf[rdlen] = 0;
     //       printf("%s",buf);
	    i++;
//        } else if (rdlen < 0) {
//            printf("Error from read: %d: %s\n", rdlen, strerror(errno));
//        }
        /* repeat read to get full message 
    } while (1);
}
*/

