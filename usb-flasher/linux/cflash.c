/***************************************************************************//**
 * @file
 * @brief TD RF Module flashing utility.
 * @author Telecom Design S.A.
 * @version 1.1.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2016 Telecom Design S.A., http://www.telecom-design.com</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Telecom Design SA has no
 * obligation to support this Software. Telecom Design SA is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Telecom Design SA will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
  ******************************************************************************/

#include <features.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <string.h>

// ****************************************************************************
// TYPES:
// ****************************************************************************

typedef struct _HDR
{
	unsigned int  address:20;
	unsigned int  command:4;
	unsigned int  product:8;
} HDR;

// ****************************************************************************
// DEFINES:
// ****************************************************************************

#define CONSOLE					0

#define SUCCESS					0
#define ERROR_FILE				1
#define ERROR_ACK				2
#define ERROR_SYNC				3
#define ERROR_INT				4
#define ERROR_REBOOT			5
#define ERROR_FLASH				6
#define ERROR_READ				7
#define	ERROR_WRITE				8

#define TIMEOUT					200	//In 1/10 of second

#define FRM_SZ                  60
#define HDR_SZ                  sizeof(HDR)
#define DAT_SZ                  (FRM_SZ - HDR_SZ)

#define WRITE_RAM 				0x4
#define WRITE_FLASH				0x5
#define ERASE_ANDWRITEFLASH		0x6
#define REBOOT					0x7
#define JUMP					0x8

#define ACK                     0x30
#define ACK_UPGRADE				0x31
#define ACK_REBOOT				0x32

/*Code hack*/
#define ACK_3					0x34
#define ACK_4					0x38
#define ACK_5					0x70
#define ACK_6					0xb0

#define ESC                     0x1b

#define VERSION                 "4.1"

// ****************************************************************************
// STATICS:
// ****************************************************************************

/* The following arrays must have equal length and the values must
 * correspond.
 */
static int baudrates[] = {
  0, 9600, 19200, 38400, 57600, 115200, 230400, 460800
};

static speed_t baud_bits[] = {
  0, B9600, B19200, B38400, B57600, B115200, B230400, B460800
};

static unsigned long    totalBytes;

static int              serial_fd;

static unsigned char    product     = 1;

static unsigned int     baudrate    = 115200;

static bool             trace       = false;

static struct termios   orig_termios;

void dump(char *text, unsigned char *s, unsigned char sz);

// ****************************************************************************
// CODE:
// ****************************************************************************
/**
 * keyboard core
 */
static void reset_terminal_mode(void)
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

static void set_conio_terminal_mode(void)
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

static int kbhit(void)
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

static int getch(void)
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}

/**
 * Determine baud rate index
 */
static int indexOfBaud(int baudrate)
{
  int i;

  for (i = 0; i < sizeof(baudrates) / sizeof(baudrates[0]); ++i) {
    if (baudrates[i] == baudrate)
      return i;
  }

  return 0;
}

/**
 *  Opens serial port, set's it to 115200bps 8N1 RTS/CTS mode.
 *
 * @param[in@ dev
 *   device name
 * @return
 *   file descriptor or -1 on error
 */
static int open_serialport(char *dev)
{
    int fd;

    if ((fd = open(dev, O_RDWR | O_NOCTTY)) != -1) {

        int index = indexOfBaud(baudrate);
		struct termios options_rst;
        struct termios options_cpy;
        struct termios options;

        printf("Serial port %s opened at speed %d.\r\n", dev, baudrate);

        fcntl(fd, F_SETFL, 0);

        // Get the parameters
		tcgetattr(fd, &options);

        if (index > 0) {

            // Do like minicom: set 0 in speed options
            cfsetispeed(&options, 0);
            cfsetospeed(&options, 0);
        }

        // Enable the receiver and set local mode and 8N1
		options.c_cflag = (CLOCAL | CREAD | CS8 | HUPCL);

        if (index > 0) {
			cfsetispeed(&options, baud_bits[index]);
			cfsetospeed(&options, baud_bits[index]);
        } else {
            printf("Invalid serial speed %d.\r\n", baudrate);
        }

        // Set raw input
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag  = IGNBRK;

        // Set raw output
		options.c_oflag &= ~(OPOST | OLCUC | ONLRET | ONOCR | OCRNL);

        // Non blocking read
		options.c_cc[VTIME] = 0;
        options.c_cc[VMIN]  = 0;

        tcflush(fd, TCIFLUSH);

        tcsetattr(fd, TCSANOW, &options);
    }

    return fd;
}

void SendReset(int fd)
{
	struct termios options_rst;
	struct termios options;
        int ret;
        int loopIdx;
	unsigned char atz[10] = {0x41, 0x54, 0x5A, 0x0D, 0xA, 0x41, 0x54, 0x5A, 0x0D, 0xA};

	fcntl(fd, F_SETFL, 0);

    // Get the parameters
	tcgetattr(fd, &options); 

	// Set the reset options for the port...
	options_rst = options;
	cfsetispeed(&options_rst, B9600);
	cfsetospeed(&options_rst, B9600);
	tcsetattr(fd, TCSANOW, &options_rst);
	write(fd, atz, 10);
	usleep(100000);
    while ((ret = read(serial_fd, atz, sizeof(atz))) == 0) {
		;
    }
    if (trace) {
        printf("SendReset: ");
        for (loopIdx = 0; loopIdx < 10; loopIdx++) {
            printf ("%02x ", atz[loopIdx]);
		}
        for (loopIdx = 0; loopIdx < 10; loopIdx++) {
            if (atz[loopIdx] >= ' ') {
                printf ("%c", atz[loopIdx]);
            } else {
                printf (".");
			}
        }
        printf("\r\n");
    }
	tcsetattr(fd, TCSANOW, &options);
}

void dump(char *text, unsigned char *s, unsigned char sz)
{
    static unsigned char dump[3 * 16];
    unsigned char i, j, k;
    static char hex[10];

/*
    for (i = 0; i < sz; i++) {
        if (s[i] >= ' ') {
            printf("%c", s[i]);
        } else {
            printf(".");
		}
    }
    printf("\r\n");
*/

    printf("%s", text);
    for (i = 0, k = 0; i < sz; i++) {
        sprintf(hex, "%02x ", s[i]);
        for (j = 0; hex[j]; j++) {
            dump[k++] = hex[j];
        }
        if (k == 16*3) {
            dump[k] = '\0';
            printf("%s\r\n", dump);
            k = 0;
        }
    }
    if (k > 0) {
        dump[k] = '\0';
        printf("%s\r\n", dump);
    }
}

static unsigned char waitack(unsigned short delay)
{
    struct timeval timeout;
    unsigned char tmp[80];
    fd_set rfds;
    int ret;

    FD_ZERO(&rfds);
    FD_SET(serial_fd, &rfds);

    // Set the requested select timeout
    timeout.tv_sec  = 0;//delay / 1000;
    timeout.tv_usec = delay * 1000;

    // Wait for characters
	if (select(serial_fd + 1, &rfds, NULL, NULL, &timeout) > 0) {
        if (FD_ISSET(serial_fd, &rfds)) {
            if ((ret = read(serial_fd, tmp, sizeof(tmp))) > 0) {
                if (trace) {
                    dump("ACK: ", tmp, ret);
                }
                return tmp[0];
            }
        }
    }

    // Nothing received
	return 0;
}

static bool upgrade(char *filename)
{
    unsigned int i, ret, sz, percent, old = -1;
    unsigned char binline[FRM_SZ], flood;
    HDR *hdr = (HDR *)binline;
	unsigned char c;
	char prompt[20];
	int count = 0, cpt = 0;
	FILE *fp;

	if ((fp = fopen(filename, "rb")) == NULL) {
		printf("Failure : couldn't open file %s\r\n", filename);
		fprintf(stderr, "ErrorCode : %d\r\n", ERROR_FILE);
		return false;
	}
    fseek(fp, 0L, SEEK_END);

    // Get File size (progress bar)
	totalBytes = ftell(fp);
    fseek(fp, 0L, SEEK_SET);
    printf("Flashing %ld bytes on products id %#-2.2x\r\n", totalBytes, product);
	printf("Synchronising, any key to abort...\r\n");
	SendReset(serial_fd);
	printf("ATZ reset sent...\r\n");
	flood = 'X';
	ret = write(serial_fd, &flood, 1);

    // loop sending upgrade pattern
	while (!kbhit() && ((c = waitack(100)) != ACK_UPGRADE) && cpt < TIMEOUT) {
		ret = write(serial_fd, &flood, 1);
		cpt++;
	}
        printf ("cpt: %d - c: %02x / %c\r\n", cpt, c, c);
	if (c == ACK_UPGRADE) {
		printf("Dans upgrade - product: %02x...\r\n", product);
		memset(binline, 0, FRM_SZ);
		binline[DAT_SZ] = 0x2B;	//'+';
		hdr->product = product;

#if CONSOLE
		printf("OK,\r\n'U' to upgrade, any other key to abort...\r\n");

		while (!kbhit()) {
#endif
			ret = write(serial_fd, binline, FRM_SZ);
			if ((c = waitack(10000)) != ACK) {
				printf("Did not get local ACK (%02x)\r\n", c);
				fprintf(stderr, "ErrorCode : %d\r\n", ERROR_ACK);
				fclose(fp);
				return false;
			}
#if CONSOLE
			sleep(2);
		}
#endif
	} else if (cpt >= TIMEOUT) {
		printf("Synchronisation timeout\r\n");
		fprintf(stderr, "ErrorCode : %d\r\n", ERROR_SYNC);
		return false;
	} else {
		getchar();
		printf("Synchronisation interrupted\r\n");
		fprintf(stderr, "ErrorCode : %d\r\n", ERROR_SYNC);
		return false;
	}

#if CONSOLE
    if (getch() != 'U') {
        printf("Interrupted...\r\n");
		fprintf(stderr, "ErrorCode : %d\r\n", ERROR_INT);
        return false;
    }
#endif

    printf("Upgrading, hit ESC to abort...\r\n");
    hdr->command = WRITE_FLASH;
    while (true) {
        if (kbhit() && (getch() == ESC)) {

			// keyboard interrupt
			printf("\r\nFlashing interrupted\r\n");
		    fprintf(stderr, "ErrorCode : %d\r\n", ERROR_FLASH);
            return false;
        }
        if ((sz = fread(&binline[HDR_SZ], 1, DAT_SZ, fp)) < 0) {

            // file read error
            printf("\r\nRead error (%s)\r\n", strerror(errno));
            fprintf(stderr, "ErrorCode : %d\r\n", ERROR_READ);
            return false;
        }
        if (sz == 0) {

	        // end of file
			break;
        }
        if (trace) {
            sprintf(prompt, "FRM %d\r\n", ++count);
            dump(prompt, binline, FRM_SZ);
        }
	    if ((ret = write(serial_fd, binline, FRM_SZ)) != FRM_SZ) {

            // send data packet
            printf("\r\nWrite error (%d/%d)\r\n", ret, FRM_SZ);
	    	fprintf(stderr, "ErrorCode : %d\r\n", ERROR_WRITE);
            return false;
	    }
	    if (waitack(6000) != ACK ) {
 
            // wait for local acknowledge
			printf("\r\nWrite ack error\r\n");
			fprintf(stderr, "ErrorCode : %d\r\n", ERROR_WRITE);
			return false;
		}
	    hdr->address += DAT_SZ;
	    // update completion indicator
		if ((percent = (hdr->address * 100) / totalBytes) != old) {
	        printf("\r%d%%", percent);
	        old = percent;
	        fflush(stdout);
	    }

	    // important: wait 50ms
		usleep(50000);
    }
    printf("\r\nUpgrade OK\r\n");

    // reboot product
	hdr->command = REBOOT;
    memset(&binline[HDR_SZ], 'X', DAT_SZ);
    if (trace) {
        dump("", binline, FRM_SZ);
    }
    write(serial_fd, binline, FRM_SZ);
    if (waitack(6000) != ACK_REBOOT) {
		printf("Did not get local ACK for reboot\r\n");
		fprintf(stderr, "ErrorCode : %d\r\n", ERROR_REBOOT);
		return false;
	}
    fprintf(stderr, "ErrorCode : %d\r\n", SUCCESS);
    return true;
}

// shows how to use this program
static void usage(void)
{
	printf("Usage: [options] <file>\n");
	printf("options:\n");
	printf("  -d <device>         : Serial port device to connect to [/dev/ttyUSB0]\n");
	printf("  -b <baudrate>       : Serial port speed (0,9600,19200, ...)\n");
	printf("  -p <product>        : Product type (1..255)\n");
	printf("  -t                  : Activate trace mode\n");
	printf("  -h                  : Show this help message\n");
}

int main(int argc, char *argv[])
{
    char *dev = "/dev/ttyUSB0";
    char *filename;
    int opt;
    bool ok;

    printf("cflash version %s\n", VERSION);
    printf("Copyright (c) 2012-2013 Telecom Design S.A.\n");
    while ((opt = getopt(argc, argv, "d:p:b:th?")) > 0) {
        switch (opt) {
            case 'p':
				product = strtoul(optarg, 0, 0);
                break;
            case 'd':
                dev = optarg;
                break;
            case 'b':
                baudrate = atoi(optarg);
                break;
            case 't':
                trace = true;
                break;
            case 'h':
            case '?':
                usage();
                exit(1);
                break;
        }
    }
    if (((filename = argv[optind]) == NULL) || (access(filename, 0) != 0)) {
        printf("Can't find file %s.\r\n", filename);
        exit(1);
    }

    // Open the serial port
	if ((serial_fd = open_serialport(dev)) < 0) {
        printf("Can't open %s. %s (%d).\r\n", dev, strerror(errno), errno);
        exit(1);
    }
    set_conio_terminal_mode();
    ok = upgrade(filename);
    reset_terminal_mode();
    close(serial_fd);
    exit (ok ? 0 : 1);
}

// ****************************************************************************
