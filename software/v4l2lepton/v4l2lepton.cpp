#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>                /* low-level i/o */
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <semaphore.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <stdint.h>

#include "Palettes.h"
#include "SPI.h"
#include "Lepton_I2C.h"

#define PACKET_SIZE 164
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
#define PACKETS_PER_FRAME 60
#define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16*PACKETS_PER_FRAME)
#define FPS 2;

static char const *v4l2dev = "/dev/video1";
static char *spidev = NULL;
static int v4l2sink = -1;
static int sf = 1;
static int width = sf*80;
static int height = sf*60;
static char *vidsendbuf = NULL;
static int vidsendsiz = 0;

static int resets = 0;
static uint8_t result[PACKET_SIZE*PACKETS_PER_FRAME];
static uint16_t *frameBuffer;

static void init_device() {
    SpiOpenPort(spidev);
}


#define COLORBAR_HEIGHT 5
#define HISTOGRAM_HEIGHT 30
#define TEXT_HEIGHT 40

static void draw_char(int start_height, int R, int C, char c)
{
	
	int	fontdata[95][7] = {
	{  0,   0,   0,   0,   0,   0,   0, }, // { 0} ' '
    {  4,   4,   4,   4,   0,   0,   4, }, // { 1} '!'
    { 10,  10,  10,   0,   0,   0,   0, }, // { 2} '"'
    { 10,  10,  31,  10,  31,  10,  10, }, // { 3} '#'
    {  4,  15,  20,  14,   5,  30,   4, }, // { 4} '$'
    { 24,  25,   2,   4,   8,  19,   3, }, // { 5} '%'
    { 12,  18,  20,   8,  21,  18,  13, }, // { 6} '&'
    { 12,   4,   8,   0,   0,   0,   0, }, // { 7} '''
    {  2,   4,   8,   8,   8,   4,   2, }, // { 8} '('
    {  8,   4,   2,   2,   2,   4,   8, }, // { 9} ')'
    {  0,   4,  21,  14,  21,   4,   0, }, // {10} '*'
    {  0,   4,   4,  31,   4,   4,   0, }, // {11} '+'
    {  0,   0,   0,   0,  12,   4,   8, }, // {12} ','
    {  0,   0,   0,  31,   0,   0,   0, }, // {13} '-'
    {  0,   0,   0,   0,   0,  12,  12, }, // {14} '.'
    {  0,   1,   2,   4,   8,  16,   0, }, // {15} '/'
    { 14,  17,  19,  21,  25,  17,  14, }, // {16} '0'
    {  4,  12,   4,   4,   4,   4,  14, }, // {17} '1'
    { 14,  17,   1,   2,   4,   8,  31, }, // {18} '2'
    { 31,   2,   4,   2,   1,  17,  14, }, // {19} '3'
    {  2,   6,  10,  18,  31,   2,   2, }, // {20} '4'
    { 31,  16,  30,   1,   1,  17,  14, }, // {21} '5'
    {  6,   8,  16,  30,  17,  17,  14, }, // {22} '6'
    { 31,  17,   1,   2,   4,   4,   4, }, // {23} '7'
    { 14,  17,  17,  14,  17,  17,  14, }, // {24} '8'
    { 14,  17,  17,  15,   1,   2,  12, }, // {25} '9'
    {  0,  12,  12,   0,  12,  12,   0, }, // {26} ':'
    {  0,  12,  12,   0,  12,   4,   8, }, // {27} ';'
    {  2,   4,   8,  16,   8,   4,   2, }, // {28} '<'
    {  0,   0,  31,   0,  31,   0,   0, }, // {29} '='
    {  8,   4,   2,   1,   2,   4,   8, }, // {30} '>'
    { 14,  17,   1,   2,   4,   0,   4, }, // {31} '?'
    { 14,  17,   1,  13,  21,  21,  14, }, // {32} '@'
    { 14,  17,  17,  17,  31,  17,  17, }, // {33} 'A'
    { 30,  17,  17,  30,  17,  17,  30, }, // {34} 'B'
    { 14,  17,  16,  16,  16,  17,  14, }, // {35} 'C'
    { 28,  18,  17,  17,  17,  18,  28, }, // {36} 'D'
    { 31,  16,  16,  30,  16,  16,  31, }, // {37} 'E'
    { 31,  16,  16,  30,  16,  16,  16, }, // {38} 'F'
    { 14,  17,  16,  23,  17,  17,  15, }, // {39} 'G'
    { 17,  17,  17,  31,  17,  17,  17, }, // {40} 'H'
    { 14,   4,   4,   4,   4,   4,  14, }, // {41} 'I'
    {  7,   2,   2,   2,   2,  18,  12, }, // {42} 'J'
    { 17,  18,  20,  24,  20,  18,  17, }, // {43} 'K'
    { 16,  16,  16,  16,  16,  16,  31, }, // {44} 'L'
    { 17,  27,  21,  21,  17,  17,  17, }, // {45} 'M'
    { 17,  17,  25,  21,  19,  17,  17, }, // {46} 'N'
    { 14,  17,  17,  17,  17,  17,  14, }, // {47} 'O'
    { 30,  17,  17,  30,  16,  16,  16, }, // {48} 'P'
    { 14,  17,  17,  17,  21,  18,  13, }, // {49} 'Q'
    { 30,  17,  17,  30,  20,  18,  17, }, // {50} 'R'
    { 15,  16,  16,  14,   1,   1,  30, }, // {51} 'S'
    { 31,   4,   4,   4,   4,   4,   4, }, // {52} 'T'
    { 17,  17,  17,  17,  17,  17,  14, }, // {53} 'U'
    { 17,  17,  17,  17,  17,  10,   4, }, // {54} 'V'
    { 17,  17,  17,  21,  21,  21,  10, }, // {55} 'W'
    { 17,  17,  10,   4,  10,  17,  17, }, // {56} 'X'
    { 17,  17,  17,  10,   4,   4,   4, }, // {57} 'Y'
    { 31,   1,   2,   4,   8,  16,  31, }, // {58} 'Z'
    { 28,  16,  16,  16,  16,  16,  28, }, // {59} '['
    { 17,  10,  31,   4,  31,   4,   4, }, // {60} '\'
    { 14,   2,   2,   2,   2,   2,  14, }, // {61} '}'
    {  4,  10,  17,   0,   0,   0,   0, }, // {62} '^'
    {  0,   0,   0,   0,   0,   0,  31, }, // {63} '_'
    {  8,   4,   2,   0,   0,   0,   0, }, // {64} '`'
    {  0,   0,  14,   1,  15,  17,  15, }, // {65} 'a'
    { 16,  16,  22,  25,  17,  17,  30, }, // {66} 'b'
    {  0,   0,  14,  16,  16,  17,  14, }, // {67} 'c'
    {  1,   1,  13,  19,  17,  17,  15, }, // {68} 'd'
    {  0,   0,  14,  17,  31,  16,  14, }, // {69} 'e'
    {  6,   9,   8,  28,   8,   8,   8, }, // {70} 'f'
    {  0,  15,  17,  17,  15,   1,  14, }, // {71} 'g'
    { 16,  16,  22,  25,  17,  17,  17, }, // {72} 'h'
    {  4,   0,  12,   4,   4,   4,  14, }, // {73} 'i'
    {  2,   0,   6,   2,   2,  18,  12, }, // {74} 'j'
    { 16,  16,  18,  20,  24,  20,  18, }, // {75} 'k'
    { 12,   4,   4,   4,   4,   4,  14, }, // {76} 'l'
    {  0,   0,  26,  21,  21,  17,  17, }, // {77} 'm'
    {  0,   0,  22,  25,  17,  17,  17, }, // {78} 'n'
    {  0,   0,  14,  17,  17,  17,  14, }, // {79} 'o'
    {  0,   0,  30,  17,  30,  16,  16, }, // {80} 'p'
    {  0,   0,  13,  19,  15,   1,   1, }, // {81} 'q'
    {  0,   0,  22,  25,  16,  16,  16, }, // {82} 'r'
    {  0,   0,  14,  16,  14,   1,  30, }, // {83} 's'
    {  8,   8,  28,   8,   8,   9,   6, }, // {84} 't'
    {  0,   0,  17,  17,  17,  19,  13, }, // {85} 'u'
    {  0,   0,  17,  17,  17,  10,   4, }, // {86} 'v'
    {  0,   0,  17,  21,  21,  21,  10, }, // {87} 'w'
    {  0,   0,  17,  10,   4,  10,  17, }, // {88} 'x'
    {  0,   0,  17,  17,  15,   1,  14, }, // {89} 'y'
    {  0,   0,  31,   2,   4,   8,  31, }, // {90} 'z'
    {  2,   4,   4,   8,   4,   4,   2, }, // {91} '{'
    {  4,   4,   4,   4,   4,   4,   4, }, // {92} '|'
    {  8,   4,   4,   2,   4,   4,   8, }, // {93} '}'
    {  0,   4,   2,  31,   2,   4,   0, }, // {94} '~'
	};

	int total_height = start_height + TEXT_HEIGHT;
	int idx = c - ' ';
	if(idx < 0 || idx > 95)
	{
		idx = 31;
	}
	int * font = fontdata[idx];
	for(int j=0; j<7; j++)
	{
		int fontline = font[j];
		for (int i=0; i<5; i++)
		{
			int pix = 0;
			if((1<<(4-i)) & fontline)
				pix = 255;

			int row = start_height + 1 + (R-1) * 10 + j;
			int column = 5 + C * 7 + i;

			if(row < total_height && column < width)
			{
				int idx = row * width * 3 + column * 3;

				vidsendbuf[idx + 0] = pix; 
				vidsendbuf[idx + 1] = pix;
				vidsendbuf[idx + 2] = pix;
			}
		}
	}
	



}
static void draw_string(int start_height, int R, int C, char * c)
{
	while(*c != 0)
	{
		draw_char(start_height, R, C, *c);
		C++;
		c++;
	}
}

static void grab_frame() {

    resets = 0;
    for (int j = 0; j < PACKETS_PER_FRAME; j++) {
        read(spi_cs_fd, result + sizeof(uint8_t) * PACKET_SIZE * j, sizeof(uint8_t) * PACKET_SIZE);
        int packetNumber = result[j * PACKET_SIZE + 1];
        if (packetNumber != j) {
            j = -1;
            resets += 1;
            usleep(1000);
            if (resets == 750) {
                SpiClosePort();
                usleep(750000);
                SpiOpenPort(spidev);
            }
        }
    }
    if (resets >= 30) {
        fprintf( stderr, "done reading, resets: \n" );
    }

    frameBuffer = (uint16_t *)result;
    int row, column;
    int32_t value;
    uint16_t minValue = 65535;
    uint16_t maxValue = 0;

    for (int i = 0; i < FRAME_SIZE_UINT16; i++) {
        if (i % PACKET_SIZE_UINT16 < 2) {
            continue;
        }

        int temp = result[i * 2];
        result[i * 2] = result[i * 2 + 1];
        result[i * 2 + 1] = temp;

        value = frameBuffer[i];
        if (value > maxValue) {
            maxValue = value;
        }
        if (value < minValue) {
            minValue = value;
        }
        column = i % PACKET_SIZE_UINT16 - 2;
        row = i / PACKET_SIZE_UINT16;
    }

	char buf[20];
	sprintf(buf, "%4d %4d", minValue, maxValue);


	//Fixed-temperature scaling range
	minValue = 8100;
	maxValue = 8500;

	int hminValue = 8300;
	int hmaxValue = 8500;


	if(hminValue < minValue) hminValue = minValue;
	if(hmaxValue > maxValue) hmaxValue = maxValue;

	float avgBody = 0;
	int nBody = 0;
	float avgBg = 0; //Background
	int nBg = 0;

    float hist[width];
	for (int i = 0; i < width; i++)
	{
		hist[i] = 0;
	}


	float diff = (maxValue - minValue);	
    float scale = 255 / diff;
    const int *colormap = colormap_rainbow;
    //const int *colormap = colormap_ironblack;
    for (int i = 0; i < FRAME_SIZE_UINT16; i++) {
        if (i % PACKET_SIZE_UINT16 < 2) {
            continue;
        }
        value = (frameBuffer[i] - minValue) * scale;
		if(value < 0) value = 0;
		if(value > 255) value = 255;

        column = (i % PACKET_SIZE_UINT16) - 2;
        row = i / PACKET_SIZE_UINT16;

		int val = frameBuffer[i];
		if(val > 8300 && val < 8600)
		{
			avgBody += val;
			nBody++;
		}
		else
		{
			avgBg += val;
			nBg++;
		}


		int bin = (int)((frameBuffer[i] - hminValue) * width / (hmaxValue - hminValue));
		if(bin >= 0 && bin < width)
		{
			hist[bin]++;
		}

        // Set video buffer pixel to scaled colormap value
        int idx = row * width * 3 + column * 3; 
        vidsendbuf[idx + 0] = colormap[3 * value];
        vidsendbuf[idx + 1] = colormap[3 * value + 1];
        vidsendbuf[idx + 2] = colormap[3 * value + 2];

    }


	int startHeight = height;

	int draw_colorbar = 1;
	if(draw_colorbar)
	{
	for (int i = 0; i < width; i++)
	{
		int value = (int)((i / (float)width) * 255);
		for (int j = 0; j < COLORBAR_HEIGHT; j++)
		{
        	int idx = (j+startHeight) * width * 3 + i * 3;
	        vidsendbuf[idx + 0] = colormap[3 * value];
	        vidsendbuf[idx + 1] = colormap[3 * value + 1];
    	    vidsendbuf[idx + 2] = colormap[3 * value + 2];
		
		}
	}
	startHeight += COLORBAR_HEIGHT;
	}

	float maxBinCount = 0;
	for (int i = 0; i < width; i++)
	{
		if(hist[i] > maxBinCount)
		{
			maxBinCount = hist[i];
		}
	}

	int draw_histogram = 1;
	if(draw_histogram)
	{
	for (int i = 0; i < width; i++)
	{
		int edge = hist[i] / maxBinCount * HISTOGRAM_HEIGHT;
		for (int bh = 0; bh < HISTOGRAM_HEIGHT ; bh++)
		{
			row = startHeight + HISTOGRAM_HEIGHT  - bh - 1;
			column = i;

			int idx = row * width * 3 + column * 3;

			int pix = 255;
			if(bh >= edge)
				pix = 0;
#if COLORMAP_WHITE
			vidsendbuf[idx + 0] = pix; 
			vidsendbuf[idx + 1] = pix;
			vidsendbuf[idx + 2] = pix;
#else
			if(pix == 0)
			{
				vidsendbuf[idx + 0] = pix; 
				vidsendbuf[idx + 1] = pix;
				vidsendbuf[idx + 2] = pix;
			}
			else
			{
				float fpix = i / (float)width;
				float hb = (hminValue-minValue) / (float)(maxValue - minValue);
				float hs = (hmaxValue - hminValue) / (float)(maxValue - minValue);
				fpix = fpix * hs + hb;

				pix = (int)(fpix * 255);
	        	vidsendbuf[idx + 0] = colormap[3 * pix];
	        	vidsendbuf[idx + 1] = colormap[3 * pix + 1];
    	    	vidsendbuf[idx + 2] = colormap[3 * pix + 2];
			}
#endif

		}
	}	
	startHeight += HISTOGRAM_HEIGHT;
	}

	if(nBody > 0)
		avgBody /= nBody;
	if(nBg > 0)
		avgBg /= nBg;

	draw_string(startHeight, 1, 0, buf);

	sprintf(buf, "%4d %d", nBg, (int) avgBg);
	draw_string(startHeight, 2, 0, buf);
	sprintf(buf, "%4d %d", nBody, (int)avgBody);
	draw_string(startHeight, 3, 0, buf);

    /*
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    memset( vidsendbuf, 0, 3);
    memcpy( vidsendbuf+3, vidsendbuf, vidsendsiz-3 );
    */
}

static void stop_device() {
    SpiClosePort();
}

static void open_vpipe()
{
    v4l2sink = open(v4l2dev, O_WRONLY);
    if (v4l2sink < 0) {
        fprintf(stderr, "Failed to open v4l2sink device. (%s)\n", strerror(errno));
        exit(-2);
    }
    // setup video for proper format
    struct v4l2_format v;
    int t;
    v.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    t = ioctl(v4l2sink, VIDIOC_G_FMT, &v);
    if( t < 0 )
        exit(t);
    v.fmt.pix.width = width;
    v.fmt.pix.height = height + COLORBAR_HEIGHT + HISTOGRAM_HEIGHT + TEXT_HEIGHT;
    v.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

    vidsendsiz = width * (height + COLORBAR_HEIGHT + HISTOGRAM_HEIGHT + TEXT_HEIGHT)  * 3;
    v.fmt.pix.sizeimage = vidsendsiz;
    t = ioctl(v4l2sink, VIDIOC_S_FMT, &v);
    if( t < 0 )
        exit(t);
    vidsendbuf = (char*)malloc( vidsendsiz );
}

static pthread_t sender;
static sem_t lock1,lock2;
static void *sendvid(void *v)
{
    (void)v;
    for (;;) {
        sem_wait(&lock1);
        if (vidsendsiz != write(v4l2sink, vidsendbuf, vidsendsiz))
            exit(-1);
        sem_post(&lock2);
    }
}

void usage(char *exec)
{
    printf("Usage: %s [options]\n"
           "Options:\n"
           "  -d | --device name       Use name as spidev device "
               "(/dev/spidev0.1 by default)\n"
           "  -h | --help              Print this message\n"
           "  -v | --video name        Use name as v4l2loopback device "
               "(%s by default)\n"
           "", exec, v4l2dev);
}

static const char short_options [] = "d:hv:";

static const struct option long_options [] = {
    { "device",  required_argument, NULL, 'd' },
    { "help",    no_argument,       NULL, 'h' },
    { "video",   required_argument, NULL, 'v' },
    { 0, 0, 0, 0 }
};

int main(int argc, char **argv)
{
    struct timespec ts;

    // processing command line parameters
    for (;;) {
        int index;
        int c;

        c = getopt_long(argc, argv,
                        short_options, long_options,
                        &index);

        if (-1 == c)
            break;

        switch (c) {
            case 0:
                break;

            case 'd':
                spidev = optarg;
                break;

            case 'h':
                usage(argv[0]);
                exit(EXIT_SUCCESS);

            case 'v':
                v4l2dev = optarg;
                break;

            default:
                usage(argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    open_vpipe();
    // open and lock response
    if (sem_init(&lock2, 0, 1) == -1)
        exit(-1);
    sem_wait(&lock2);

    if (sem_init(&lock1, 0, 1) == -1)
        exit(-1);
    pthread_create(&sender, NULL, sendvid, NULL);

    for (;;) {
        // wait until a frame can be written
        fprintf( stderr, "Waiting for sink\n" );
        sem_wait(&lock2);
        // setup source
printf("trySPI");
        init_device(); // open and setup SPI
printf("SPI");
        for (;;) {
            grab_frame();
            // push it out
            sem_post(&lock1);
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 2;
            // wait for it to get written (or is blocking)
            if (sem_timedwait(&lock2, &ts))
                break;
        }
        stop_device(); // close SPI
    }
    close(v4l2sink);
    return 0;
}
