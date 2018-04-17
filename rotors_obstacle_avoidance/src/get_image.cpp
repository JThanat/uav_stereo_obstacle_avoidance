// g++ -fpermissive camera.c get_image.cpp -o get_image -pthread -std=c++11
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <malloc.h>
#include <stdint.h>
#include <signal.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/videodev2.h>
#include <fcntl.h>

#include <getopt.h>
#include <syslog.h>
#include <math.h>

namespace getimage
{

	void printError(char* errMsg){
		fprintf(stderr,"%s\n", errMsg);
		fprintf(stderr, "exiting\n");
		fflush(stderr);
		exit(0);
	}

	void flushBuffer()
	{
		uint8_t mode = 0;
		uint8_t bits = 8;
		uint32_t speed = 25000000 / 4;
		uint16_t delay = 0;
		int ret = 0;
		int fd;
		int exposure = 2; // milisec
		char *gpioName = "/sys/class/gpio/gpio237/value";
		char *spidevName = "/dev/spidev0.0";

		int gpio_fd = -1;
		//pull gpio high in case it was previously low
		gpio_fd = fopen(gpioName, "w");
		if (gpio_fd == NULL)
		{
			printError("Can't open GPIO, you may not have enough privilege");
		}

		fwrite("1", sizeof(char), 1, gpio_fd);
		fflush(gpio_fd);
		fclose(gpio_fd);
		usleep(20000); //wait for 20 milliseconds

		fd = open(spidevName, O_RDWR);
		if (fd < 0)
			printError("can't open device");

		/*
		* spi mode
		*/
		ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
		if (ret == -1)
			printError("can't set spi mode");

		ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
		if (ret == -1)
			printError("can't get spi mode");

		/*
		* bits per word
		*/
		ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
		if (ret == -1)
			printError("can't set bits per word");

		ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
		if (ret == -1)
			printError("can't get bits per word");

		/*
		* max speed hz
		*/
		ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
		if (ret == -1)
			printError("can't set max speed hz");

		ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
		if (ret == -1)
			printError("can't get max speed hz");

		printf("spi mode: %d\n", mode);
		printf("bits per word: %d\n", bits);
		printf("max speed: %d Hz (%d KHz)\n", speed, speed / 1000);

		//buffers
		uint8_t *tx = (uint8_t *)malloc(4 * sizeof(uint8_t));
		uint8_t expoHigh = exposure >> 8;
		uint8_t expoLow = exposure % 256;
		uint8_t *rx = (uint8_t *)malloc(4 * sizeof(uint8_t));
		struct spi_ioc_transfer tr{};
		while(true)
		{
			gpio_fd = -1;

			gpio_fd = fopen(gpioName, "w");
			if (gpio_fd == NULL)
			{
				printError("Can't open GPIO, you may not have enough privilege");
			}

			fwrite("0", sizeof(char), 1, gpio_fd);
			fflush(gpio_fd);
			fclose(gpio_fd);

			usleep(20000); //wait for 20 milliseconds

			tx[0] = 0x90;	 //command, 0x90 is set trigger.
			tx[1] = 0x00;	 //trigger offset
			tx[2] = expoHigh; // tx[2][3:0] is exposure[11:8].
			tx[3] = expoLow;  // tx[3][7:0] is exposure[7:0].
			
			tr.tx_buf = (unsigned long)tx;
			tr.rx_buf = (unsigned long)rx;
			tr.len = 4;
			tr.delay_usecs = delay;
			tr.speed_hz = speed;
			tr.bits_per_word = bits;
			ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
			usleep(20000); //wait for 20 milliseconds

			gpio_fd = -1;

			gpio_fd = fopen(gpioName, "w");
			if (gpio_fd == NULL)
			{
				printError("Can't open GPIO, you may not have enough privilege");
			}

			fwrite("1", sizeof(char), 1, gpio_fd);
			fflush(gpio_fd);
			fclose(gpio_fd);

			openlog("remote", LOG_PID, LOG_USER);
			syslog(LOG_INFO, "SPI sent");
			closelog();

			usleep(250000);
		}
		free(tx);
		free(rx);

		close(fd);
	}
}
