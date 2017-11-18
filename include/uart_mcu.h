//$ sudo chmod 666 /dev/ttyUSB0

#ifndef UART_MCU_H
#define UART_MCU_H

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define ID 0x55
#define SERIAL_PORT "/dev/ttyUSB0"

int serial_init(int baudRate, int *);
void read_buf(int *);

#endif
