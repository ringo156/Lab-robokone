#include "uart_mcu.h"


int serial_init(int baudRate, int *fd){    
    unsigned char msg[] = "serial port open...\n";
    struct termios tio;
    *fd = open(SERIAL_PORT, O_RDWR);     // デバイスをオープンする
    if (*fd < 0) {
        printf("open error\n");
        return -1;
    }
    tio.c_cflag += CREAD;               // 受信有効
    tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
    tio.c_cflag += CS8;                 // データビット:8bit
    tio.c_cflag += 0;                   // ストップビット:1bit
    tio.c_cflag += 0;                   // パリティ:None

    cfsetispeed( &tio, baudRate );
    cfsetospeed( &tio, baudRate );

    cfmakeraw(&tio);                    // RAWモード

    tcsetattr( *fd, TCSANOW, &tio );     // デバイスに設定を行う

    ioctl(*fd, TCSETS, &tio);            // ポートの設定を有効にする

    printf("connect open\n");

    return 0;

}

void read_buf(int *fd){
    unsigned char buf[255];
    int len, i;

    len = read(*fd, buf, sizeof(buf));
    if (0 < len) {
        for(i = 0; i < len; i++) {
            printf("%02X", buf[i]);
        }
        printf("\n");
    }

}