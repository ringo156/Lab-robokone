/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#ifndef PC_CONTROL
#define PC_CONTROL
#include <project.h>
#include "Servo.h"

//#define DEBUG_RX_DATA

/*************************************************
 * PCとの通信について...
 * 
 * 5Byte単位で通信を行う.
 * +------+---------+-------+-------+-------+
 * | 0x55 | Command | data1 | data2 | Blank |
 * +------+---------+-------+-------+-------+
 * 最後のバイトにはたぶんそのうちCRCにする. 今は空.
 * 
 * Command
 *  bit     +- 7 -+--- 6 ~ 0 ---+
 *  content | MSF | Mode Number |
 *          + ----+-------------+
 *
 *     MSF Mode Switching Flag 
 *         モード切り替え時に1
 *     Mode Number
 *         MSF=0の時は現在のモード
 *         MSF=1の時は次に切り替えるモード
 *
 * data1 data2
 *     モードに渡されるデータ
 *
 * Blank
 *     空. そのうちCRCいれるかもと思って開けてる
 *
 **************************************************/

//#define COM_COMMAND_MODE 0x7f
//#define COM_COMMAND_CHANGEFLAG 0x80
#define RUN_DIRECTION 0xC0
#define RUN_SPEED     0x3f
    
union Servo_rx{
    uint8 g_rxData[4];
    float angle;
};

typedef enum
{
    MODE_RUN = 1,
    MODE_LINETRACE,
    MODE_CATCH,
    MODE_SHOOT,
}Mode;

typedef enum
{
    STOP = 0,
    LOW,
    NORMAL,
    HIGH,
}Speed;

typedef enum
{
    RED = 0,
    YELLOW,
    BLUE,
}Color;

typedef enum
{
    FAILED = 0xf0,
    SUCCESS,
}StateCatch;


//--- 受信データ構造 ---//
/*typedef union
{
    uint8 whole;
    struct
    {
        Mode mode        :2;
        uint8 bit5        :1;
        uint8 bit4        :1;
        uint8 bit3        :1;
        uint8 bit2        :1;
        uint8 bit1        :1;
        uint8 bit0        :1;
    }Bit;
    
    struct 
    {
        Mode modeSelect  :2;
        uint8 pad         :1;
        uint8 move        :1;
        uint8 run         :1;
        uint8 turn        :1;
        Speed speed       :2;
    }ApproachMode;
    
    struct
    {
        Mode modeSelect  :2;
        uint8 pad         :3;
        uint8 targetPos   :3;
    }LineTraceMode;
    
    struct
    {
        Mode modeSelect        :2;
        Color color            :2;
        StateCatch stateCatch  :2;
        uint8 pad              :2;
    }CatchMode;
    
    struct
    {
        Mode modeSelect  :2;
        uint8 color       :2;
        uint8 pad         :4;
    }ShootMode;
}RxData;*/

typedef struct
{
    uint8 command;
    uint8 data1;
    uint8 data2;
}ComData;


/*
 * Methods
*/
void initComPc();
void setTxData(ComData *txData);
int getRxData(ComData *rxData);
void getRxRadian(float *radian, uint8 val);


#endif /* PC_CONTROL */

/* [] END OF FILE */
