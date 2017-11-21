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
#include <stdio.h>
#include "pc_control.h"

/* 通信状態 */
#define COM_SENDING   0x01
#define COM_RECEIVING 0x02
#define COM_TIMEOUT   0x04
#define COM_USING     0x08
#define COM_CONFLICT  0x10
#define COM_UNREAD    0x20
uint8 g_comStateFlag;

/* 受信データ */
uint8  g_rxCommand;
uint8  g_rxData1;
uint8  g_rxData2;
uint8  g_tmpRxData[3];  // 衝突したときの一時的な保存場所
uint8  g_rxBytes = 0;
uint8  g_rxBuffer[8];
union Servo_rx rx_data[5];

/* 送信データ */
uint8  g_txCommand;
uint8  g_txData1;
uint8  g_txData2;

uint8  g_txCounter = 0;

/* debug */
extern char Data[255];
extern uint8 debug_flag;
uint8 link;


CY_ISR(pc_tx_isr)
{   
    if (g_txCounter >= 4)
    {
        //g_comStateFlag |= COM_RECEIVING;
        g_comStateFlag &= ~COM_SENDING;
    }else
    {
        g_txCounter++;
    }
}

CY_ISR(pc_rx_isr)
{
   
    uint8 receivedData = ComPc_GetByte();
    
    //sprintf(Data, "%x\n", (int)receivedData);
    //debug_flag = 1;
    if(g_rxBytes == 0)
    {
        if(receivedData != 0x55)
        {
            return;
        }
        g_rxBuffer[g_rxBytes] = receivedData;
        g_rxBytes++;
    }else
    // Command
    if(g_rxBytes == 1)
    {
        //g_rxBuffer[g_rxBytes] = receivedData;
        link = receivedData;//1から始まる
        g_rxBytes++;
    }else
    // Data1
    if(g_rxBytes == 2)
    {
        //g_rxBuffer[g_rxBytes] = receivedData;
        rx_data[link].g_rxData[3] = receivedData;
        g_rxBytes++;
    }else
    // Data2
    if(g_rxBytes == 3)
    {
        //g_rxBuffer[g_rxBytes] = receivedData;
        rx_data[link].g_rxData[2] = receivedData;
        g_rxBytes++;
    }else
    if(g_rxBytes == 4)
    {
        //g_rxBuffer[g_rxBytes] = receivedData;
        rx_data[link].g_rxData[1] = receivedData;
        g_rxBytes++;
    }
    else
    if(g_rxBytes == 5)
    {
        //g_rxBuffer[g_rxBytes] = receivedData;]
        rx_data[link].g_rxData[0] = receivedData;
        g_rxBytes = 0;
        //if(!(g_comStateFlag & COM_USING))
        //{
            //g_comStateFlag |= COM_USING;
            g_rxCommand = g_rxBuffer[1];
            g_rxData1   = g_rxBuffer[2];
            g_rxData2   = g_rxBuffer[3];
            
    }
}


void initComPc()
{
    g_comStateFlag = 0;
    g_rxCommand = 0;
    g_rxData1 = 0;
    g_rxData2 = 0;
    g_txCommand = 0;
    g_txData1 = 0;
    g_txData2 = 0;
    ComPc_Start();
    ComPc_EnableRxInt();
    ComPc_EnableTxInt();
    PcTxIsr_StartEx(pc_tx_isr);
    PcRxIsr_StartEx(pc_rx_isr);
}


void setTxData(ComData *txData)
{
    //g_txData = rxData.whole;
    /*g_txCommand = txData->command;
    g_txData1   = txData->data1;
    g_txData2   = txData->data2;*/
    static uint8 old_command = 0xff;
    static uint8 old_data1   = 0xff;
    static uint8 old_data2   = 0xff;
    
    if( txData->command == old_command &&
        txData->data1   == old_data1   &&
        txData->data2   == old_data2      )
    {
        return;
    }
    
    if(g_comStateFlag & COM_RECEIVING)
    {
        return;
    }
    g_comStateFlag |= COM_SENDING;
    old_command = txData->command;
    old_data1   = txData->data1;
    old_data2   = txData->data2;
    ComPc_PutChar(0x55);
    ComPc_PutChar(old_command);
    ComPc_PutChar(old_data1);
    ComPc_PutChar(old_data2);
    ComPc_PutChar(0x00);
    
    /*debug_flag = 1;
    sprintf(Data, "0:%2x, 1:%2x, 2:%2x, 3:%2x, 4:%2x\n", 
        (int)0x55,
        (int)old_command,
        (int)old_data1,
        (int)old_data2,
        (int)0x00
    );*/
}


int getRxData(ComData *rxData)
{
    //RxData ret;
    //ret.whole = g_rxData;
    /*if(!(g_comStateFlag & COM_CONFLICT))
    {
        g_comStateFlag |= COM_USING;
        rxData->command = g_rxCommand;
        rxData->data1 = g_rxData1;
        rxData->data2 = g_rxData2;
        g_comStateFlag &= ~COM_USING;
        g_comStateFlag &= ~COM_UNREAD;
    }else
    {
        if (g_comStateFlag & COM_UNREAD)
        {
            rxData->command = g_rxCommand = g_tmpRxData[0];
            rxData->data1   = g_rxData1   = g_tmpRxData[1];
            rxData->data2   = g_rxData2   = g_tmpRxData[2];
            g_comStateFlag &= ~COM_CONFLICT;
        }
    }
    g_comStateFlag &= ~COM_RECEIVING;
    */
    //ComPc_DisableRxInt();
    rxData->command = g_rxCommand;
    rxData->data1   = g_rxData1;
    rxData->data2   = g_rxData2;
    //ComPc_EnableRxInt();
    
    return 0;
}

void getRxRadian(float *radian, uint8 val){
    uint8 i;
    for(i=0;i<val;i++){
        radian[i] = rx_data[i+1].angle;
    }
}

/* [] END OF FILE */
