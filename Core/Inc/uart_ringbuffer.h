#ifndef _UART_RINGBUFFER_H_
#define _UART_RINGBUFFER_H_
 
#include "usart.h"
#include "string.h"
 
#define RXTXBUF_SIZE 128
 
/*串口环形缓冲串口接收*/
typedef struct
{
    UART_HandleTypeDef dut_huart; //为串口 huart1
    unsigned char *pRxBuf;    //接收缓冲区
    unsigned int usRxBufSize; //接收大小
    unsigned int usRxWrite;    //当前位置
    unsigned int usRxRead;   //读取位置
    unsigned char rec_end_flag; //接收完成标志位
    DMA_HandleTypeDef dut_dma; //接收DMA
    unsigned char *dmaRxbuf;  //DMA接收指针
    volatile unsigned int dmaRxlen; //DMA 接收长度
 
}DUT_USART_FIFO;
 
unsigned char GetBuf(DUT_USART_FIFO *_pUart,unsigned char *p_ucBuf,unsigned int *_usBufLen);
void UartIRQ(DUT_USART_FIFO *_pUart);
void EnabledUart(DUT_USART_FIFO *_pUart);
 
#endif
