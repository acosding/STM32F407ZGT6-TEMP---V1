#include "uart_ringbuffer.h"
 
unsigned char GetChar(DUT_USART_FIFO *_pUart, unsigned char *_pByte)
{
    unsigned int usRxWrite=_pUart->usRxWrite;
    if (_pUart->usRxRead == usRxWrite)
    {
            return 0;
    }
    else
    {
        *_pByte = _pUart->pRxBuf[_pUart->usRxRead];
        if (++_pUart->usRxRead >= _pUart->usRxBufSize)
        {
            _pUart->usRxRead = 0;
        }
        return 1;
    }
}
unsigned char GetBuf(DUT_USART_FIFO *_pUart,unsigned char *p_ucBuf,unsigned int *_usBufLen)
{
    unsigned int i =0;
    unsigned int usRxBufSize = _pUart->usRxBufSize;
    unsigned int usDataRxNum;
    unsigned int usRxWrite,usRxRead;
    unsigned char aside;
 
    if ( _pUart->usRxRead == _pUart->usRxWrite )
    {
        *_usBufLen = 0;
        return 0;
    }
    usRxWrite = _pUart->usRxWrite; //usRxWrite为缓冲区下一次要写入的位置//
    usRxRead = _pUart->usRxRead;
    (usRxWrite > usRxRead) ? (usDataRxNum  = usRxWrite - usRxRead)
                           : (usDataRxNum  = usRxWrite + usRxBufSize - usRxRead);
    *_usBufLen = usDataRxNum;  
    while ( _pUart->usRxRead != _pUart->usRxWrite )
    {
         if ( i++ < usDataRxNum )
         {
             GetChar( _pUart, p_ucBuf++ );
          }
            else
            {
                GetChar( _pUart, &aside );
            }
      }
     return 1;
}
 
void UartIRQ(DUT_USART_FIFO *_pUart)
{
    if((__HAL_UART_GET_FLAG(&_pUart->dut_huart,UART_FLAG_IDLE) != RESET)){
			__HAL_UART_CLEAR_IDLEFLAG(&_pUart->dut_huart);//清除标志
      HAL_UART_DMAStop(&_pUart->dut_huart); //停止接收，这边一定要要这条，不然会出现上一条和本次接收的一起回复回来的情况
      _pUart->dmaRxlen  = RXTXBUF_SIZE - __HAL_DMA_GET_COUNTER(&_pUart->dut_dma);
//            printf("2._pUart->dmaRxlen %d\n",_pUart->dmaRxlen);
      for (unsigned int i = 0; i < _pUart->dmaRxlen; i++){
				_pUart->pRxBuf[_pUart->usRxWrite]=_pUart->dmaRxbuf[i];
        if (++_pUart->usRxWrite>=_pUart->usRxBufSize){
					_pUart->usRxWrite = 0;
        }        
      }
      memset(_pUart->dmaRxbuf,0x00,sizeof(_pUart->dmaRxbuf));
      _pUart->dmaRxlen = 0;
      _pUart->rec_end_flag = 1; // 接受完成标志位置1
			HAL_UART_Receive_DMA(&_pUart->dut_huart,_pUart->dmaRxbuf,RXTXBUF_SIZE);//重新打开DMA接收
			
    }  
}
 
void EnabledUart(DUT_USART_FIFO *_pUart)
{
  __HAL_UART_ENABLE_IT(&_pUart->dut_huart, UART_IT_IDLE); 
  HAL_UART_Receive_DMA(&_pUart->dut_huart,_pUart->dmaRxbuf,RXTXBUF_SIZE);
}
