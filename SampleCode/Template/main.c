/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define LED_R							(PH0)
#define LED_Y							(PH1)
#define LED_G							(PH2)

#define TEST_NUMBER 					(16)   /* page numbers */

#define SPI_FLASH_PORT  					(SPI0)
#define SPI_CLK_FREQ  					(1000000)	//(20000000)

#define SPI_MASTER_TX_DMA_CH 			(9)
#define SPI_MASTER_RX_DMA_CH 			(10)
#define SPI_FLASH_PDMA_OPENED_CH   	((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH))
#define SPI_FLASH_PAGE_BYTE 				(0x100)

uint8_t TxBuffer[SPI_FLASH_PAGE_BYTE] = {0};
uint8_t RxBuffer[SPI_FLASH_PAGE_BYTE] = {0};

uint8_t SPI_FLASH_page_counter = 0;

//#define CUSTOM_SPI_FLASH_PIN

/*
	Device ID (command: AB hex) : 15
	Device ID (command: 90 hex) : C2 15
	RDID (command: 9F hex) : C2 20 16	
*/
#define CUSTOM_SPI_FLASH_ID				(0xC215)	//MX25L3205D

enum
{
	SPI_TX = 0,
	SPI_RX = 1,		
};

typedef enum{

	flag_uart_rx = 0 ,
	flag_error ,	
	flag_ccc ,
	
	flag_DEFAULT	
}Flag_Index;

uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(uint8_t *pucBuff, int nBytes)
{
	#if 1
    uint16_t i = 0;	
    for ( i = 0; i < nBytes; i++)
    {
        pucBuff[i] = 0x00;
    }	
	#else	//extra 20 bytes , with <string.h>
	memset(pucBuff, 0, nBytes * (sizeof(pucBuff[0]) ));
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void SPI_Master_RX_PDMA(uint8_t* Rx , uint16_t len)
{
	uint32_t u32RegValue = 0;
	uint32_t u32Abort = 0;	
	
    PDMA_Open(PDMA, (1 << SPI_MASTER_RX_DMA_CH));

	//RX	
    PDMA_SetTransferCnt(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_8, len);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_MASTER_RX_DMA_CH, (uint32_t)&SPI_FLASH_PORT->RX, PDMA_SAR_FIX, (uint32_t)Rx, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_128);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    SPI_TRIGGER_RX_PDMA(SPI_FLASH_PORT);

    while(1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS(PDMA);
        /* Check the DMA transfer done interrupt flag */
        if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done interrupt flags */
            if((PDMA_GET_TD_STS(PDMA) & (1 << SPI_MASTER_RX_DMA_CH)) == (1 << SPI_MASTER_RX_DMA_CH))
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA,1 << SPI_MASTER_RX_DMA_CH);
                /* Disable SPI PDMA RX function */
                SPI_DISABLE_RX_PDMA(SPI_FLASH_PORT);
                break;
            }

            /* Check the DMA transfer abort interrupt flag */
            if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA_GET_ABORT_STS(PDMA);
                /* Clear the target abort flag */
                PDMA_CLR_ABORT_FLAG(PDMA,u32Abort);
                break;
            }
        }
    }

}

void SPI_Master_TX_PDMA(uint8_t* Tx , uint16_t len)
{
	uint32_t u32RegValue = 0;
	uint32_t u32Abort = 0;	

    PDMA_Open(PDMA, (1 << SPI_MASTER_TX_DMA_CH));

	//TX
    PDMA_SetTransferCnt(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_8, len);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_MASTER_TX_DMA_CH, (uint32_t)Tx, PDMA_SAR_INC, (uint32_t)&SPI_FLASH_PORT->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_128);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    SPI_TRIGGER_TX_PDMA(SPI_FLASH_PORT);

    while(1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS(PDMA);
        /* Check the DMA transfer done interrupt flag */
        if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done interrupt flags */
            if((PDMA_GET_TD_STS(PDMA) & (1 << SPI_MASTER_TX_DMA_CH)) == (1 << SPI_MASTER_TX_DMA_CH))
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA,1 << SPI_MASTER_TX_DMA_CH);
                /* Disable SPI PDMA TX function */
                SPI_DISABLE_TX_PDMA(SPI_FLASH_PORT);
                break;
            }

            /* Check the DMA transfer abort interrupt flag */
            if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA_GET_ABORT_STS(PDMA);
                /* Clear the target abort flag */
                PDMA_CLR_ABORT_FLAG(PDMA,u32Abort);
                break;
            }
        }
    }

}

uint16_t SpiFlash_ReadMidDid(void)
{
    uint8_t u8RxData[6], u8IDCnt = 0;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x90, Read Manufacturer/Device ID
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x90);

    // send 24-bit '0', dummy
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // receive 16-bit
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    while(!SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT))
        u8RxData[u8IDCnt ++] = SPI_READ_RX(SPI_FLASH_PORT);

    return ( (u8RxData[4]<<8) | u8RxData[5] );
}

void SpiFlash_ChipErase(void)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    //////////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0xC7, Chip Erase
    SPI_WRITE_TX(SPI_FLASH_PORT, 0xC7);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
}

uint8_t SpiFlash_ReadStatusReg(void)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x05, Read status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x05);

    // read status
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    SPI_READ_RX(SPI_FLASH_PORT);

    return (SPI_READ_RX(SPI_FLASH_PORT) & 0xff);
}

void SpiFlash_WriteStatusReg(uint8_t u8Value)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    ///////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x01, Write status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x01);

    // write status
    SPI_WRITE_TX(SPI_FLASH_PORT, u8Value);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

void SpiFlash_WaitReady(void)
{
    uint8_t ReturnValue = 0;
//    uint32_t cnt = 0;
	
    do
    {
        ReturnValue = SpiFlash_ReadStatusReg();
        ReturnValue = ReturnValue & 1;

		#if 1	//debug purpose
//		printf("BUSY counter : %4d\r\n" , cnt++);
//		printf(".");
		LED_Y ^= 1;
		
		#endif

    }
    while(ReturnValue!=0);   // check the BUSY bit

	printf("\r\n");
}

void SpiFlash_NormalPageProgram(uint32_t StartAddress, uint8_t *u8DataBuffer , uint8_t EnablePDMA)
{
    uint32_t i = 0;
	
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);


    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x02, Page program
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x02);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress>>16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress>>8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);


    // write data
	if (EnablePDMA)
	{
		SPI_Master_TX_PDMA(u8DataBuffer , SPI_FLASH_PAGE_BYTE);
	}
	else
	{
	    while(1)
	    {
	        if(!SPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT))
	        {
//				printf("%3d\r\n" , i);			
	            SPI_WRITE_TX(SPI_FLASH_PORT, u8DataBuffer[i]);
	            if (i++ >= (SPI_FLASH_PAGE_BYTE-1) )
					break;				
	        }
	    }
	}

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
}

void SpiFlash_PageWrite(uint32_t page_no, uint8_t *u8DataBuffer , uint8_t EnablePDMA)
{
	SpiFlash_NormalPageProgram(page_no*SPI_FLASH_PAGE_BYTE , u8DataBuffer , EnablePDMA);
	SpiFlash_WaitReady();
}

void SpiFlash_NormalRead(uint32_t StartAddress, uint8_t *u8DataBuffer , uint8_t EnablePDMA)
{
    uint32_t i = 0;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x03, Read data
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x03);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress>>16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress>>8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    while(SPI_IS_BUSY(SPI_FLASH_PORT));
    // clear RX buffer
    SPI_ClearRxFIFO(SPI_FLASH_PORT);

    // read data
    if (EnablePDMA)
    {
		SPI_Master_RX_PDMA(u8DataBuffer , SPI_FLASH_PAGE_BYTE);
    }
	else
	{
	    for(i = 0 ; i < SPI_FLASH_PAGE_BYTE ; i++)
	    {
	        SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
	        while(SPI_IS_BUSY(SPI_FLASH_PORT));
	        u8DataBuffer[i] = SPI_READ_RX(SPI_FLASH_PORT);
	    }
	}

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

void SpiFlash_PageRead(uint32_t page_no, uint8_t *u8DataBuffer , uint8_t EnablePDMA)
{
	SpiFlash_NormalRead(page_no*SPI_FLASH_PAGE_BYTE , u8DataBuffer , EnablePDMA);
}

/*
TARGET
	SPI SS : PB.15
	SPI CLK : PB.14		
	SPI MISO : PB.13	
	SPI MOSI : PB.12	

SAMPLE CODE
	SPI SS : PA.3	
	SPI CLK : PA.2			
	SPI MISO : PA.1		
	SPI MOSI : PA.0	
    
*/

void SpiFlash_Init(void)
{
    uint16_t u16ID = 0;
    uint16_t i = 0;
	
    /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 20MHz */
    SPI_Open(SPI_FLASH_PORT, SPI_MASTER, SPI_MODE_0, 8, SPI_CLK_FREQ);

    /* Disable auto SS function, control SS signal manually. */
    SPI_DisableAutoSS(SPI_FLASH_PORT);
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

	u16ID = SpiFlash_ReadMidDid();
	printf("ID : 0x%2X\r\n" , u16ID);

	
	//initial TX , RX data
    for (i=0; i < SPI_FLASH_PAGE_BYTE; i++)
    {
        TxBuffer[i] = 0xFF;
        RxBuffer[i] = 0xFF;
    }

}

void UARTx_Process(void)
{
	uint8_t res = 0;
    uint16_t i = 0;
	static uint8_t cnt = 0;
    uint16_t page_cnt = 0;
	
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '?':

				printf("\r\n==========================\r\n");
				
				printf("1: chip erase\r\n");
				printf("2: fill in TX data\r\n");
				printf("3: page counter\r\n");
				printf("4: Page write\r\n");
				printf("5: Page read\r\n");		
				printf("6: self test , write , read , compare\r\n");	
				printf("7: self test , write , read , compare  , with PDMA\r\n");
				
				printf("8: read ID (0x90)\r\n");	
				
				printf("==========================\r\n\r\n");
				break;	

		
			case '1':
				printf("perform SpiFlash_ChipErase\r\n");
				
			    /* Erase SPI flash */
			    SpiFlash_ChipErase();

			    /* Wait ready */
			    SpiFlash_WaitReady();

				printf("erase finish\r\n\r\n");
				break;	

			case '2':
				printf("increase test data start from 0x%2X\r\n" , cnt);

				//reset TxBuffer
				reset_buffer(TxBuffer,SPI_FLASH_PAGE_BYTE);

				//fill in data
			    for ( i = 0; i < SPI_FLASH_PAGE_BYTE; i++)
			    {
			        TxBuffer[i] = 0x00 + i + cnt;
			    }

				printf("TxBuffer : \r\n");
				dump_buffer_hex(TxBuffer,SPI_FLASH_PAGE_BYTE);				
				cnt++; 
			
				break;

			case '3':
				printf("SPI_FLASH_page_counter current : 0x%2X\r\n\r\n" ,SPI_FLASH_page_counter++);	
			
				break;

			case '4':
				printf("programming...\r\n");
				SpiFlash_PageWrite(SPI_FLASH_page_counter,TxBuffer,DISABLE);
				printf("programming finish\r\n\r\n");
				
				break;

			case '5':
				//reset RxBuffer
			    reset_buffer(RxBuffer,SPI_FLASH_PAGE_BYTE);

				printf("read page ...\r\n");
				SpiFlash_PageRead(SPI_FLASH_page_counter,RxBuffer,DISABLE);
				dump_buffer_hex(RxBuffer,SPI_FLASH_PAGE_BYTE);				
				printf("read page finish\r\n\r\n");	
				
				break;				

			case '6':
				//reset RxBuffer
			    reset_buffer(RxBuffer,SPI_FLASH_PAGE_BYTE);

				printf("perform SpiFlash_ChipErase\r\n");
				
			    /* Erase SPI flash */
			    SpiFlash_ChipErase();

			    /* Wait ready */
			    SpiFlash_WaitReady();

				printf("erase finish\r\n\r\n");

				for ( page_cnt = 0 ; page_cnt < TEST_NUMBER ; page_cnt++)
				{
					printf("\r\nSELF TEST ... (page : %2d)\r\n" , page_cnt);

					//reset TxBuffer
					reset_buffer(TxBuffer,SPI_FLASH_PAGE_BYTE);

					//fill in data
				    for ( i = 0; i < SPI_FLASH_PAGE_BYTE; i++)
				    {
				        TxBuffer[i] = 0x00 + i + cnt;
				    }

//					printf("\r\nTxBuffer : \r\n");
//					dump_buffer_hex(TxBuffer,SPI_FLASH_PAGE_BYTE);				
					cnt++;
					
					SpiFlash_PageWrite(page_cnt,TxBuffer,DISABLE);
					SpiFlash_PageRead(page_cnt,RxBuffer,DISABLE);

//					printf("\r\nRxBuffer\r\n");
//					dump_buffer_hex(RxBuffer,SPI_FLASH_PAGE_BYTE);		

					compare_buffer(TxBuffer,RxBuffer,SPI_FLASH_PAGE_BYTE);
				}

				printf("SELF TEST finish\r\n\r\n");	
			
				break;	


			case '7':

				//reset RxBuffer
			    reset_buffer(RxBuffer,SPI_FLASH_PAGE_BYTE);

				printf("perform SpiFlash_ChipErase\r\n");
				
			    /* Erase SPI flash */
			    SpiFlash_ChipErase();

			    /* Wait ready */
			    SpiFlash_WaitReady();

				printf("erase finish\r\n\r\n");

				for ( page_cnt = 0 ; page_cnt < TEST_NUMBER ; page_cnt++)
				{
					printf("\r\nPDMA SELF TEST ... (page : %2d)\r\n" , page_cnt);

					//reset TxBuffer
					reset_buffer(TxBuffer,SPI_FLASH_PAGE_BYTE);

					//fill in data
				    for ( i = 0; i < SPI_FLASH_PAGE_BYTE; i++)
				    {
				        TxBuffer[i] = 0x00 + i + cnt;
				    }

//					printf("\r\nTxBuffer : \r\n");
//					dump_buffer_hex(TxBuffer,SPI_FLASH_PAGE_BYTE);				
					cnt++;
					
					SpiFlash_PageWrite(page_cnt,TxBuffer,ENABLE);
					SpiFlash_PageRead(page_cnt,RxBuffer,ENABLE);

//					printf("\r\nRxBuffer\r\n");
//					dump_buffer_hex(RxBuffer,SPI_FLASH_PAGE_BYTE);		

					compare_buffer(TxBuffer,RxBuffer,SPI_FLASH_PAGE_BYTE);
				}

				printf("PDMA SELF TEST finish\r\n\r\n");	

				break;

			case '8':			
				i = SpiFlash_ReadMidDid();
				printf("SpiFlash_ReadMidDid : 0x%2X\r\n\n" , i);
				
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();
			
				break;		
			
		}
	}
}

void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
//			set_flag(flag_uart_rx,ENABLE);
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}

void TMR1_IRQHandler(void)
{
	static uint16_t CNT = 0;	
//	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
	
		if (CNT++ > 1000)
		{		
			CNT = 0;
//			printf("%s : %2d\r\n" , __FUNCTION__ , log++);

			LED_G ^= 1;
		}
    }
}

void TIMER1_HW_Init(void)
{
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void TIMER0_HW_Init(void)
{
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
}

void TIMER0_Polling(uint32_t u32Usec)
{
	TIMER_Delay(TIMER0, u32Usec);
}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
    CLK_EnableModuleClock(SPI0_MODULE);

    CLK_EnableModuleClock(PDMA_MODULE);

	TIMER0_HW_Init();
	TIMER1_HW_Init();
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

	//conflict with EVM UART pin , PB12/PB13
	#if defined (CUSTOM_SPI_FLASH_PIN)	

    /* Setup SPI0 multi-function pins */
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB12MFP_SPI0_MOSI | SYS_GPB_MFPH_PB13MFP_SPI0_MISO | SYS_GPB_MFPH_PB14MFP_SPI0_CLK | SYS_GPB_MFPH_PB15MFP_SPI0_SS;

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN14_Msk;

    /* Enable SPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PB, 0xF, GPIO_SLEWCTL_HIGH);

	#else
    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS;

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Enable SPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, 0xF, GPIO_SLEWCTL_HIGH);
	#endif
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{	
    SYS_Init();
	UART0_Init();

	LED_Init();
	TIMER1_Init();

	SpiFlash_Init();


    /* Got no where to go, just loop forever */
    while(1)
    {
//		TIMER0_Polling(1000);

		if (is_flag_set(flag_uart_rx))
		{
			set_flag(flag_uart_rx,DISABLE);
//			UARTx_Process();
		}
	
    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
