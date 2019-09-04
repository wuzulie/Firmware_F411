/**
  ******************************************************************************
  * @file    usbd_cdc_vcp.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED 
#pragma     data_alignment = 4 
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

#include "usbd_cdc_vcp.h"
#include "usb_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;
static xQueueHandle usbDataDelivery;

LINE_CODING linecoding =
  {
    500000, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };
  
#define USB_RX_LEN	 	200	//USB接收数据缓存区
uint8_t usb_rx_buf[USB_RX_LEN];
uint8_t	usb_rx_ptr_in = 0;
uint8_t usb_rx_ptr_out = 0;

/* These are external variables imported from CDC core to be used for IN 
   transfer management. */
extern uint8_t  APP_Rx_Buffer []; /* Write CDC received data in this buffer.
                                     These data will be sent over USB IN endpoint
                                     in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */

/* Private function prototypes -----------------------------------------------*/
static uint16_t VCP_Init     (void);
static uint16_t VCP_DeInit   (void);
static uint16_t VCP_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataTx   (uint8_t data);
static uint16_t VCP_DataRx   (uint8_t* Buf, uint32_t Len);

//static uint16_t VCP_COMConfig(uint8_t Conf);

CDC_IF_Prop_TypeDef VCP_fops = 
{
  VCP_Init,
  VCP_DeInit,
  VCP_Ctrl,
  VCP_DataTx,
  VCP_DataRx
};

static uint16_t VCP_Init(void)
{
  return USBD_OK;
}

static uint16_t VCP_DeInit(void)
{
  return USBD_OK;
}


/**
  * @brief  VCP_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{ 
  switch (Cmd)
  {
  case SEND_ENCAPSULATED_COMMAND:
    /* Not  needed for this driver */
    break;

  case GET_ENCAPSULATED_RESPONSE:
    /* Not  needed for this driver */
    break;

  case SET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case GET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case CLEAR_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case SET_LINE_CODING:
    linecoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));
    linecoding.format = Buf[4];
    linecoding.paritytype = Buf[5];
    linecoding.datatype = Buf[6];
    break;

  case GET_LINE_CODING:
    Buf[0] = (uint8_t)(linecoding.bitrate);
    Buf[1] = (uint8_t)(linecoding.bitrate >> 8);
    Buf[2] = (uint8_t)(linecoding.bitrate >> 16);
    Buf[3] = (uint8_t)(linecoding.bitrate >> 24);
    Buf[4] = linecoding.format;
    Buf[5] = linecoding.paritytype;
    Buf[6] = linecoding.datatype; 
    break;

  case SET_CONTROL_LINE_STATE:
    /* Not  needed for this driver */
    break;

  case SEND_BREAK:
    /* Not  needed for this driver */
    break;    
    
  default:
    break;
  }

  return USBD_OK;
}

/**
  * @brief  VCP_DataTx
  *         CDC received data to be send over USB IN endpoint are managed in 
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
static uint16_t VCP_DataTx (uint8_t data)
{
  if (linecoding.datatype == 7)
  {
    APP_Rx_Buffer[APP_Rx_ptr_in] = data & 0x7F;
  }
  else if (linecoding.datatype == 8)
  {
    APP_Rx_Buffer[APP_Rx_ptr_in] = data;
  }
  
  APP_Rx_ptr_in++;
  
  /* To avoid buffer overflow */
  if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
  {
    APP_Rx_ptr_in = 0;
  }  
  
  return USBD_OK;
}

/**
  * @brief  VCP_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
static uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
//  uint32_t i;
//  if(usb_rx_ptr_in+1 != usb_rx_ptr_out)
//  {
//	  for (i = 0; i < Len; i++)
//	  {
//		usb_rx_buf[usb_rx_ptr_in] = Buf[i];
//		usb_rx_ptr_in++;
//		if(usb_rx_ptr_in >= USB_RX_LEN)
//			usb_rx_ptr_in = 0;
//	  } 
//  }
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	for(int i=0; i<Len; i++)
	{
		uint8_t data = Buf[i];
		xQueueSendFromISR(usbDataDelivery, &data, &xHigherPriorityTaskWoken);
	}
	return USBD_OK;
}

void usbd_cdc_vcp_Init(void)
{
	// 初始化usb
	USBD_Init(&USB_OTG_dev,
		USB_OTG_FS_CORE_ID,
		&USR_desc,
		&USBD_CDC_cb,
		&USR_cb);
	
	usbDataDelivery = xQueueCreate(128, sizeof(uint8_t));	/*队列 128个消息*/
}

bool usbGetDataWithTimout(uint8_t *c)
{
	if (xQueueReceive(usbDataDelivery, c, 1000) == pdTRUE)	/*接收usbDataDelivery(1024个容量)消息*/
	{
		return true;
	}
	*c = 0;
	return false;
}

void usbsendData(u8* data, u16 length)
{
	for(int i=0; i<length; i++)
	{
		VCP_DataTx(data[i]);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
