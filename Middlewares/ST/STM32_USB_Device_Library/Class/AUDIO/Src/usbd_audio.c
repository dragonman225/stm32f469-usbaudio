/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                AUDIO Class  Description
  *          ===================================================================
 *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 48KHz.
  *             - Bit resolution: 16
  *             - Number of channels: 2
  *             - No volume control
  *             - Mute/Unmute capability
  *             - Asynchronous Endpoints
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* BSPDependencies
  - "stm32xxxxx_{eval}{discovery}.c"
  - "stm32xxxxx_{eval}{discovery}_io.c"
  - "stm32xxxxx_{eval}{discovery}_audio.c"
  EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"
#include "usbd_ctlreq.h"

#include "stm32469i_discovery_audio.h"
/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_AUDIO
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Macros
  * @{
  */

// clang-format off
#define AUDIO_SAMPLE_FREQ(frq) (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

/*
 * Max packet size: (freq / 1000 + extra_samples) * channels * bytes_per_sample
 * e.g. (96000 / 1000 + 1) * 2(stereo) * 2(16bit) = 388
 */
#define AUDIO_PACKET_SZE(frq) (uint8_t)(((frq / 1000U + 1) * 2U * 2U) & 0xFFU), \
                              (uint8_t)((((frq / 1000U + 1) * 2U * 2U) >> 8) & 0xFFU)

/* Feature Unit Config */
#define AUDIO_CONTROL_FEATURES AUDIO_CONTROL_MUTE | AUDIO_CONTROL_VOL

/* Nomial feedback data for different frequencies */
#define AUDIO_FB_DEFAULT \
        (USBD_AUDIO_FREQ_DEFAULT == 96000) ? (96 << 22) \
      : (USBD_AUDIO_FREQ_DEFAULT == 48000) ? (48 << 22) \
      : (USBD_AUDIO_FREQ_DEFAULT == 44100) ? ((44 << 22) + (1 << 22) / 10) \
      : (48 << 22)

/* Feedback is limited to +/- 1kHz */
#define AUDIO_FB_DELTA (uint32_t)(1 << 22)
// clang-format on

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx);

static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx);

static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);

static uint8_t* USBD_AUDIO_GetCfgDesc(uint16_t* length);

static uint8_t* USBD_AUDIO_GetDeviceQualifierDesc(uint16_t* length);

static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef* pdev, uint8_t epnum);

static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef* pdev, uint8_t epnum);

static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef* pdev);

static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef* pdev);

static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef* pdev);

static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum);

static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum);

static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_REQ_GetMax(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_REQ_GetMin(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_REQ_GetRes(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);

static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);

static void AUDIO_OUT_StopAndReset(USBD_HandleTypeDef* pdev);

static void AUDIO_OUT_Restart(USBD_HandleTypeDef* pdev);

static uint8_t VOL_PERCENT(int16_t vol);

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_AUDIO = {
    USBD_AUDIO_Init,
    USBD_AUDIO_DeInit,
    USBD_AUDIO_Setup,
    USBD_AUDIO_EP0_TxReady,
    USBD_AUDIO_EP0_RxReady,
    USBD_AUDIO_DataIn,
    USBD_AUDIO_DataOut,
    USBD_AUDIO_SOF,
    USBD_AUDIO_IsoINIncomplete,
    USBD_AUDIO_IsoOutIncomplete,
    USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetDeviceQualifierDesc,
};

/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END = {
    /* Configuration 1 */
    0x09,                              /* bLength */
    USB_DESC_TYPE_CONFIGURATION,       /* bDescriptorType */
    LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ), /* wTotalLength 121 bytes*/
    HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
    0x02, /* bNumInterfaces */
    0x01, /* bConfigurationValue */
    0x00, /* iConfiguration */
    0xC0, /* bmAttributes  BUS Powred*/
    0x32, /* bMaxPower = 100 mA*/
    /* 09 byte*/

    /* USB Speaker Standard interface descriptor */
    AUDIO_INTERFACE_DESC_SIZE,   /* bLength */
    USB_DESC_TYPE_INTERFACE,     /* bDescriptorType */
    0x00,                        /* bInterfaceNumber */
    0x00,                        /* bAlternateSetting */
    0x00,                        /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,      /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOCONTROL, /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,    /* bInterfaceProtocol */
    0x00,                        /* iInterface */
    /* 09 byte*/

    /* USB Speaker Class-specific AC Interface Descriptor */
    AUDIO_INTERFACE_DESC_SIZE,       /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
    AUDIO_CONTROL_HEADER,            /* bDescriptorSubtype */
    0x00, /* 1.00 */                 /* bcdADC */
    0x01,
    0x27, /* wTotalLength = 39*/
    0x00,
    0x01, /* bInCollection */
    0x01, /* baInterfaceNr */
    /* 09 byte*/

    /* USB Speaker Input Terminal Descriptor */
    AUDIO_INPUT_TERMINAL_DESC_SIZE,  /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
    AUDIO_CONTROL_INPUT_TERMINAL,    /* bDescriptorSubtype */
    0x01,                            /* bTerminalID */
    0x01,                            /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
    0x01,
    0x00, /* bAssocTerminal */
    0x02, /* bNrChannels */
    0x03, /* wChannelConfig 0x0003  FL FR */
    0x00,
    0x00, /* iChannelNames */
    0x00, /* iTerminal */
    /* 12 byte*/

    /* USB Speaker Audio Feature Unit Descriptor */
    0x09,                            /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
    AUDIO_CONTROL_FEATURE_UNIT,      /* bDescriptorSubtype */
    AUDIO_OUT_STREAMING_CTRL,        /* bUnitID */
    0x01,                            /* bSourceID */
    0x01,                            /* bControlSize */
    AUDIO_CONTROL_FEATURES,          /* bmaControls(0) */
    0,                               /* bmaControls(1) */
    0x00,                            /* iTerminal */
    /* 09 byte*/

    /*USB Speaker Output Terminal Descriptor */
    0x09,                            /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
    AUDIO_CONTROL_OUTPUT_TERMINAL,   /* bDescriptorSubtype */
    0x03,                            /* bTerminalID */
    0x01,                            /* wTerminalType  0x0301*/
    0x03,
    0x00, /* bAssocTerminal */
    0x02, /* bSourceID */
    0x00, /* iTerminal */
    /* 09 byte*/

    /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
    /* Interface 1, Alternate Setting 0                                             */
    AUDIO_INTERFACE_DESC_SIZE,     /* bLength */
    USB_DESC_TYPE_INTERFACE,       /* bDescriptorType */
    0x01,                          /* bInterfaceNumber */
    0x00,                          /* bAlternateSetting */
    0x00,                          /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,        /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING, /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,      /* bInterfaceProtocol */
    0x00,                          /* iInterface */
    /* 09 byte*/

    /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
    /* Interface 1, Alternate Setting 1                                           */
    AUDIO_INTERFACE_DESC_SIZE,     /* bLength */
    USB_DESC_TYPE_INTERFACE,       /* bDescriptorType */
    0x01,                          /* bInterfaceNumber */
    0x01,                          /* bAlternateSetting */
    0x02,                          /* bNumEndpoints - 1 output & 1 feedback */
    USB_DEVICE_CLASS_AUDIO,        /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING, /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,      /* bInterfaceProtocol */
    0x00,                          /* iInterface */
    /* 09 byte*/

    /* USB Speaker Audio Streaming Interface Descriptor */
    AUDIO_STREAMING_INTERFACE_DESC_SIZE, /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
    AUDIO_STREAMING_GENERAL,             /* bDescriptorSubtype */
    0x01,                                /* bTerminalLink */
    0x01,                                /* bDelay */
    0x01,                                /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
    0x00,
    /* 07 byte*/

    /* USB Speaker Audio Type I Format Interface Descriptor */
    0x11,                            /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
    AUDIO_STREAMING_FORMAT_TYPE,     /* bDescriptorSubtype */
    AUDIO_FORMAT_TYPE_I,             /* bFormatType */
    0x02,                            /* bNrChannels */
    0x02,                            /* bSubFrameSize :  2 Bytes per frame (16bits) */
    0x10,                            /* bBitResolution (16-bits per sample) */
    0x03,                            /* bSamFreqType two frequencies supported */
    AUDIO_SAMPLE_FREQ(44100),        /* Audio sampling frequency coded on 3 bytes */
    AUDIO_SAMPLE_FREQ(48000),        /* Audio sampling frequency coded on 3 bytes */
    AUDIO_SAMPLE_FREQ(96000),        /* Audio sampling frequency coded on 3 bytes */
    /* 17 byte*/

    /* Endpoint 1 - Standard Descriptor */
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE,     /* bLength */
    USB_DESC_TYPE_ENDPOINT,                /* bDescriptorType */
    AUDIO_OUT_EP,                          /* bEndpointAddress 1 out endpoint*/
    USBD_EP_TYPE_ISOC_ASYNC,               /* bmAttributes */
    AUDIO_PACKET_SZE(USBD_AUDIO_FREQ_MAX), /* wMaxPacketSize in Bytes (freq / 1000 + extra_samples) * channels * bytes_per_sample */
    0x01,                                  /* bInterval */
    0x00,                                  /* bRefresh */
    AUDIO_IN_EP,                           /* bSynchAddress */
    /* 09 byte*/

    /* Endpoint - Audio Streaming Descriptor */
    AUDIO_STREAMING_ENDPOINT_DESC_SIZE, /* bLength */
    AUDIO_ENDPOINT_DESCRIPTOR_TYPE,     /* bDescriptorType */
    AUDIO_ENDPOINT_GENERAL,             /* bDescriptor */
    0x01,                               /* bmAttributes - Sampling Frequency control is supported. See UAC Spec 1.0 p.62 */
    0x00,                               /* bLockDelayUnits */
    0x00,                               /* wLockDelay */
    0x00,
    /* 07 byte*/

    /* Endpoint 2 - Standard Descriptor - See UAC Spec 1.0 p.63 4.6.2.1 Standard AS Isochronous Synch Endpoint Descriptor */
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE, /* bLength */
    USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
    AUDIO_IN_EP,                       /* bEndpointAddress */
    0x11,                              /* bmAttributes */
    0x03, 0x00,                        /* wMaxPacketSize in Bytes */
    0x01,                              /* bInterval 1ms */
    SOF_RATE,                          /* bRefresh 4ms = 2^2 */
    0x00,                              /* bSynchAddress */
                                       /* 09 byte*/
};

/** 
 * USB Standard Device Descriptor
 * @see https://www.keil.com/pack/doc/mw/USB/html/_u_s_b__device__qualifier__descriptor.html
 */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
};

volatile uint32_t tx_flag = 1;
volatile uint32_t is_playing = 0;
volatile uint32_t all_ready = 0;

volatile uint32_t fb_nom = AUDIO_FB_DEFAULT;
volatile uint32_t fb_value = AUDIO_FB_DEFAULT;
volatile uint32_t audio_buf_writable_size_last = AUDIO_TOTAL_BUF_SIZE / 2U;
volatile int32_t fb_raw = AUDIO_FB_DEFAULT;
volatile uint8_t fb_data[3] = {
    (uint8_t)((AUDIO_FB_DEFAULT & 0x0000FF00) >> 8),
    (uint8_t)((AUDIO_FB_DEFAULT & 0x00FF0000) >> 16),
    (uint8_t)((AUDIO_FB_DEFAULT & 0xFF000000) >> 24)};

/* FNSOF is critical for frequency changing to work */
volatile uint32_t fnsof = 0;

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Functions
  * @{
  */

/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx)
{
  USBD_AUDIO_HandleTypeDef* haudio;

  /* Open EP OUT */
  USBD_LL_OpenEP(pdev, AUDIO_OUT_EP, USBD_EP_TYPE_ISOC, AUDIO_OUT_PACKET);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 1U;

  /* Open EP IN */
  USBD_LL_OpenEP(pdev, AUDIO_IN_EP, USBD_EP_TYPE_ISOC, 3);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 1U;

  /* Flush feedback endpoint */
  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);

  /** 
   * Set tx_flag 1 to block feedback transmission in SOF handler since 
   * device is not ready.
   */
  tx_flag = 1U;

  /* Allocate Audio structure */
  pdev->pClassData = USBD_malloc(sizeof(USBD_AUDIO_HandleTypeDef));

  if (pdev->pClassData == NULL) {
    return USBD_FAIL;
  } else {
    haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;
    haudio->alt_setting = 0U;
    haudio->offset = AUDIO_OFFSET_UNKNOWN;
    haudio->wr_ptr = 0U;
    haudio->rd_ptr = 0U;
    haudio->rd_enable = 0U;
    haudio->freq = USBD_AUDIO_FREQ_DEFAULT;
    haudio->bit_depth = 16U;
    haudio->vol = USBD_AUDIO_VOL_DEFAULT;

    /* Initialize the Audio output Hardware layer */
    if (((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->Init(haudio->freq, VOL_PERCENT(haudio->vol), 0U) != 0) {
      return USBD_FAIL;
    }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Init
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef* pdev,
                                 uint8_t cfgidx)
{
  /* Flush all endpoints */
  USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);
  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);

  /* Close EP OUT */
  USBD_LL_CloseEP(pdev, AUDIO_OUT_EP);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 0U;

  /* Close EP IN */
  USBD_LL_CloseEP(pdev, AUDIO_IN_EP);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 0U;

  /* Clear feedback transmission flag */
  tx_flag = 0U;

  /* DeInit physical Interface components */
  if (pdev->pClassData != NULL) {
    ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->DeInit(0U);
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef* pdev,
                                USBD_SetupReqTypedef* req)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  uint16_t len;
  uint8_t* pbuf;
  uint16_t status_info = 0U;
  uint8_t ret = USBD_OK;

  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    /* AUDIO Class Requests */
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest) {
        case AUDIO_REQ_GET_CUR:
          AUDIO_REQ_GetCurrent(pdev, req);
          break;

        case AUDIO_REQ_GET_MAX:
          AUDIO_REQ_GetMax(pdev, req);
          break;

        case AUDIO_REQ_GET_MIN:
          AUDIO_REQ_GetMin(pdev, req);
          break;

        case AUDIO_REQ_GET_RES:
          AUDIO_REQ_GetRes(pdev, req);
          break;

        case AUDIO_REQ_SET_CUR:
          AUDIO_REQ_SetCurrent(pdev, req);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    /* Standard Requests */
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest) {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            USBD_CtlSendData(pdev, (uint8_t*)(void*)&status_info, 2U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE) {
            pbuf = USBD_AUDIO_CfgDesc + 18;
            len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);

            USBD_CtlSendData(pdev, pbuf, len);
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            USBD_CtlSendData(pdev, (uint8_t*)(void*)&haudio->alt_setting, 1U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            if ((uint8_t)(req->wValue) <= USBD_MAX_NUM_INTERFACES) {
              /* Do things only when alt_setting changes */
              if (haudio->alt_setting != (uint8_t)(req->wValue)) {
                haudio->alt_setting = (uint8_t)(req->wValue);
                if (haudio->alt_setting == 0U) {
                  AUDIO_OUT_StopAndReset(pdev);
                } else if (haudio->alt_setting == 1U) {
                  haudio->bit_depth = 16U;
                  AUDIO_OUT_Restart(pdev);
                } else if (haudio->alt_setting == 2U) {
                  haudio->bit_depth = 24U;
                  AUDIO_OUT_Restart(pdev);
                }
              }

              USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
            } else {
              /* Call the error management function (command will be nacked */
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return ret;
}

/**
  * @brief  USBD_AUDIO_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t* USBD_AUDIO_GetCfgDesc(uint16_t* length)
{
  *length = sizeof(USBD_AUDIO_CfgDesc);
  return USBD_AUDIO_CfgDesc;
}

/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef* pdev,
                                 uint8_t epnum)
{
  /* epnum is the lowest 4 bits of bEndpointAddress. See UAC 1.0 spec, p.61 */
  if (epnum == (AUDIO_IN_EP & 0xf)) {
    tx_flag = 0U;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  /**
   * Order of 3 bytes in feedback packet: { LO byte, MID byte, HI byte }
   * 
   * For example,
   * 48.000(dec) => 300000(hex, 8.16) => 0C0000(hex, 10.14) => transmit { 00, 00, 0C }
   * 
   * Note that ALSA accepts 8.16 format.
   */

  /**
   * 1. Must be static so that the values are kept when the function is 
   *    again called.
   * 2. Must be volatile so that it will not be optimized out by the compiler.
   */
  // static volatile uint32_t fb_value = AUDIO_FB_DEFAULT;
  // static volatile uint32_t audio_buf_writable_size_last = AUDIO_TOTAL_BUF_SIZE / 2U;
  // static volatile int32_t fb_raw = AUDIO_FB_DEFAULT;
  static volatile uint32_t sof_count = 0;

  /* Do stuff only when playing */
  if (haudio->rd_enable == 1U && all_ready == 1U) {
    /* Remaining writable buffer size */
    uint32_t audio_buf_writable_size;

    /* Update audio read pointer */
    haudio->rd_ptr = AUDIO_TOTAL_BUF_SIZE - BSP_AUDIO_OUT_GetRemainingDataSize();

    /* Calculate remaining writable buffer size */
    if (haudio->rd_ptr < haudio->wr_ptr) {
      audio_buf_writable_size = haudio->rd_ptr + AUDIO_TOTAL_BUF_SIZE - haudio->wr_ptr;
    } else {
      audio_buf_writable_size = haudio->rd_ptr - haudio->wr_ptr;
    }

    /* Monitor remaining writable buffer size with LED */
    if (audio_buf_writable_size < AUDIO_BUF_SAFEZONE) {
      BSP_LED_On(LED2);
    } else {
      BSP_LED_Off(LED2);
    }

    sof_count += 1;

    if (sof_count == 1U) {
      sof_count = 0;
      /* Calculate feedback value based on the change of writable buffer size */
      // int32_t audio_buf_writable_size_change;
      int32_t audio_buf_writable_size_dev_from_nom;
      // audio_buf_writable_size_change = audio_buf_writable_size - audio_buf_writable_size_last;
      audio_buf_writable_size_dev_from_nom = audio_buf_writable_size - (AUDIO_TOTAL_BUF_SIZE >> 1);
      fb_value += audio_buf_writable_size_dev_from_nom * 2702;
      // fb_raw += audio_buf_writable_size_change * 0x1000;
      // fb_value = (uint32_t)fb_raw;

      /* Update last writable buffer size */
      audio_buf_writable_size_last = audio_buf_writable_size;

      /* Check feedback max / min */
      if (fb_value > fb_nom + AUDIO_FB_DELTA) {
        fb_value = fb_raw = fb_nom + AUDIO_FB_DELTA;
      } else if (fb_value < fb_nom - AUDIO_FB_DELTA) {
        fb_value = fb_raw = fb_nom - AUDIO_FB_DELTA;
      }

      /* Set 10.14 format feedback data */
      fb_data[0] = (fb_value & 0x0000FF00) >> 8;
      fb_data[1] = (fb_value & 0x00FF0000) >> 16;
      fb_data[2] = (fb_value & 0xFF000000) >> 24;
    }

    /* Transmit feedback only when the last one is transmitted */
    if (tx_flag == 0U) {
      /* Get FNSOF. Use volatile for fnsof_new since its address is mapped to a hardware register. */
      USB_OTG_GlobalTypeDef* USBx = USB_OTG_FS;
      uint32_t USBx_BASE = (uint32_t)USBx;
      uint32_t volatile fnsof_new = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> 8;

      if ((fnsof & 0x1) == (fnsof_new & 0x1)) {
        USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t*)fb_data, 3U);
        /* Block transmission until it's finished. */
        tx_flag = 1U;
      }
    }
  }

  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Sync
  *         handle Sync event called from usbd_audio_if.c
  * @param  pdev: device instance
  * @retval status
  */
void USBD_AUDIO_Sync(USBD_HandleTypeDef* pdev, AUDIO_OffsetTypeDef offset)
{
}

/**
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * USBD_AUDIO_IsoINIncomplete & USBD_AUDIO_IsoOutIncomplete are not 
 * enabled by default.
 * 
 * Go to Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
 * Fill in USBD_LL_IsoINIncomplete and USBD_LL_IsoOUTIncomplete with 
 * actual handler functions.
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */

/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
  USB_OTG_GlobalTypeDef* USBx = USB_OTG_FS;
  uint32_t USBx_BASE = (uint32_t)USBx;
  fnsof = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> 8;

  if (tx_flag == 1U) {
    tx_flag = 0U;
    USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
  }

  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef* pdev,
                                  uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  static uint8_t tmpbuf[1024];

  if (all_ready == 1U && epnum == AUDIO_OUT_EP) {
    uint32_t curr_length = USBD_GetRxCount(pdev, epnum);
    uint32_t curr_pos = haudio->wr_ptr;
    uint32_t rest = AUDIO_TOTAL_BUF_SIZE - curr_pos;

    /* Ignore strangely large packets */
    if (curr_length > AUDIO_OUT_PACKET) {
      curr_length = 0U;
    }

    /* Update circular audio buffer */
    // if (rest < curr_length) {
    //   if (rest > 0U) {
    //     memcpy(&haudio->buffer[curr_pos], tmpbuf, rest);
    //     curr_length -= rest;
    //   }
    //   if (curr_length > 0U) {
    //     memcpy(&haudio->buffer[0], &tmpbuf[rest], curr_length);
    //     haudio->wr_ptr = curr_length;
    //   }
    // } else {
    //   if (curr_length > 0U) {
    //     memcpy(&haudio->buffer[curr_pos], tmpbuf, curr_length);
    //     haudio->wr_ptr += curr_length;
    //   }
    // }
    
    uint32_t num_of_samples = 0U;
    uint32_t i, tmpbuf_ptr = 0U;
    if (haudio->bit_depth == 16U) {
      num_of_samples = curr_length / 4;
    } else {
      num_of_samples = curr_length / 6;
    }

    for (i = 0; i < num_of_samples; i++) {
      /* Copy one sample */
      if (haudio->bit_depth == 16U) {
        /* { 0: L_LOBYTE, 1: L_HIBYTE, 2: R_LOBYTE, 3: R_HIBYTE } */
        haudio->buffer[haudio->wr_ptr++] = (uint16_t)(tmpbuf[tmpbuf_ptr] << 8);
        haudio->buffer[haudio->wr_ptr++] = (uint16_t)(tmpbuf[tmpbuf_ptr + 1]);
        haudio->buffer[haudio->wr_ptr++] = (uint16_t)(tmpbuf[tmpbuf_ptr + 2] << 8);
        haudio->buffer[haudio->wr_ptr++] = (uint16_t)(tmpbuf[tmpbuf_ptr + 3]);
        tmpbuf_ptr += 4;
      } else {
        /* { 0: L_LOBYTE, 1: L_MDBYTE, 2: L_HIBYTE, 3: R_LOBYTE, 4: R_MDBYTE, 5: R_HIBYTE } */
        haudio->buffer[haudio->wr_ptr++] = *(uint16_t *)&tmpbuf[tmpbuf_ptr];
        haudio->buffer[haudio->wr_ptr++] = (uint16_t)(tmpbuf[tmpbuf_ptr + 2]);
        haudio->buffer[haudio->wr_ptr++] = *(uint16_t *)&tmpbuf[tmpbuf_ptr + 3];
        haudio->buffer[haudio->wr_ptr++] = (uint16_t)(tmpbuf[tmpbuf_ptr + 5]);
        tmpbuf_ptr += 6;
      }

      /* Rollback if reach end of buffer */
      if (haudio->wr_ptr >= AUDIO_TOTAL_BUF_SIZE) {
        haudio->wr_ptr = 0U;
      }
    }

    /* Start playing when half of the audio buffer is filled */
    if (haudio->wr_ptr >= AUDIO_TOTAL_BUF_SIZE / 2U) {
      if (haudio->offset == AUDIO_OFFSET_UNKNOWN && is_playing == 0U) {
        is_playing = 1U;

        if (haudio->rd_enable == 0U) {
          haudio->rd_enable = 1U;
          /* Set last writable buffer size to actual value. Note that rd_ptr is 0 now.  */
          audio_buf_writable_size_last = AUDIO_TOTAL_BUF_SIZE - haudio->wr_ptr;
        }

        haudio->offset = AUDIO_OFFSET_NONE;

        ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->AudioCmd(&haudio->buffer[0], AUDIO_TOTAL_BUF_SIZE / 2U, AUDIO_CMD_START);
      }
    }

    /* Rollback if reach end of buffer */
    // if (haudio->wr_ptr >= AUDIO_TOTAL_BUF_SIZE) {
    //   haudio->wr_ptr = 0U;
    // }

    USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, tmpbuf, AUDIO_OUT_PACKET);
  }

  return USBD_OK;
}

/**
 * @brief  AUDIO_Req_GetCurrent
 *         Handles the GET_CUR Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_MUTE: {
        /* Current mute state */
        uint8_t mute = 0;
        USBD_CtlSendData(pdev, &mute, 1);
      };
          break;
      case AUDIO_CONTROL_REQ_FU_VOL: {
        /* Current volume. See UAC Spec 1.0 p.77 */
        USBD_CtlSendData(pdev, (uint8_t*)&haudio->vol, 2);
      };
          break;
    }
  } else if ((req->bmRequest & 0x1f) == AUDIO_STREAMING_REQ) {
    if (HIBYTE(req->wValue) == AUDIO_STREAMING_REQ_FREQ_CTRL) {
      /* Current frequency */
      uint32_t freq __attribute__((aligned(4))) = haudio->freq;
      USBD_CtlSendData(pdev, (uint8_t*)&freq, 3);
    }
  }
}

/**
 * @brief  AUDIO_Req_GetMax
 *         Handles the GET_MAX Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetMax(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_VOL: {
        int16_t vol_max = USBD_AUDIO_VOL_MAX;
        USBD_CtlSendData(pdev, (uint8_t*)&vol_max, 2);
      };
          break;
    }
  }
}

/**
 * @brief  AUDIO_Req_GetMin
 *         Handles the GET_MIN Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetMin(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_VOL: {
        int16_t vol_min = USBD_AUDIO_VOL_MIN;
        USBD_CtlSendData(pdev, (uint8_t*)&vol_min, 2);
      };
          break;
    }
  }
}

/**
 * @brief  AUDIO_Req_GetRes
 *         Handles the GET_RES Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetRes(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_VOL: {
        int16_t vol_res = USBD_AUDIO_VOL_STEP;
        USBD_CtlSendData(pdev, (uint8_t*)&vol_res, 2);
      };
          break;
    }
  }
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  if (req->wLength) {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx(pdev,
                      haudio->control.data,
                      req->wLength);

    haudio->control.cmd = AUDIO_REQ_SET_CUR;          /* Set the request value */
    haudio->control.req_type = req->bmRequest & 0x1f; /* Set the request type. See UAC Spec 1.0 - 5.2.1 Request Layout */
    haudio->control.len = (uint8_t)req->wLength;      /* Set the request data length */
    haudio->control.unit = HIBYTE(req->wIndex);       /* Set the request target unit */
    haudio->control.cs = HIBYTE(req->wValue);         /* Set the request control selector (high byte) */
    haudio->control.cn = LOBYTE(req->wValue);         /* Set the request control number (low byte) */
  }
}

/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  if (haudio->control.cmd == AUDIO_REQ_SET_CUR) { /* In this driver, to simplify code, only SET_CUR request is managed */

    if (haudio->control.req_type == AUDIO_CONTROL_REQ) {
      switch (haudio->control.cs) {
        /* Mute Control */
        case AUDIO_CONTROL_REQ_FU_MUTE: {
          ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->MuteCtl(haudio->control.data[0]);
        };
            break;
        /* Volume Control */
        case AUDIO_CONTROL_REQ_FU_VOL: {
          int16_t vol = *(int16_t*)&haudio->control.data[0];
          haudio->vol = vol;
          ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->VolumeCtl(VOL_PERCENT(vol));
        };
            break;
      }

    } else if (haudio->control.req_type == AUDIO_STREAMING_REQ) {
      /* Frequency Control */
      if (haudio->control.cs == AUDIO_STREAMING_REQ_FREQ_CTRL) {
        uint32_t new_freq = *(uint32_t*)&haudio->control.data & 0x00ffffff;

        if (haudio->freq != new_freq) {
          haudio->freq = new_freq;
          AUDIO_OUT_Restart(pdev);
        }
      }
    }

    haudio->control.req_type = 0U;
    haudio->control.cs = 0U;
    haudio->control.cn = 0U;
    haudio->control.cmd = 0U;
    haudio->control.len = 0U;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef* pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}

/**
 * @brief  Stop playing and reset buffer pointers
 * @param  pdev: instance
 */
static void AUDIO_OUT_StopAndReset(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  all_ready = 0U;
  tx_flag = 1U;
  is_playing = 0U;
  audio_buf_writable_size_last = AUDIO_TOTAL_BUF_SIZE / 2U;

  haudio->offset = AUDIO_OFFSET_UNKNOWN;
  haudio->rd_enable = 0U;
  haudio->rd_ptr = 0U;
  haudio->wr_ptr = 0U;

  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
  USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);

  ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->DeInit(0);
}

/**
 * @brief  Restart playing with new parameters
 * @param  pdev: instance
 */
static void AUDIO_OUT_Restart(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  AUDIO_OUT_StopAndReset(pdev);

  switch (haudio->freq) {
    case 44100:
      fb_raw = fb_nom = fb_value = (44 << 22) + (1 << 22) / 10;
      break;
    case 48000:
      fb_raw = fb_nom = fb_value = 48 << 22;
      break;
    case 96000:
      fb_raw = fb_nom = fb_value = 96 << 22;
    default:
      fb_raw = fb_nom = fb_value = 96 << 22;
  }

  ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->Init(haudio->freq, VOL_PERCENT(haudio->vol), 0);

  tx_flag = 0U;
  all_ready = 1U;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t* USBD_AUDIO_GetDeviceQualifierDesc(uint16_t* length)
{
  *length = sizeof(USBD_AUDIO_DeviceQualifierDesc);
  return USBD_AUDIO_DeviceQualifierDesc;
}

/**
* @brief  USBD_AUDIO_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef* pdev,
                                     USBD_AUDIO_ItfTypeDef* fops)
{
  if (fops != NULL) {
    pdev->pUserData = fops;
  }
  return USBD_OK;
}

/* Convert USB volume value to % */
uint8_t VOL_PERCENT(int16_t vol)
{
  return (uint8_t)((vol - (int16_t)USBD_AUDIO_VOL_MIN) / (((int16_t)USBD_AUDIO_VOL_MAX - (int16_t)USBD_AUDIO_VOL_MIN) / 100));
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
