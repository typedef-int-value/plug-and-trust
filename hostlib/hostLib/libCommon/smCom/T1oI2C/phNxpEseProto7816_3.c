/*
 * Copyright 2012-2014,2018-2019 NXP
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "phNxpEseProto7816_3.h"
#include "phNxpEsePal_i2c.h"
#include "phEseTypes.h"
#include "log.h"

/**
 * \addtogroup ISO7816-3_protocol_lib
 *
 * @{ */

phNxpEseProto7816_t phNxpEseProto7816_3_Var;

/******************************************************************************
\section Introduction Introduction

 * This module provide the 7816-3 protocol level implementation for ESE
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_SendRawFrame(uint32_t data_len, uint8_t *p_data);
static bool_t phNxpEseProto7816_GetRawFrame(uint32_t *data_len, uint8_t **pp_data);
static uint16_t phNxpEseProto7816_ComputeCRC(unsigned char *p_buff, uint32_t offset,
        uint32_t length);
static bool_t phNxpEseProto7816_CheckCRC(uint32_t data_len, uint8_t *p_data);
static bool_t phNxpEseProto7816_SendSFrame(sFrameInfo_t sFrameData);
static bool_t phNxpEseProto7816_SendIframe(iFrameInfo_t iFrameData);
static bool_t phNxpEseProto7816_sendRframe(rFrameTypes_t rFrameType);
static bool_t phNxpEseProto7816_SetFirstIframeContxt(void);
static bool_t phNxpEseProto7816_SetNextIframeContxt(void);
static bool_t phNxpEseProro7816_SaveIframeData(uint8_t *p_data, uint32_t data_len);
static bool_t phNxpEseProro7816_SaveSframeData(uint8_t *p_data, uint32_t data_len);
static bool_t phNxpEseProto7816_ResetRecovery(void);
static bool_t phNxpEseProto7816_RecoverySteps(void);
static bool_t phNxpEseProto7816_DecodeFrame(uint8_t *p_data, uint32_t data_len);
static bool_t phNxpEseProto7816_ProcessResponse(void);
static bool_t TransceiveProcess(void);
static bool_t phNxpEseProto7816_RSync(void);

/******************************************************************************
 * Function         phNxpEseProto7816_SendRawFrame
 *
 * Description      This internal function is called send the data to ESE
 *
 * param[in]        uint32_t: number of bytes to be written
 * param[in]        uint8_t : data buffer
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_SendRawFrame(uint32_t data_len, uint8_t *p_data)
{
    ESESTATUS status = ESESTATUS_FAILED;
    status = phNxpEse_WriteFrame(data_len, p_data);
    if (ESESTATUS_SUCCESS != status)
    {
        LOG_E("%s Error phNxpEse_WriteFrame ", __FUNCTION__);
    }

    return (status == ESESTATUS_SUCCESS)?TRUE : FALSE;
}

/******************************************************************************
 * Function         phNxpEseProto7816_GetRawFrame
 *
 * Description      This internal function is called read the data from the ESE
 *
 * param[out]        uint32_t: number of bytes read
 * param[out]        uint8_t : Read data from ESE
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_GetRawFrame(uint32_t *data_len, uint8_t **pp_data)
{
    bool_t bStatus = FALSE;
    ESESTATUS status = ESESTATUS_FAILED;

    status = phNxpEse_read(data_len, pp_data);
    if (ESESTATUS_SUCCESS != status)
    {
        LOG_E("%s phNxpEse_read failed , status : 0x%x ", __FUNCTION__, status);
    }
    else
    {
        bStatus = TRUE;
    }
    return bStatus;
}

/******************************************************************************
 * Function         phNxpEseProto7816_ComputeCRC
 *
 * Description      This internal function is called compute the CRC
 *
 * param[in]        unsigned char: data buffer
 * param[in]        uint32_t : offset from which CRC to be calculated
 * param[in]        uint32_t : total length of frame
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
static uint16_t phNxpEseProto7816_ComputeCRC(unsigned char *p_buff, uint32_t offset,
        uint32_t length)
{
    uint16_t CRC = 0xFFFF, i = 0;
    uint8_t new_byte[2]={0};
    ENSURE_OR_GO_EXIT(p_buff != NULL);
    for (i = offset; i < length; i++)
    {
        CRC ^= p_buff[i];
        for (int bit = 8; bit > 0; --bit)
        {
            if ((CRC & 0x0001) == 0x0001)
            {
                CRC = (unsigned short)((CRC >> 1) ^ 0x8408);
            }
            else
            {
                CRC >>= 1;
            }
        }

    }
    CRC ^=0xFFFF;
    new_byte[0] = (CRC & 0xFF);
	new_byte[1]	= ((CRC >> 8) & 0xFF);
	CRC =new_byte[0] << 8 | new_byte[1];
exit:
    return (uint16_t) CRC;
}

/******************************************************************************
 * Function         phNxpEseProto7816_CheckCRC
 *
 * Description      This internal function is called compute and compare the
 *                  received CRC of the received data
 *
 * param[in]        uint32_t : frame length
 * param[in]        uint8_t: data buffer
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_CheckCRC(uint32_t data_len, uint8_t *p_data)
{
    bool_t status = FALSE;
    uint16_t calc_crc = 0;
    uint16_t recv_crc = 0;

    ENSURE_OR_GO_EXIT(p_data != NULL);
    status = TRUE;

    recv_crc = p_data[data_len - 2] <<8 | p_data[data_len - 1] ; //combine 2 byte CRC
    /* calculate the CRC after excluding Recieved CRC  */
    /* CRC calculation includes NAD byte, so offset is set to 0 */
    calc_crc = phNxpEseProto7816_ComputeCRC(p_data, 0, (data_len -2));
    LOG_D("Received CRC:0x%x Calculated CRC:0x%x ", recv_crc, calc_crc);
    if (recv_crc != calc_crc)
    {
        status = FALSE;
        LOG_E("%s CRC failed ", __FUNCTION__);
    }
exit:
    return status;
}

/******************************************************************************
 * Function         getMaxSupportedSendIFrameSize
 *
 * Description      This internal function is called to get the max supported
 *                  I-frame size
 *
 * param[in]        void
 *
 * Returns          IFSC_SIZE_SEND
 *
 ******************************************************************************/
uint8_t getMaxSupportedSendIFrameSize(void)
{
    return IFSC_SIZE_SEND ;
}

/******************************************************************************
 * Function         phNxpEseProto7816_SendSFrame
 *
 * Description      This internal function is called to send S-frame with all
 *                   updated 7816-3 headers
 *
 * param[in]        sFrameInfo_t: Info about S frame
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_SendSFrame(sFrameInfo_t sFrameData)
{
    bool_t status = ESESTATUS_FAILED;
    uint32_t frame_len = 0;
    uint8_t p_framebuff[7] = {0};
    uint8_t pcb_byte = 0;
    sFrameInfo_t sframeData = sFrameData;
    uint16_t calc_crc=0;
    /* This update is helpful in-case a R-NACK is transmitted from the MW */
    phNxpEseProto7816_3_Var.lastSentNonErrorframeType = SFRAME;
    switch(sframeData.sFrameType)
    {
        case RESYNCH_REQ:
            frame_len = (PH_PROTO_7816_HEADER_LEN + PH_PROTO_7816_CRC_LEN);
            p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] = 0;
#if defined(T1oI2C_GP)
            /* T =1 GP block format LEN field is of 2 byte*/
            p_framebuff[PH_PROPTO_7816_LEN_LOWER_OFFSET] = 0;
#endif
            p_framebuff[PH_PROPTO_7816_INF_BYTE_OFFSET] = 0x00;

            pcb_byte |= PH_PROTO_7816_S_BLOCK_REQ; /* PCB */
            pcb_byte |= PH_PROTO_7816_S_RESYNCH;
            break;
#if defined(T1oI2C_UM1225_SE050)
        case INTF_RESET_REQ:
            frame_len = (PH_PROTO_7816_HEADER_LEN + PH_PROTO_7816_CRC_LEN);
            p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] = 0;
            p_framebuff[PH_PROPTO_7816_INF_BYTE_OFFSET] = 0x00;

            pcb_byte |= PH_PROTO_7816_S_BLOCK_REQ; /* PCB */
            pcb_byte |= PH_PROTO_7816_S_RESET;
            break;
        case PROP_END_APDU_REQ:
            frame_len = (PH_PROTO_7816_HEADER_LEN + PH_PROTO_7816_CRC_LEN);
            p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] = 0;
            p_framebuff[PH_PROPTO_7816_INF_BYTE_OFFSET] = 0x00;

            pcb_byte |= PH_PROTO_7816_S_BLOCK_REQ; /* PCB */
            pcb_byte |= PH_PROTO_7816_S_END_OF_APDU;
            break;
        case ATR_REQ:
            frame_len = (PH_PROTO_7816_HEADER_LEN + PH_PROTO_7816_CRC_LEN);
            p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] = 0;
            p_framebuff[PH_PROPTO_7816_INF_BYTE_OFFSET] = 0x00;

            pcb_byte |= PH_PROTO_7816_S_BLOCK_REQ; /* PCB */
            pcb_byte |= PH_PROTO_7816_S_GET_ATR;
            break;
#endif
        case WTX_RSP:
            frame_len = (PH_PROTO_7816_HEADER_LEN + 1 + PH_PROTO_7816_CRC_LEN);
#if defined(T1oI2C_UM1225_SE050)
            /* T =1 UM1225 SE050 block format LEN field is of 2 byte*/
            p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] = 0x01;
#elif defined(T1oI2C_GP)
            /* T =1 GP block format LEN field is of 2 byte*/
            p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] = 0x00;
            p_framebuff[PH_PROPTO_7816_LEN_LOWER_OFFSET] = 0x01;
#endif
            p_framebuff[PH_PROPTO_7816_INF_BYTE_OFFSET] = 0x01;

            pcb_byte |= PH_PROTO_7816_S_BLOCK_RSP;
            pcb_byte |= PH_PROTO_7816_S_WTX;
            break;
        case CHIP_RESET_REQ:
            frame_len = (PH_PROTO_7816_HEADER_LEN + PH_PROTO_7816_CRC_LEN);
            p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] = 0;
#if defined(T1oI2C_GP)
            /* T =1 GP block format LEN field is of 2 byte*/
            p_framebuff[PH_PROPTO_7816_LEN_LOWER_OFFSET] = 0;
#endif
            p_framebuff[PH_PROPTO_7816_INF_BYTE_OFFSET] = 0x00;

            pcb_byte |= PH_PROTO_7816_S_BLOCK_REQ; /* PCB */
            pcb_byte |= PH_PROTO_7816_S_CHIP_RST;
            break;
#if defined(T1oI2C_GP)
        case SWR_REQ:
            frame_len = (PH_PROTO_7816_HEADER_LEN + PH_PROTO_7816_CRC_LEN);
            p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] = 0;
            p_framebuff[PH_PROPTO_7816_LEN_LOWER_OFFSET] = 0;
            p_framebuff[PH_PROPTO_7816_INF_BYTE_OFFSET] = 0x00;

            pcb_byte |= PH_PROTO_7816_S_BLOCK_REQ; /* PCB */
            pcb_byte |= PH_PROTO_7816_S_SWR;
            break;
        case RELEASE_REQ:
            frame_len = (PH_PROTO_7816_HEADER_LEN + PH_PROTO_7816_CRC_LEN);
            p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] = 0;
            p_framebuff[PH_PROPTO_7816_LEN_LOWER_OFFSET] = 0;
            p_framebuff[PH_PROPTO_7816_INF_BYTE_OFFSET] = 0x00;

            pcb_byte |= PH_PROTO_7816_S_BLOCK_REQ; /* PCB */
            pcb_byte |= PH_PROTO_7816_S_RELEASE;
            break;
        case CIP_REQ:
            frame_len = (PH_PROTO_7816_HEADER_LEN + PH_PROTO_7816_CRC_LEN);
            p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] = 0;
            p_framebuff[PH_PROPTO_7816_LEN_LOWER_OFFSET] = 0;
            p_framebuff[PH_PROPTO_7816_INF_BYTE_OFFSET] = 0x00;

            pcb_byte |= PH_PROTO_7816_S_BLOCK_REQ; /* PCB */
            pcb_byte |= PH_PROTO_7816_S_GET_CIP;
            break;
#endif
        default:
            LOG_E(" %s :Invalid S-block",__FUNCTION__);
            return status;
    }

    /* frame the packet */
    p_framebuff[PH_PROPTO_7816_NAD_OFFSET] = 0x5A; /* NAD Byte */
    p_framebuff[PH_PROPTO_7816_PCB_OFFSET] = pcb_byte; /* PCB */

    calc_crc = phNxpEseProto7816_ComputeCRC(p_framebuff, 0,(frame_len - 2));
    p_framebuff[frame_len - 2] = (calc_crc >> 8) & 0xFF;
    p_framebuff[frame_len - 1] = calc_crc & 0xFF;
    LOG_D("S-Frame PCB: %x ", p_framebuff[PH_PROPTO_7816_PCB_OFFSET]);
    status = phNxpEseProto7816_SendRawFrame(frame_len, p_framebuff);

    return status;
}

/******************************************************************************
 * Function         phNxpEseProto7816_sendRframe
 *
 * Description      This internal function is called to send R-frame with all
 *                   updated 7816-3 headers
 *
 * param[in]        sFrameInfo_t: Info about R frame
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
static  bool_t phNxpEseProto7816_sendRframe(rFrameTypes_t rFrameType)
{
    bool_t status = FALSE;
#if defined(T1oI2C_UM1225_SE050)
    uint8_t recv_ack[5]= {0x5A,0x80,0x00,0x00,0x00};
#elif defined(T1oI2C_GP)
    uint8_t recv_ack[6]= {0x5A,0x80,0x00,0x00,0x00,0x00};
#endif
    uint16_t calc_crc=0;
    if(RNACK == rFrameType) /* R-NACK */
    {
        recv_ack[PH_PROPTO_7816_PCB_OFFSET] = 0x82;
    }
    else /* R-ACK*/
    {
        /* This update is helpful in-case a R-NACK is transmitted from the MW */
        phNxpEseProto7816_3_Var.lastSentNonErrorframeType = RFRAME;
    }
    recv_ack[PH_PROPTO_7816_PCB_OFFSET] |=((phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.seqNo^1) << 4);
    LOG_D("%s recv_ack[PH_PROPTO_7816_PCB_OFFSET]:0x%x ", __FUNCTION__, recv_ack[PH_PROPTO_7816_PCB_OFFSET]);
    calc_crc = phNxpEseProto7816_ComputeCRC(recv_ack, 0x00, (sizeof(recv_ack) -2));

    recv_ack[(sizeof(recv_ack) -2)] = (calc_crc >> 8) & 0xFF;
    recv_ack[(sizeof(recv_ack) -1)] = calc_crc &0xFF ;
    status = phNxpEseProto7816_SendRawFrame(sizeof(recv_ack), recv_ack);
    return status;
}

/******************************************************************************
 * Function         phNxpEseProto7816_SendIframe
 *
 * Description      This internal function is called to send I-frame with all
 *                   updated 7816-3 headers
 *
 * param[in]        sFrameInfo_t: Info about I frame
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_SendIframe(iFrameInfo_t iFrameData)
{
    bool_t status = FALSE;
    uint32_t frame_len = 0;
    uint8_t p_framebuff[MAX_DATA_LEN];
    uint8_t pcb_byte = 0;
    uint16_t calc_crc = 0;
    if (0 == iFrameData.sendDataLen)
    {
        LOG_E("%s Line: [%d] I frame Len is 0, INVALID ",__FUNCTION__,__LINE__);
        return FALSE;
    }
    /* This update is helpful in-case a R-NACK is transmitted from the MW */
    phNxpEseProto7816_3_Var.lastSentNonErrorframeType = IFRAME;
    frame_len = (iFrameData.sendDataLen+ PH_PROTO_7816_HEADER_LEN + PH_PROTO_7816_CRC_LEN);

    /* frame the packet */
    p_framebuff[PH_PROPTO_7816_NAD_OFFSET] = SEND_PACKET_SOF; /* NAD Byte */

    if (iFrameData.isChained)
    {
        /* make B6 (M) bit high */
        pcb_byte |= PH_PROTO_7816_CHAINING;
    }

    /* Update the send seq no */
    pcb_byte |= (phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.seqNo << 6);

    /* store the pcb byte */
    p_framebuff[PH_PROPTO_7816_PCB_OFFSET] = pcb_byte;
#if defined(T1oI2C_UM1225_SE050)
    /* store I frame length */
    /* for T1oI2C_UM1225_SE050 LEN field is of 1 byte*/
    p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] =iFrameData.sendDataLen;
#elif defined(T1oI2C_GP)
    /* store I frame length */
    /* for T1oI2C_GP LEN field is of 2 byte*/
    p_framebuff[PH_PROPTO_7816_LEN_UPPER_OFFSET] =(((uint16_t)iFrameData.sendDataLen) >> 8 & 0xff);
    p_framebuff[PH_PROPTO_7816_LEN_LOWER_OFFSET] =(((uint16_t)iFrameData.sendDataLen) & 0xff);
#endif
    /* store I frame */
    phNxpEse_memcpy(&(p_framebuff[PH_PROPTO_7816_INF_BYTE_OFFSET]), iFrameData.p_data + iFrameData.dataOffset, iFrameData.sendDataLen);
    calc_crc = phNxpEseProto7816_ComputeCRC(p_framebuff, 0, (frame_len - 2));

    p_framebuff[frame_len - 2] = (calc_crc >> 8) & 0xff;
    p_framebuff[frame_len - 1] = calc_crc & 0xff;
    status = phNxpEseProto7816_SendRawFrame(frame_len, p_framebuff);

    return status;
}

/******************************************************************************
 * Function         phNxpEseProto7816_SetFirstIframeContxt
 *
 * Description      This internal function is called to set the context for next I-frame.
 *                  Not applicable for the first I-frame of the transceive
 *
 * param[in]        void
 *
 * Returns          Always return TRUE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_SetFirstIframeContxt(void)
{
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.dataOffset = 0;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType = IFRAME;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.seqNo = phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.seqNo ^ 1;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_IFRAME;
    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.pRsp->len = 0;
    if (phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.totalDataLen > phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.maxDataLen)
    {
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.isChained = TRUE;
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.sendDataLen = phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.maxDataLen;
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.totalDataLen = phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.totalDataLen -
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.maxDataLen;
    }
    else
    {
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.sendDataLen = phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.totalDataLen;
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.isChained = FALSE;
    }
    LOG_D("I-Frame Data Len: %ld Seq. no:%d ", phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.sendDataLen, phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.seqNo);
    return TRUE;
}

/******************************************************************************
 * Function         phNxpEseProto7816_SetNextIframeContxt
 *
 * Description      This internal function is called to set the context for next I-frame.
 *                  Not applicable for the first I-frame of the transceive
 *
 * param[in]        void
 *
 * Returns          Always return TRUE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_SetNextIframeContxt(void)
{
    /* Expecting to reach here only after first of chained I-frame is sent and before the last chained is sent */
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType = IFRAME;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_IFRAME;

    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.seqNo = phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.seqNo ^ 1;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.dataOffset = phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.dataOffset +
                                                                        phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.maxDataLen;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.p_data = phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.p_data;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.maxDataLen = phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.maxDataLen;

    //if  chained
    if (phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.totalDataLen >
            phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.maxDataLen)
    {
        LOG_D("%s Process Chained Frame ",__FUNCTION__);
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.isChained = TRUE;
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.sendDataLen = phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.maxDataLen;
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.totalDataLen = phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.totalDataLen -
                                                                                phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.maxDataLen;
    }
    else
    {
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.isChained = FALSE;
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.sendDataLen = phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.totalDataLen;
    }
    LOG_D("I-Frame Data Len: %ld ", phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.sendDataLen);
    return TRUE;
}

/******************************************************************************
 * Function         phNxpEseProro7816_SaveIframeData
 *
 * Description      This internal function is called to save recv I-frame data
 *
 * param[in]        uint8_t: data buffer
 * param[in]        uint32_t: buffer length
 *
 * Returns          Always return TRUE.
 *
 ******************************************************************************/
static bool_t phNxpEseProro7816_SaveIframeData(uint8_t *p_data, uint32_t data_len)
{
    uint32_t offset = 0;
    LOG_D("Data[0]=0x%x len=%ld Data[%ld]=0x%x Data[%ld]=0x%x ", p_data[0], data_len,data_len-1, p_data[data_len-2],p_data[data_len-1]);

    offset = phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.pRsp->len ;
    phNxpEse_memcpy((phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.pRsp->p_data + offset), p_data, data_len);
    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.pRsp->len += data_len;
    return TRUE;
}

/******************************************************************************
 * Function         phNxpEseProro7816_SaveSframeData
 *
 * Description      This internal function is called to save recv S-frame data
 *
 * param[in]        uint8_t: data buffer
 * param[in]        uint32_t: buffer length
 *
 * Returns          Always return TRUE.
 *
 ******************************************************************************/
static bool_t phNxpEseProro7816_SaveSframeData(uint8_t *p_data, uint32_t data_len)
{
    uint32_t offset = 0;
    LOG_D("Data[0]=0x%x len=%ld Data[%ld]=0x%x Data[%ld]=0x%x ", p_data[0], data_len,data_len-1, p_data[data_len-2],p_data[data_len-1]);

    phNxpEse_memcpy((phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.pRsp->p_data + offset), p_data, data_len);
    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.pRsp->len = data_len;
    return TRUE;
}

/******************************************************************************
 * Function         phNxpEseProto7816_ResetRecovery
 *
 * Description      This internal function is called to do reset the recovery pareameters
 *
 * param[in]        void
 *
 * Returns          Always return TRUE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_ResetRecovery(void)
{
    phNxpEseProto7816_3_Var.recoveryCounter = 0;
    return TRUE;
}

/******************************************************************************
 * Function         phNxpEseProto7816_RecoverySteps
 *
 * Description      This internal function is called when 7816-3 stack failed to recover
 *                  after PH_PROTO_7816_FRAME_RETRY_COUNT, and the interface has to be
 *                  recovered
 *
 * param[in]        void
 *
 * Returns          Always return TRUE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_RecoverySteps(void)
{
    if(phNxpEseProto7816_3_Var.recoveryCounter <= PH_PROTO_7816_FRAME_RETRY_COUNT)
    {
#if defined(T1oI2C_UM1225_SE050)
        phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType = INTF_RESET_REQ;
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = INTF_RESET_REQ;
        phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_INTF_RST;
#elif defined(T1oI2C_GP)
        phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType = SWR_REQ;
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = SWR_REQ;
        phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_SWR;
#endif
    }
    else
    { /* If recovery fails */
        phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
    }
    return TRUE;
}

/******************************************************************************
 * Function         phNxpEseProto7816_DecodeSFrameData
 *
 * Description      This internal function is to decode S-frame payload.
 *
 * param[in]        uint8_t; data buffer
 *
 * Returns          void
 *
 ******************************************************************************/
static void phNxpEseProto7816_DecodeSFrameData(uint8_t *p_data)
{
    uint8_t maxSframeLen = 0, frameOffset = 0;

    ENSURE_OR_GO_EXIT(p_data != NULL);
#if defined(T1oI2C_UM1225_SE050)
    frameOffset = PH_PROPTO_7816_LEN_UPPER_OFFSET;
#elif defined(T1oI2C_GP)
    /* current GP implementation support max payload of 0x00FE, so considering lower offset */
    frameOffset = PH_PROPTO_7816_LEN_LOWER_OFFSET;
#endif
    maxSframeLen = p_data[frameOffset] + frameOffset; /* to be in sync with offset which starts from index 0 */
    while(maxSframeLen > frameOffset)
    {
        frameOffset += 1; /* To get the Type (TLV) */
        LOG_D("%s frameoffset=%d value=0x%x ", __FUNCTION__, frameOffset, p_data[frameOffset]);
        frameOffset += p_data[frameOffset + 1]; /* Goto the end of current marker */

    }
exit:
    return;
}

/******************************************************************************
 * Function         phNxpEseProto7816_DecodeFrame
 *
 * Description      This internal function is used to
 *                  1. Identify the received frame
 *                  2. If the received frame is I-frame with expected sequence number, store it or else send R-NACK
                    3. If the received frame is R-frame,
                       3.1 R-ACK with expected seq. number: Send the next chained I-frame
                       3.2 R-ACK with different sequence number: Sebd the R-Nack
                       3.3 R-NACK: Re-send the last frame
                    4. If the received frame is S-frame, send back the correct S-frame response.
 *
 * param[in]        uint8_t : data buffer
 * param[in]        uint32_t : buffer length
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_DecodeFrame(uint8_t *p_data, uint32_t data_len)
{
    bool_t status = TRUE;
    uint8_t pcb;
    phNxpEseProto7816_PCB_bits_t pcb_bits;
    LOG_D("Retry Counter = %d ", phNxpEseProto7816_3_Var.recoveryCounter);

    ENSURE_OR_GO_EXIT(p_data != NULL);

    pcb = p_data[PH_PROPTO_7816_PCB_OFFSET];
    phNxpEse_memset(&pcb_bits, 0x00, sizeof(phNxpEseProto7816_PCB_bits_t));
    phNxpEse_memcpy(&pcb_bits, &pcb, sizeof(uint8_t));

    if (0x00 == pcb_bits.msb) /* I-FRAME decoded should come here */
    {
        LOG_D("%s I-Frame Received ", __FUNCTION__);
        phNxpEseProto7816_3_Var.wtx_counter = 0;
        phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdFrameType = IFRAME ;
        if (phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.seqNo != pcb_bits.bit7)       //   != pcb_bits->bit7)
        {
            LOG_D("%s I-Frame lastRcvdIframeInfo.seqNo:0x%x ", __FUNCTION__, pcb_bits.bit7);
            phNxpEseProto7816_ResetRecovery();
            phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.seqNo = 0x00;
            phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.seqNo |= pcb_bits.bit7;

            if (pcb_bits.bit6)
            {
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.isChained = TRUE;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType = RFRAME;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.RframeInfo.errCode = NO_ERROR ;
                phNxpEseProro7816_SaveIframeData(&p_data[PH_PROPTO_7816_INF_BYTE_OFFSET], data_len - PH_PROTO_7816_INF_FILED);
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_R_ACK ;
            }
            else
            {
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.isChained = FALSE;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                phNxpEseProro7816_SaveIframeData(&p_data[PH_PROPTO_7816_INF_BYTE_OFFSET], data_len - PH_PROTO_7816_INF_FILED);
            }
        }
        else
        {
            wait_ms(DELAY_ERROR_RECOVERY/1000);
            if(phNxpEseProto7816_3_Var.recoveryCounter < PH_PROTO_7816_FRAME_RETRY_COUNT)
            {
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType = RFRAME;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.RframeInfo.errCode= OTHER_ERROR ;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_R_NACK ;
                phNxpEseProto7816_3_Var.recoveryCounter++;
            }
            else
            {
                phNxpEseProto7816_RecoverySteps();
                phNxpEseProto7816_3_Var.recoveryCounter++;
            }
        }
    }
    else if ((0x01 == pcb_bits.msb) && (0x00 == pcb_bits.bit7)) /* R-FRAME decoded should come here */
    {
        LOG_D("%s R-Frame Received", __FUNCTION__);
        phNxpEseProto7816_3_Var.wtx_counter = 0;
        phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdFrameType = RFRAME;
        phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdRframeInfo.seqNo = 0; // = 0;
        phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdRframeInfo.seqNo |= pcb_bits.bit5;

        if ((pcb_bits.lsb == 0x00) && (pcb_bits.bit2 == 0x00))
        {
            phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdRframeInfo.errCode = NO_ERROR;
            phNxpEseProto7816_ResetRecovery();
            if(phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdRframeInfo.seqNo !=
                    phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.seqNo)
            {
                phNxpEseProto7816_SetNextIframeContxt();
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_IFRAME;
            }

        } /* Error handling 1 : Parity error */
        else if (((pcb_bits.lsb == 0x01) && (pcb_bits.bit2 == 0x00)) ||
            /* Error handling 2: Other indicated error */
            ((pcb_bits.lsb == 0x00) && (pcb_bits.bit2 == 0x01)))
        {
            wait_ms(DELAY_ERROR_RECOVERY/1000);
            if((pcb_bits.lsb == 0x00) && (pcb_bits.bit2 == 0x01))
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdRframeInfo.errCode = OTHER_ERROR;
            else
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdRframeInfo.errCode = PARITY_ERROR;
            if(phNxpEseProto7816_3_Var.recoveryCounter < PH_PROTO_7816_FRAME_RETRY_COUNT)
            {
                if(phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.FrameType == IFRAME)
                {
                    phNxpEse_memcpy(&phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx,
                        &phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx,
                            sizeof(phNxpEseProto7816_NextTx_Info_t));
                    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_IFRAME;
                    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType = IFRAME;
                }
                else if(phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.FrameType == RFRAME)
                {
                    /* Usecase to reach the below case:
                    I-frame sent first, followed by R-NACK and we receive a R-NACK with
                    last sent I-frame sequence number*/
                    if((phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdRframeInfo.seqNo ==
                    phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.seqNo) &&
                        (phNxpEseProto7816_3_Var.lastSentNonErrorframeType == IFRAME))
                    {
                        phNxpEse_memcpy(&phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx,
                        &phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx,
                            sizeof(phNxpEseProto7816_NextTx_Info_t));
                        phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_IFRAME;
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType = IFRAME;
                    }
                    /* Usecase to reach the below case:
                    R-frame sent first, followed by R-NACK and we receive a R-NACK with
                    next expected I-frame sequence number*/
                    else if((phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdRframeInfo.seqNo !=
                    phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.seqNo) &&
                        (phNxpEseProto7816_3_Var.lastSentNonErrorframeType == RFRAME))
                    {
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType = RFRAME;
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.RframeInfo.errCode = NO_ERROR ;
                        phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_R_ACK ;
                    }
                    /* Usecase to reach the below case:
                    I-frame sent first, followed by R-NACK and we receive a R-NACK with
                    next expected I-frame sequence number + all the other unexpected scenarios */
                    else
                    {
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= RFRAME;
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.RframeInfo.errCode = OTHER_ERROR ;
                        phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_R_NACK ;
                    }
                }
                else if(phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.FrameType == SFRAME)
                {
                    /* Copy the last S frame sent */
                    phNxpEse_memcpy(&phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx,
                        &phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx,
                            sizeof(phNxpEseProto7816_NextTx_Info_t));
                }
                phNxpEseProto7816_3_Var.recoveryCounter++;
            }
            else
            {
                phNxpEseProto7816_RecoverySteps();
                phNxpEseProto7816_3_Var.recoveryCounter++;
            }
            //resend previously send I frame
        }
        /* Error handling 3 */
        else if ((pcb_bits.lsb == 0x01) && (pcb_bits.bit2 == 0x01))
        {
            wait_ms(DELAY_ERROR_RECOVERY/1000);
            if(phNxpEseProto7816_3_Var.recoveryCounter < PH_PROTO_7816_FRAME_RETRY_COUNT)
            {
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdRframeInfo.errCode = SOF_MISSED_ERROR;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx = phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx;
                phNxpEseProto7816_3_Var.recoveryCounter++;
            }
            else
            {
                phNxpEseProto7816_RecoverySteps();
                phNxpEseProto7816_3_Var.recoveryCounter++;
            }
        }
    }
    else if ((0x01 == pcb_bits.msb) && (0x01 == pcb_bits.bit7)) /* S-FRAME decoded should come here */
    {
        LOG_D("%s S-Frame Received ", __FUNCTION__);
        int32_t frameType = (int32_t)(pcb & 0x3F); /*discard upper 2 bits */
        phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdFrameType = SFRAME;
        if(frameType!=WTX_REQ)
        {
            phNxpEseProto7816_3_Var.wtx_counter = 0;
        }
        switch(frameType)
        {
            case RESYNCH_RSP:
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType = RESYNCH_RSP;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= UNKNOWN;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                break;
            case IFSC_RES:
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType = IFSC_RES;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= UNKNOWN;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE ;
                break;
            case ABORT_RES:
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType = ABORT_RES;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= UNKNOWN;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE ;
                break;
            case WTX_REQ:
                phNxpEseProto7816_3_Var.wtx_counter++;
                LOG_D("%s Wtx_counter value - %lu ", __FUNCTION__, phNxpEseProto7816_3_Var.wtx_counter);
                LOG_D("%s Wtx_counter wtx_counter_limit - %lu ", __FUNCTION__, phNxpEseProto7816_3_Var.wtx_counter_limit);
                /* Previous sent frame is some S-frame but not WTX response S-frame */
                if(phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.SframeInfo.sFrameType != WTX_RSP &&
                    phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.FrameType == SFRAME)
                {   /* Goto recovery if it keep coming here for more than recovery counter max. value */
                    if(phNxpEseProto7816_3_Var.recoveryCounter < PH_PROTO_7816_FRAME_RETRY_COUNT)
                    {   /* Re-transmitting the previous sent S-frame */
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx = phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx;
                        phNxpEseProto7816_3_Var.recoveryCounter++;
                    }
                    else
                    {
                        phNxpEseProto7816_RecoverySteps();
                        phNxpEseProto7816_3_Var.recoveryCounter++;
                    }
                }
                else
                {   /* Checking for WTX counter with max. allowed WTX count */
                    if(phNxpEseProto7816_3_Var.wtx_counter == phNxpEseProto7816_3_Var.wtx_counter_limit)
                    {
#if defined(T1oI2C_UM1225_SE050)
                        phNxpEseProto7816_3_Var.wtx_counter = 0;
                        phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType = INTF_RESET_REQ;
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = INTF_RESET_REQ;
                        phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_INTF_RST;
                        LOG_E("%s Interface Reset to eSE wtx count reached!!! ", __FUNCTION__);
#elif defined(T1oI2C_GP)
                        phNxpEseProto7816_3_Var.wtx_counter = 0;
                        phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType = SWR_REQ;
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = SWR_REQ;
                        phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_SWR;
                        LOG_E("%s Software Reset to eSE wtx count reached!!! ", __FUNCTION__);
#endif
                    }
                    else
                    {
                        wait_ms(DELAY_ERROR_RECOVERY/1000);
                        phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType = WTX_REQ;
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
                        phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = WTX_RSP;
                        phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_WTX_RSP ;
                    }
                }
                break;
#if defined(T1oI2C_UM1225_SE050)
            case INTF_RESET_RSP:
                if(p_data[PH_PROPTO_7816_FRAME_LENGTH_OFFSET] > 0)
                    phNxpEseProto7816_DecodeSFrameData(p_data);
                phNxpEseProro7816_SaveSframeData(&p_data[PH_PROPTO_7816_INF_BYTE_OFFSET], data_len - PH_PROTO_7816_INF_FILED);
                if(phNxpEseProto7816_3_Var.recoveryCounter > PH_PROTO_7816_FRAME_RETRY_COUNT){
                    /*Max recovery counter reached, send failure to APDU layer  */
                    LOG_E("%s Max retry count reached!!! ", __FUNCTION__);
                    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                    status = FALSE;
                }
                else{
                    phNxpEseProto7816_ResetProtoParams();
                    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType= INTF_RESET_RSP;
                    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= UNKNOWN;
                    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                }
                break;
            case PROP_END_APDU_RSP:
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType= PROP_END_APDU_RSP;
                if(p_data[PH_PROPTO_7816_FRAME_LENGTH_OFFSET] > 0)
                    phNxpEseProto7816_DecodeSFrameData(p_data);
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= UNKNOWN;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                break;
            case ATR_RES:
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType= ATR_RES;
                if(p_data[PH_PROPTO_7816_FRAME_LENGTH_OFFSET] > 0)
                    phNxpEseProto7816_DecodeSFrameData(p_data);
                phNxpEseProro7816_SaveSframeData(&p_data[PH_PROPTO_7816_INF_BYTE_OFFSET], data_len - PH_PROTO_7816_INF_FILED);
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= UNKNOWN;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                break;
#endif
            case CHIP_RESET_RES:
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType= CHIP_RESET_RES;
                if(p_data[PH_PROPTO_7816_FRAME_LENGTH_OFFSET] > 0)
                    phNxpEseProto7816_DecodeSFrameData(p_data);
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= UNKNOWN;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                break;
#if defined(T1oI2C_GP)
            case SWR_RSP:
                if(p_data[PH_PROPTO_7816_FRAME_LENGTH_OFFSET] > 0)
                    phNxpEseProto7816_DecodeSFrameData(p_data);
                if(phNxpEseProto7816_3_Var.recoveryCounter > PH_PROTO_7816_FRAME_RETRY_COUNT){
                    /*Max recovery counter reached, send failure to APDU layer  */
                    LOG_E("%s Max retry count reached!!! ", __FUNCTION__);
                    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                    status = FALSE;
                }
                else{
                    phNxpEseProto7816_ResetProtoParams();
                    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType= SWR_RSP;
                    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= UNKNOWN;
                    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                }
                break;
            case RELEASE_RES:
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType= RELEASE_RES;
                if(p_data[PH_PROPTO_7816_FRAME_LENGTH_OFFSET] > 0)
                    phNxpEseProto7816_DecodeSFrameData(p_data);
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= UNKNOWN;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                break;
            case CIP_RES:
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.sFrameType= CIP_RES;
                if(p_data[PH_PROPTO_7816_FRAME_LENGTH_OFFSET] > 0)
                    phNxpEseProto7816_DecodeSFrameData(p_data);
                phNxpEseProro7816_SaveSframeData(&p_data[PH_PROPTO_7816_INF_BYTE_OFFSET], data_len - PH_PROTO_7816_INF_FILED);
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= UNKNOWN;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                break;
#endif
            default:
                LOG_E("%s Wrong S-Frame Received ", __FUNCTION__);
                break;
        }
    }
    else
    {
        LOG_E("%s Wrong-Frame Received ", __FUNCTION__);
    }
exit:
    return status ;
}

/******************************************************************************
 * Function         phNxpEseProto7816_ProcessResponse
 *
 * Description      This internal function is used to
 *                  1. Check the CRC
 *                  2. Initiate decoding of received frame of data.
 *
 * param[in]        void
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_ProcessResponse(void)
{
    uint32_t data_len = 0;
    uint8_t *p_data = NULL;
    bool_t status = FALSE;
    bool_t checkCrcPass = TRUE;
    status = phNxpEseProto7816_GetRawFrame(&data_len, &p_data);
    LOG_D("%s p_data ----> %p len ----> 0x%lx ", __FUNCTION__,p_data, data_len);
    if(TRUE == status)
    {
        /* Resetting the timeout counter */
        phNxpEseProto7816_3_Var.timeoutCounter = PH_PROTO_7816_VALUE_ZERO;
        /* CRC check followed */
        checkCrcPass = phNxpEseProto7816_CheckCRC(data_len, p_data);
        if(checkCrcPass == TRUE)
        {
            /* Resetting the RNACK retry counter */
            phNxpEseProto7816_3_Var.rnack_retry_counter = PH_PROTO_7816_VALUE_ZERO;
            status = phNxpEseProto7816_DecodeFrame(p_data, data_len);
        }
        else
        {
            LOG_E("%s CRC Check failed ", __FUNCTION__);
            if(phNxpEseProto7816_3_Var.rnack_retry_counter < phNxpEseProto7816_3_Var.rnack_retry_limit)
            {
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdFrameType = INVALID ;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= RFRAME;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.RframeInfo.errCode = PARITY_ERROR ;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.RframeInfo.seqNo =(!phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.seqNo) << 4;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_R_NACK ;
                phNxpEseProto7816_3_Var.rnack_retry_counter++;
            }
            else
            {
                phNxpEseProto7816_3_Var.rnack_retry_counter = PH_PROTO_7816_VALUE_ZERO;
                /* Re-transmission failed completely, Going to exit */
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                phNxpEseProto7816_3_Var.timeoutCounter = PH_PROTO_7816_VALUE_ZERO;
                status = FALSE;
            }
        }
    }
    else
    {
        LOG_E("%s phNxpEseProto7816_GetRawFrame failed ", __FUNCTION__);
        if((SFRAME == phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.FrameType) &&
                ((WTX_RSP == phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.SframeInfo.sFrameType) ||
                 (RESYNCH_RSP == phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.SframeInfo.sFrameType)))
        {
            if(phNxpEseProto7816_3_Var.rnack_retry_counter < phNxpEseProto7816_3_Var.rnack_retry_limit)
            {
                phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdFrameType = INVALID ;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= RFRAME;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.RframeInfo.errCode = OTHER_ERROR ;
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.RframeInfo.seqNo =(!phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.seqNo) << 4;
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_R_NACK ;
                phNxpEseProto7816_3_Var.rnack_retry_counter++;
            }
            else
            {
                phNxpEseProto7816_3_Var.rnack_retry_counter = PH_PROTO_7816_VALUE_ZERO;
                /* Re-transmission failed completely, Going to exit */
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                phNxpEseProto7816_3_Var.timeoutCounter = PH_PROTO_7816_VALUE_ZERO;
            }
        }
        else
        {
            wait_ms(DELAY_ERROR_RECOVERY/1000);
            /* re transmit the frame */
            if(phNxpEseProto7816_3_Var.timeoutCounter < PH_PROTO_7816_TIMEOUT_RETRY_COUNT)
            {
                phNxpEseProto7816_3_Var.timeoutCounter++;
                LOG_E("%s re-transmitting the previous frame ", __FUNCTION__);
                phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx = phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx ;
            }
            else
            {
                /* Re-transmission failed completely, Going to exit */
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                phNxpEseProto7816_3_Var.timeoutCounter = PH_PROTO_7816_VALUE_ZERO;
            }
        }
    }
    return status;
}

/******************************************************************************
 * Function         TransceiveProcess
 *
 * Description      This internal function is used to
 *                  1. Send the raw data received from application after computing CRC
 *                  2. Receive the the response data from ESE, decode, process and
 *                     store the data.
 *
 * param[in]        void
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
static bool_t TransceiveProcess(void)
{
    bool_t status = FALSE;
    sFrameInfo_t sFrameInfo;

    sFrameInfo.sFrameType = INVALID_REQ_RES;
    sFrameInfo.pRsp = NULL;

    while(phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState != IDLE_STATE)
    {
        LOG_D("%s nextTransceiveState %x ", __FUNCTION__, phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState);
        switch(phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState)
        {
            case SEND_IFRAME:
                status = phNxpEseProto7816_SendIframe(phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo);
                break;
            case SEND_R_ACK:
                status = phNxpEseProto7816_sendRframe(RACK);
                break;
            case SEND_R_NACK:
                status = phNxpEseProto7816_sendRframe(RNACK);
                break;
            case SEND_S_RSYNC:
                sFrameInfo.sFrameType = RESYNCH_REQ;
                status = phNxpEseProto7816_SendSFrame(sFrameInfo);
                break;
            case SEND_S_WTX_RSP:
                sFrameInfo.sFrameType = WTX_RSP;
                status = phNxpEseProto7816_SendSFrame(sFrameInfo);
                break;
            case SEND_S_CHIP_RST:
                sFrameInfo.sFrameType = CHIP_RESET_REQ;
                status = phNxpEseProto7816_SendSFrame(sFrameInfo);
                break;
#if defined(T1oI2C_UM1225_SE050)
            case SEND_S_INTF_RST:
                sFrameInfo.sFrameType = INTF_RESET_REQ;
                status = phNxpEseProto7816_SendSFrame(sFrameInfo);
                break;
            case SEND_S_EOS:
                sFrameInfo.sFrameType = PROP_END_APDU_REQ;
                status = phNxpEseProto7816_SendSFrame(sFrameInfo);
                break;
            case SEND_S_ATR:
                sFrameInfo.sFrameType = ATR_REQ;
                status = phNxpEseProto7816_SendSFrame(sFrameInfo);
                break;
#elif defined(T1oI2C_GP)
            case SEND_S_CIP:
                sFrameInfo.sFrameType = CIP_REQ;
                status = phNxpEseProto7816_SendSFrame(sFrameInfo);
                break;
            case SEND_S_SWR:
                sFrameInfo.sFrameType = SWR_REQ;
                status = phNxpEseProto7816_SendSFrame(sFrameInfo);
                break;
            case SEND_S_RELEASE:
                sFrameInfo.sFrameType = RELEASE_REQ;
                status = phNxpEseProto7816_SendSFrame(sFrameInfo);
                break;
#else
#error Either T1oI2C_UM1225_SE050 or T1oI2C_GP must be defined.
#endif
            default:
                phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
                break;
        }
        if(TRUE == status)
        {
            phNxpEse_memcpy(&phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx,
                &phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx,
                    sizeof(phNxpEseProto7816_NextTx_Info_t));
            status = phNxpEseProto7816_ProcessResponse();
        }
        else
        {
            LOG_E("%s Transceive send failed, going to recovery! ", __FUNCTION__);
            phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
        }
    };
    return status;
}

/******************************************************************************
 * Function         phNxpEseProto7816_Transceive
 *
 * Description      This function is used to
 *                  1. Send the raw data received from application after computing CRC
 *                  2. Receive the the response data from ESE, decode, process and
 *                     store the data.
 *                  3. Get the final complete data and sent back to application
 *
 * param[in]        phNxpEse_data: Command to ESE C-APDU
 * param[out]       phNxpEse_data: Response from ESE R-APDU
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
bool_t phNxpEseProto7816_Transceive(phNxpEse_data *pCmd, phNxpEse_data *pRsp)
{
    bool_t status = FALSE;
    uint32_t reqDataLen = 0;
    LOG_D("Enter %s  ", __FUNCTION__);
    if((NULL == pCmd) || (NULL == pRsp) ||
            (phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState != PH_NXP_ESE_PROTO_7816_IDLE))
        return status;
    reqDataLen = pRsp->len;
    /* Updating the transceive information to the protocol stack */
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_TRANSCEIVE;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.p_data = pCmd->p_data;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.totalDataLen = pCmd->len;
    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.pRsp = pRsp;
    LOG_D("Transceive data ptr 0x%p len:%ld ", pCmd->p_data, pCmd->len);
    phNxpEseProto7816_SetFirstIframeContxt();
    status = TransceiveProcess();
    if(FALSE == status)
    {
        /* ESE hard reset to be done */
        LOG_E("%s Transceive failed, hard reset to proceed ",__FUNCTION__);
    }
    else if(pRsp->len > reqDataLen )
    {
        LOG_W("Need '%d' bytes. Got '%d' to copy.", pRsp->len, reqDataLen);
        pRsp->len = 0;
        status = FALSE;
    }
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_IDLE;
    return status;
}

/******************************************************************************
 * Function         phNxpEseProto7816_RSync
 *
 * Description      This function is used to send the RSync command
 *
 * param[in]        void
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
static bool_t phNxpEseProto7816_RSync(void)
{
    bool_t status = FALSE;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_TRANSCEIVE;
    /* send the end of session s-frame */
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = RESYNCH_REQ;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_RSYNC;
    status = TransceiveProcess();
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_IDLE;
    return status;
}

/******************************************************************************
 * Function         phNxpEseProto7816_ResetProtoParams
 *
 * Description      This function is used to reset the 7816 protocol stack instance
 *
 * param[in]        void
 *
 * Returns          Always return TRUE.
 *
 ******************************************************************************/
bool_t phNxpEseProto7816_ResetProtoParams(void)
{
    unsigned long int tmpWTXCountlimit = PH_PROTO_7816_VALUE_ZERO;
    unsigned long int tmpRNACKCountlimit = PH_PROTO_7816_VALUE_ZERO;
    tmpWTXCountlimit = phNxpEseProto7816_3_Var.wtx_counter_limit;
    tmpRNACKCountlimit = phNxpEseProto7816_3_Var.rnack_retry_limit;
    phNxpEse_memset(&phNxpEseProto7816_3_Var, PH_PROTO_7816_VALUE_ZERO, sizeof(phNxpEseProto7816_t));
    phNxpEseProto7816_3_Var.wtx_counter_limit = tmpWTXCountlimit;
    phNxpEseProto7816_3_Var.rnack_retry_limit = tmpRNACKCountlimit;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_IDLE;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = IDLE_STATE;
    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdFrameType = INVALID;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType = INVALID;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.maxDataLen = IFSC_SIZE_SEND;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.p_data = NULL;
    phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.FrameType = INVALID;
    phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.maxDataLen = IFSC_SIZE_SEND;
    phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.p_data = NULL;
    /* Initialized with sequence number of the last I-frame sent */
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.seqNo = PH_PROTO_7816_VALUE_ONE;
    /* Initialized with sequence number of the last I-frame received */
    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.seqNo = PH_PROTO_7816_VALUE_ONE;
    /* Initialized with sequence number of the last I-frame received */
    phNxpEseProto7816_3_Var.phNxpEseLastTx_Cntx.IframeInfo.seqNo = PH_PROTO_7816_VALUE_ONE;
    phNxpEseProto7816_3_Var.recoveryCounter = PH_PROTO_7816_VALUE_ZERO;
    phNxpEseProto7816_3_Var.timeoutCounter = PH_PROTO_7816_VALUE_ZERO;
    phNxpEseProto7816_3_Var.wtx_counter = PH_PROTO_7816_VALUE_ZERO;
    /* This update is helpful in-case a R-NACK is transmitted from the MW */
    phNxpEseProto7816_3_Var.lastSentNonErrorframeType = UNKNOWN;
    phNxpEseProto7816_3_Var.rnack_retry_counter = PH_PROTO_7816_VALUE_ZERO;
    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdIframeInfo.pRsp = NULL;
    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.pRsp = NULL;
    return TRUE;
}


/******************************************************************************
 * Function         phNxpEseProto7816_Reset
 *
 * Description      This function is used to reset the 7816 protocol stack instance
 *
 * param[in]        void
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
bool_t phNxpEseProto7816_Reset(void)
{
    bool_t status = FALSE;
    /* Resetting host protocol instance */
    status = phNxpEseProto7816_ResetProtoParams();
    /* Resynchronising ESE protocol instance */
    //status = phNxpEseProto7816_RSync();
    return status;
}

/******************************************************************************
 * Function         phNxpEseProto7816_Open
 *
 * Description      This function is used to open the 7816 protocol stack instance
 *
 * param[in]        phNxpEseProto7816InitParam_t: ESE communication mode
 * param[out]       phNxpEse_data: ATR Response from ESE
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
bool_t phNxpEseProto7816_Open(phNxpEseProto7816InitParam_t initParam, phNxpEse_data *AtrRsp)
{
    bool_t status = FALSE;
    status = phNxpEseProto7816_ResetProtoParams();
    LOG_D("%s: First open completed", __FUNCTION__);
    /* Update WTX max. limit */
    phNxpEseProto7816_3_Var.wtx_counter_limit = initParam.wtx_counter_limit;
    phNxpEseProto7816_3_Var.rnack_retry_limit = initParam.rnack_retry_limit;
    if(initParam.interfaceReset) /* Do interface reset */
    {
        /*After power ON , initialization state takes 5ms after which slave enters active
        state where slave can exchange data with the master */
        wait_ms(WAKE_UP_DELAY_MS);
#if defined(T1oI2C_UM1225_SE050)
        /* Interface Reset respond with ATR*/
        status = phNxpEseProto7816_IntfReset(AtrRsp);
#elif defined(T1oI2C_GP)
        /* For GP soft reset does not respond with CIP so master should send CIP req. seperatly  */
        status = phNxpEseProto7816_SoftReset();
        if(status == TRUE)
        {
            status = phNxpEseProto7816_GetCip(AtrRsp);
        }
#endif
    }
    else /* Do R-Sync */
    {
        status = phNxpEseProto7816_RSync();
    }
    return status;
}

/******************************************************************************
 * Function         phNxpEseProto7816_Close
 *
 * Description      This function is used to close the 7816 protocol stack instance
 *
 * param[in]        void
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
bool_t phNxpEseProto7816_Close(void)
{
    bool_t status = FALSE;
    if(phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState != PH_NXP_ESE_PROTO_7816_IDLE)
        return status;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_DEINIT;
    phNxpEseProto7816_3_Var.recoveryCounter = 0;
    phNxpEseProto7816_3_Var.wtx_counter = 0;
#if defined(T1oI2C_UM1225_SE050)
    /* send the end of session s-frame */
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = PROP_END_APDU_REQ;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_EOS;
#elif defined(T1oI2C_GP)
    /* send the release request s-frame */
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = RELEASE_REQ;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_RELEASE;
#endif
    status = TransceiveProcess();
    if(FALSE == status)
    {
        /* reset all the structures */
        LOG_E("%s TransceiveProcess failed  ", __FUNCTION__);
    }
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_IDLE;
    return status;
}

#if defined(T1oI2C_UM1225_SE050)
/******************************************************************************
 * Function         phNxpEseProto7816_IntfReset
 *
 * Description      This function is used to reset just the current interface
                    and get the ATR response on successful reset
 *
 * param[in]        phNxpEse_data: ATR response from ESE
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
bool_t phNxpEseProto7816_IntfReset(phNxpEse_data *AtrRsp)
{
    bool_t status = FALSE;

    ENSURE_OR_GO_EXIT(AtrRsp != NULL);
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_TRANSCEIVE;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = INTF_RESET_REQ;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_INTF_RST;
    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.pRsp = AtrRsp;
    phNxpEse_clearReadBuffer();
    status = TransceiveProcess();
    if(FALSE == status)
    {
        /* reset all the structures */
        LOG_E("%s TransceiveProcess failed  ", __FUNCTION__);
    }

    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_IDLE;
exit:
    return status ;
}
#endif

#if defined(T1oI2C_GP)
/******************************************************************************
 * Function         phNxpEseProto7816_SoftReset
 *
 * Description      This function is used only for T1oI2C GP to reset just the current interface
 *
 * param[in]        void
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
bool_t phNxpEseProto7816_SoftReset(void)
{
    bool_t status = FALSE;

    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_TRANSCEIVE;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = SWR_REQ;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_SWR;
    phNxpEse_clearReadBuffer();
    status = TransceiveProcess();
    if(FALSE == status)
    {
        /* reset all the structures */
        LOG_E("%s TransceiveProcess failed  ", __FUNCTION__);
    }

    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_IDLE;
    return status ;
}
#endif
/******************************************************************************
 * Function         phNxpEseProto7816_SetIfscSize
 *
 * Description      This function is used to set the max T=1 data send size
 *
 * param[in]        uint16_t IFSC_Size
 *
 * Returns          Always return TRUE (1).
 *
 ******************************************************************************/
bool_t phNxpEseProto7816_SetIfscSize(uint16_t IFSC_Size)
{
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.IframeInfo.maxDataLen = IFSC_Size;
    return TRUE;
}

/******************************************************************************
 * Function         phNxpEseProto7816_ChipReset
 *
 * Description      This function is used to reset just the current interface
 *
 * param[in]        void
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
bool_t phNxpEseProto7816_ChipReset(void)
{
    bool_t status = FALSE;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_TRANSCEIVE;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = CHIP_RESET_REQ;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_CHIP_RST;
    status = TransceiveProcess();
    if(FALSE == status)
    {
        /* reset all the structures */
        LOG_E("%s TransceiveProcess failed  ", __FUNCTION__);
    }
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_IDLE;
    return status ;
}
#if defined(T1oI2C_UM1225_SE050)
/******************************************************************************
 * Function         phNxpEseProto7816_GetAtr
 *
 * Description      This function is used to reset just the current interface
 *
 * param[in]        phNxpEse_data : ATR response from ESE
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
bool_t phNxpEseProto7816_GetAtr(phNxpEse_data *pRsp)
{
    bool_t status = FALSE;

    ENSURE_OR_GO_EXIT(pRsp != NULL);
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_TRANSCEIVE;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = ATR_REQ;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_ATR;
    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.pRsp = pRsp;
    status = TransceiveProcess();
    if(FALSE == status)
    {
        /* reset all the structures */
        LOG_E("%s TransceiveProcess failed  ", __FUNCTION__);
    }
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_IDLE;
exit:
    return status ;
}
#endif

#if defined(T1oI2C_GP)
/******************************************************************************
 * Function         phNxpEseProto7816_GetCip
 *
 * Description      This function is used only by T1oI2c GP to get CIP response
 *
 * param[in]        phNxpEse_data : CIP response from ESE
 *
 * Returns          On success return TRUE or else FALSE.
 *
 ******************************************************************************/
bool_t phNxpEseProto7816_GetCip(phNxpEse_data *pRsp)
{
    bool_t status = FALSE;

    ENSURE_OR_GO_EXIT(pRsp != NULL);
    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_TRANSCEIVE;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.FrameType= SFRAME;
    phNxpEseProto7816_3_Var.phNxpEseNextTx_Cntx.SframeInfo.sFrameType = CIP_REQ;
    phNxpEseProto7816_3_Var.phNxpEseProto7816_nextTransceiveState = SEND_S_CIP;
    phNxpEseProto7816_3_Var.phNxpEseRx_Cntx.lastRcvdSframeInfo.pRsp = pRsp;
    status = TransceiveProcess();
    if(FALSE == status)
    {
        /* reset all the structures */
        LOG_E("%s TransceiveProcess failed  ", __FUNCTION__);
    }

    phNxpEseProto7816_3_Var.phNxpEseProto7816_CurrentState = PH_NXP_ESE_PROTO_7816_IDLE;
exit:
    return status ;
}
#endif
/** @} */
