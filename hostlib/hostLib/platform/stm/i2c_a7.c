/*
 *
 * Copyright 2017-2020,2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @par Description
 * MCIMX6UL-EVK / MCIMX8M-EVK board specific & Generic i2c code
 * @par History
 *
 **/
#include "i2c_a7.h"
#include "i2c.h"

// #define NX_LOG_ENABLE_SMCOM_DEBUG 1

#include "nxLog_smCom.h"

/**
 * Opens the communication channel to I2C device
 */
i2c_error_t axI2CInit(void **conn_ctx, const char *pDevName)
{
  return I2C_OK;
}

/**
 * Closes the communication channel to I2C device
 */
void axI2CTerm(void *conn_ctx, int mode)
{
}

#if defined(SCI2C)
/**
 * Write a single byte to the slave device.
 * In the context of the SCI2C protocol, this command is only invoked
 * to trigger a wake-up of the attached secure module. As such this
 * wakeup command 'wakes' the device, but does not receive a valid response.
 * \note \par bus is currently not used to distinguish between I2C masters.
 */
i2c_error_t axI2CWriteByte(void *conn_ctx, unsigned char bus, unsigned char addr, unsigned char *pTx)
{
  int nrWritten = -1;
  i2c_error_t rv;
  int axSmDevice = *(int *)conn_ctx;

  if (bus != I2C_BUS_0)
  {
    LOG_E("axI2CWriteByte on wrong bus %x (addr %x)\n", bus, addr);
  }

  nrWritten = write(axSmDevice, pTx, 1);
  if (nrWritten < 0)
  {
    // I2C_LOG_PRINTF("Failed writing data (nrWritten=%d).\n", nrWritten);
    rv = I2C_FAILED;
  }
  else
  {
    if (nrWritten == 1)
    {
      rv = I2C_OK;
    }
    else
    {
      rv = I2C_FAILED;
    }
  }

  return rv;
}
#endif // defined(SCI2C)

#if defined(SCI2C) || defined(T1oI2C)
i2c_error_t axI2CWrite(void *conn_ctx, unsigned char bus, unsigned char addr, unsigned char *pTx, unsigned short txLen)
{
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, addr, pTx, txLen, HAL_MAX_DELAY);
  LOG_D("[TX]: %d bytes transmit. ErrorCode: %d", txLen, status);
  LOG_AU8_D(pTx, txLen);
  if (status == HAL_OK)
  {
    return I2C_OK;
  }

  return I2C_FAILED;
}
#endif // defined(SCI2C) || defined(T1oI2C)

#if defined(SCI2C)
i2c_error_t axI2CWriteRead(void *conn_ctx, unsigned char bus, unsigned char addr, unsigned char *pTx,
                           unsigned short txLen, unsigned char *pRx, unsigned short *pRxLen)
{
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];
  int r = 0;
  int i = 0;
  int axSmDevice = *(int *)conn_ctx;

  if (pTx == NULL || txLen > MAX_DATA_LEN)
  {
    return I2C_FAILED;
  }

  if (pRx == NULL || *pRxLen > MAX_DATA_LEN)
  {
    return I2C_FAILED;
  }

  if (bus != I2C_BUS_0) // change if bus 0 is not the correct bus
  {
    LOG_E("axI2CWriteRead on wrong bus %x (addr %x)\n", bus, addr);
  }

  messages[0].addr = default_axSmDevice_addr;
  messages[0].flags = 0;
  messages[0].len = txLen;
  messages[0].buf = pTx;

  // NOTE:
  // By setting the 'I2C_M_RECV_LEN' bit in 'messages[1].flags' one ensures
  // the I2C Block Read feature is used.
  messages[1].addr = default_axSmDevice_addr;
  messages[1].flags = I2C_M_RD | I2C_M_RECV_LEN;
  messages[1].len = 256;
  messages[1].buf = pRx;
  messages[1].buf[0] = 1;

  // NOTE:
  // By passing the two message structures via the packets structure as
  // a parameter to the ioctl call one ensures a Repeated Start is triggered.
  packets.msgs = messages;
  packets.nmsgs = 2;

  LOG_MAU8_D("TX (axI2CWriteRead ) > ", &packets.msgs[0].buf[i], txLen);

  // Send the request to the kernel and get the result back
  r = ioctl(axSmDevice, I2C_RDWR, &packets);

  // NOTE:
  // The ioctl return value in case of a NACK on the write address is '-1'
  // This impacts the error handling routine of the caller.
  // If possible distinguish between a general I2C error and a NACK on address
  // The way to do this is platform specific (depends on I2C bus driver).
  if (r < 0)
  {
    // LOG_E("axI2CWriteRead: ioctl cmd I2C_RDWR fails with value %d (errno: 0x%08X)\n", r, errno);
    // perror("Errorstring: ");
#ifdef PLATFORM_IMX
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
#define E_NACK_I2C_IMX ENXIO
    // #warning "ENXIO"
#else
#define E_NACK_I2C_IMX EIO
    // #warning "EIO"
#endif // LINUX_VERSION_CODE
    // In case of IMX, errno == E_NACK_I2C_IMX is not exclusively bound to NACK on address,
    // it can also signal a NACK on a data byte
    if (errno == E_NACK_I2C_IMX)
    {
      // I2C_LOG_PRINTF("axI2CWriteRead: ioctl signal NACK (errno = %d)\n", errno);
      return I2C_NACK_ON_ADDRESS;
    }
    else
    {
      // printf("axI2CWriteRead: ioctl error (errno = %d)\n", errno);
      return I2C_FAILED;
    }
#else
    // I2C_LOG_PRINTF("axI2CWriteRead: ioctl cmd I2C_RDWR fails with value %d (errno: 0x%08X)\n", r, errno);
    return I2C_FAILED;
#endif // PLATFORM_IMX
  }
  else
  {
    int rlen = packets.msgs[1].buf[0] + 1;

    // I2C_LOG_PRINTF("packets.msgs[1].len is %d \n", packets.msgs[1].len);
    LOG_MAU8_D("RX (axI2CWriteRead) < ", &packets.msgs[1].buf[i], rlen);
    for (i = 0; i < rlen; i++)
    {
      pRx[i] = packets.msgs[1].buf[i];
    }
    *pRxLen = rlen;
  }

  return I2C_OK;
}
#endif // defined(SCI2C)

#ifdef T1oI2C
i2c_error_t axI2CRead(void *conn_ctx, unsigned char bus, unsigned char addr, unsigned char *pRx, unsigned short rxLen)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  do
  {
    osDelay(1);
    status = HAL_I2C_Master_Receive(&hi2c1, addr, pRx, rxLen, HAL_MAX_DELAY);
  } while (status != HAL_OK);

  LOG_D("[RX]: %d bytes received. ErrorCode: %d", rxLen, status);
  LOG_AU8_D(pRx, rxLen);
  if (status == HAL_OK)
  {
    return I2C_OK;
  }

  return I2C_FAILED;
}
#endif // T1oI2C
