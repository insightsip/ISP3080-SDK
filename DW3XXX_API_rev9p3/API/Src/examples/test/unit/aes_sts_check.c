/*! ----------------------------------------------------------------------------
 *  @file    aes_sts_check.c
 *  @brief   Unit test that decrypted an encryped key from scratch buffer to STS keys..
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <stdint.h>
#include <stdlib.h>
#include "deca_probe_interface.h"
#include "deca_private.h"

// The following defines are taken from the C0 deca_regs.h file

#define DMA_CFG0_ID                   0x10044
#define DMA_CFG0_DST_PORT_BIT_OFFSET  (13U)
#define DMA_CFG1_ID                   0x10048
#define DMA_CFG1_PYLD_SIZE_BIT_OFFSET (7U)
#define DMA_CFG1_PYLD_SIZE_BIT_MASK   0x1ff80UL
#define DMA_CFG1_HDR_SIZE_BIT_OFFSET  (0U)
#define DMA_CFG1_HDR_SIZE_BIT_MASK    0x7fU
#define AES_START_ID                  0x1004c
#define AES_START_AES_START_BIT_MASK  0x1U
#define STS_KEY0_ID                   0x2000c
#define STS_KEY1_ID                   0x20010
#define STS_KEY2_ID                   0x20014
#define STS_KEY3_ID                   0x20018
#define STS_IV0_ID                    0x2001c
#define STS_IV1_ID                    0x20020
#define STS_IV2_ID                    0x20024
#define STS_IV3_ID                    0x20028

#define SCRATCH_RAM_ID          0x160000
#define AES_KEY_RAM_MEM_ADDRESS 0x170000 /*Address of the AES keys in RAM*/

/**
 * Application entry point.
 * This function uses key from RAM. It writes data to scratch memory(sts key), encrypted it on scratch and than decrypt it directly to STS regs.
 * It simulate passing encrypted key through SPI.
 */

void aes_sts_check(void)
{

    /* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
     * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
    extern dwt_txconfig_t txconfig_options;

    /* Default communication configuration. We use default non-STS DW mode. */
    static dwt_config_t config = {
        5, /* Channel number. */
        DWT_PLEN_128, /* Preamble length. Used in TX only. */
        DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
        9, /* TX preamble code. Used in TX only. */
        9, /* RX preamble code. Used in RX only. */
        1, /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_6M8, /* Data rate. */
        DWT_PHRMODE_EXT, /* PHY header mode: extended frame mode, up to 1023-16-2 in the payload */
        DWT_PHRRATE_STD, /* PHY header rate. */
        (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
        DWT_STS_MODE_OFF, /* STS mode*/
        DWT_STS_LEN_64, /* STS length see allowed values in Enum dwt_sts_lengths_e */
        DWT_PDOA_M0 /*pdoa mode*/
    };

    /* DW3000 chip can run from high speed from start-up.*/
    port_set_dw_ic_spi_fastrate();

    /* Reset and initialize DW chip. */
    reset_DWIC(); /* Target specific drive of RSTn line into DW3000 low for a period. */

    // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
    Sleep(2);

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf);

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR)
    {
        // test_run_info((unsigned char *)"INIT FAILED");
        while (TRUE) { };
    }

    /* Configure DW3000. */
    dwt_configure(&config);

    // configure the tx spectrum parameters (power and PG delay)
    dwt_configuretxrf(&txconfig_options);

    dwt_aes_config_t dwt_aes_sts_config = { .aes_core_type = AES_core_type_CCM, // AES_core_type_GCM,
        .aes_key_otp_type = AES_key_RAM,
        .mic = MIC_8, // MIC_16,/* Means 16 bytes tag*/
        .key_src = AES_KEY_Src_RAMorOTP,
        .key_load = AES_KEY_Load,
        .key_addr = 0,
        .key_size = AES_KEY_128bit,
        .mode = AES_Encrypt };

    uint8_t sts_key[] = { 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE, 0xF }; // Not encrypted
    // uint8_t   nonce_sts[]={0x00,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x00};//Add when testing!
    uint8_t aes_key[] = { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF };
    uint32_t tmp_reg_val;
    uint8_t tmp_buff[30] = { 0 }; // Just for testing

    dwt_writetodevice(AES_KEY_RAM_MEM_ADDRESS, 0, sizeof(aes_key), aes_key);
    // dwt_update_nonce_GCM(nonce_sts);
    // dwt_update_nonce_CCM(nonce_sts,16);//Add when testing!

    dwt_configure_aes(&dwt_aes_sts_config);
    // AES_RAM_KEY_MEM_ADDRESS
    dwt_writetodevice(SCRATCH_RAM_ID, 0, sizeof(sts_key), sts_key); // Writes unencrypted key to scratch
    // Need to encrypt the key on scratch
    dwt_writetodevice(DMA_CFG0_ID, 0, 4, 0);
    // No header only data
    tmp_reg_val = (DMA_CFG1_HDR_SIZE_BIT_MASK & (0 << DMA_CFG1_HDR_SIZE_BIT_OFFSET))
                  | (DMA_CFG1_PYLD_SIZE_BIT_MASK & (((uint32_t)sizeof(sts_key)) << DMA_CFG1_PYLD_SIZE_BIT_OFFSET));

    dwt_writetodevice(DMA_CFG1_ID, 0, 4, (uint8_t *)&tmp_reg_val);

    dwt_readfromdevice(SCRATCH_RAM_ID, 0, sizeof(sts_key), tmp_buff); // Read data before encryption

    /* start AES action scratch data should be encrypted */
    uint8_t buffer = (uint8_t)AES_START_AES_START_BIT_MASK;
    dwt_writetodevice(AES_START_AES_START_BIT_MASK, 0, 1, &buffer);
    // tmp_reg_val = dwt_wait_aes_poll();//Add when testing!

    memset(tmp_buff, 0, sizeof(tmp_buff));
    dwt_readfromdevice(SCRATCH_RAM_ID, 0, sizeof(tmp_buff), tmp_buff); // Read data after encryption

    dwt_aes_sts_config.mode = AES_Decrypt;
    dwt_configure_aes(&dwt_aes_sts_config);
    tmp_reg_val = ((uint32_t)AES_Dst_STS_key) << DMA_CFG0_DST_PORT_BIT_OFFSET; // Dest port is CP
    dwt_writetodevice(DMA_CFG0_ID, 0, 4, (uint8_t *)&tmp_reg_val);

    dwt_readfromdevice(STS_KEY0_ID, 0, 4, (uint8_t *)&tmp_reg_val); // Read prev val

    dwt_writetodevice(AES_START_ID, 0, 1, &buffer);
    // tmp_reg_val = dwt_wait_aes_poll();//Add when testing!

    // These values should contain the sts key unencrypted
    dwt_readfromdevice(STS_KEY0_ID, 0, 4, (uint8_t *)&tmp_reg_val); // Read decrypt val
    dwt_readfromdevice(STS_KEY1_ID, 0, 4, (uint8_t *)&tmp_reg_val); // Read decrypt val
    dwt_readfromdevice(STS_KEY2_ID, 0, 4, (uint8_t *)&tmp_reg_val); // Read decrypt val
    dwt_readfromdevice(STS_KEY3_ID, 0, 4, (uint8_t *)&tmp_reg_val); // Read decrypt val

    dwt_readfromdevice(STS_IV0_ID, 0, 4, (uint8_t *)&tmp_reg_val); // Read decrypt val
    dwt_readfromdevice(STS_IV1_ID, 0, 4, (uint8_t *)&tmp_reg_val); // Read decrypt val
    dwt_readfromdevice(STS_IV2_ID, 0, 4, (uint8_t *)&tmp_reg_val); // Read decrypt val
    dwt_readfromdevice(STS_IV3_ID, 0, 4, (uint8_t *)&tmp_reg_val); // Read decrypt val
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * TODO: Correct and/or required notes for this example can be added later.
 *
 ****************************************************************************************************************************************************/
