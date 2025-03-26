/*! ----------------------------------------------------------------------------
 *  @file    tx_rx_aes_verification.c
 *  @brief   Unit test that checks AES encrtyprion and decryption. Loop over Tags option + all payload sizes.
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
#include <mac_802_15_8.h>
#include <port.h>
#include <stdint.h>
#include <stdlib.h>
#include "deca_probe_interface.h"


#define EXT_FRAME_LEN   (1023)
#define RX_BUFFER_0_ID  0x120000 /* Default Receive Data Buffer (and the 1st of the double buffer set) */

extern void test_run_info(unsigned char *data);
void dwt_readfromdevice(uint32_t regFileID, uint16_t index, uint16_t length, uint8_t *buffer);

/* APP_KEY_0-APP_KEY_4 is a 128 bit AES key which should be set the same
 * for both Encryption and Decryption.
 * This should match complementary RX example.
 */
const dwt_aes_key_t aes_key = { 0x41424344, 0x45464748, 0x49505152, 0x53545556, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }; /*Initialize 128bits key*/
#define MAX_MIC_OPTIONS 8

static dwt_aes_config_t aes_config = { .key_load = AES_KEY_Load,
    .key_size = AES_KEY_128bit,
    .key_src = AES_KEY_Src_Register,
    .mic = MIC_16, /* Means 16 bytes tag*/
    .mode = AES_Encrypt,
    .aes_core_type = AES_core_type_GCM,
    .aes_key_otp_type = AES_key_RAM,
    .key_addr = 0 };

/* Below is a payload, which will be sent encrypted in this example */
uint8_t tx_payload[1023];
uint8_t rx_payload[1023];

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
    DWT_PHRRATE_STD, (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS mode*/
    DWT_STS_LEN_64, /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0 /*pdoa mode*/
};

/**
 * Application entry point.
 */
int tx_rx_aes_verification(void)
{
    uint8_t mic_option, res;
    uint16_t frame_ctrl_size, total_frame_length;
    aes_results_e aes_results;
    int8_t status;
    uint16_t cnt_bytes, max_payload_size;
    /* Set the AES key */
    uint64_t PN = 0; /* Can start also with random value, should not exceed 6 bytes - 0xFFFFFFFFFFFF*/
    uint8_t nonce[12];
    dwt_aes_job_t tx_aes_job, rx_aes_job;
    /* 802.15.8 Standard */
    mac_frame_802_15_8_format_t header = { .fc[0] = 0x50, /* DATA, SRC is 48 bit MAC, DST is 48 bit MAC */
        .fc[1] = 0x40, /* no ACK, no Header Information Element present,
                                              no Payload Information Element present,
                                              frame encrypted, R = 0 */
        .seq = 0, /* Sequence number */
        .dst_addr = { 0xA, 0xB, 0xC, 0xD, 0xE, 0xF }, /* 48 bit Destination address (RX device) is 0x0F0E0D0C0B0A */
        .src_addr = { 0x1, 0x2, 0x3, 0x4, 0x5, 0x6 }, /* 48 bit Source address (TX device) is 0x060504030201 */
        .nonce = { 0 } };

    /* DW3000 chip can run from high speed from start-up.*/
    port_set_dw_ic_spi_fastrate();
    // port_set_dw_ic_spi_slowrate();

    /* Reset and initialize DW chip. */
    reset_DWIC(); /* Target specific drive of RSTn line into DW3000 low for a period. */

    // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
    Sleep(2);

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf);

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED");
        while (TRUE) { };
    }

    /* Configure DW3000. */
    dwt_configure(&config);

    // configure the tx spectrum parameters (power and PG delay)
    dwt_configuretxrf(&txconfig_options);
    dwt_set_keyreg_128((dwt_aes_key_t *)&aes_key);

    /* Fill aes job to do */
    tx_aes_job.nonce = nonce; /* use constructed nonce to encrypt payload */
    tx_aes_job.header = (uint8_t *)&header; /* plain-text header which will not be encrypted */
    tx_aes_job.header_len = sizeof(header);
    tx_aes_job.payload = tx_payload; /* payload to be encrypted */
    tx_aes_job.src_port = AES_Src_Tx_buf; /* dwt_do_aes will take plain text to the TX buffer */
    tx_aes_job.dst_port = AES_Dst_Rx_buf_0; /* dest port is RX buff */
    tx_aes_job.mode = aes_config.mode;

    rx_aes_job.src_port = AES_Src_Rx_buf_0; /* Take encrypted frame from the RX buffer */
    rx_aes_job.dst_port = AES_Dst_Rx_buf_0; /* Decrypt the frame to the same RX buffer : this will destroy original RX frame */
    rx_aes_job.mode = AES_Decrypt;

    memcpy(&nonce[6], &header.src_addr[0], 6);

    PN &= 0xFFFFFFFFFFFF; // Verify PN that was entered is not more than 6 bytes

    // Loop over all the possibilities of mic sizes
    for (mic_option = 0; mic_option < MAX_MIC_OPTIONS; mic_option++)
    {
        if (mic_option == 0)
            tx_aes_job.mic_size = 0;
        else
            tx_aes_job.mic_size = mic_option * 2 + 2;
        aes_config.mic = mic_option;
        rx_aes_job.mic_size = tx_aes_job.mic_size;
        // dwt_configure_aes(&aes_config);

        frame_ctrl_size = sizeof(header) + FCS_LEN + tx_aes_job.mic_size;
        memset(tx_payload, 0, sizeof(tx_payload));
        memset(rx_payload, 0, sizeof(rx_payload));
        max_payload_size = EXT_FRAME_LEN - sizeof(header) - FCS_LEN - tx_aes_job.mic_size; // This is the max data that can be sent
        for (cnt_bytes = 1; cnt_bytes <= max_payload_size; cnt_bytes++)
        {
            total_frame_length = frame_ctrl_size + cnt_bytes;
            // dwt_writetxfctrl(total_frame_length, 0, 0);/* Set the frame control size*/
            tx_payload[cnt_bytes - 1] = cnt_bytes; // Update tx_payload

            nonce[0] = header.nonce[0] = (uint8_t)PN;
            nonce[1] = header.nonce[1] = (uint8_t)(PN >> 8);
            nonce[2] = header.nonce[2] = (uint8_t)(PN >> 16);
            nonce[3] = header.nonce[3] = (uint8_t)(PN >> 24);
            nonce[4] = header.nonce[4] = (uint8_t)(PN >> 32);
            nonce[5] = header.nonce[5] = (uint8_t)(PN >> 40);

            /* Assuming head & nonce are both of the same storage container size */
            tx_aes_job.payload_len = cnt_bytes;
            aes_config.mode = AES_Encrypt;
            dwt_configure_aes(&aes_config);
            status = dwt_do_aes(&tx_aes_job, aes_config.aes_core_type); /* Note, 802.15.8 adds a MIC size of 16 bytes after tx_payload */
            if (status < 0)
            { // Problem with Header/Payload length or mode selection
                test_run_info((unsigned char *)"Length AES error");
                while (1) { };
            }
            else
            { // Check return status and transmit if OK
                if (status & DWT_AES_ERRORS)
                {
                    test_run_info((unsigned char *)"ERROR AES");
                    while (1) { };
                }
                else
                { // There were no errors
                    Sleep(1);
                    // rx_aes_job.mic_size      = tx_aes_job.mic_size;
                    aes_config.mode = AES_Decrypt;
                    dwt_configure_aes(&aes_config);

                    dwt_readfromdevice(RX_BUFFER_0_ID, tx_aes_job.header_len, cnt_bytes, rx_payload);

                    aes_results = rx_aes_802_15_8(total_frame_length, &rx_aes_job, rx_payload, sizeof(rx_payload), aes_config.aes_core_type);
                    res = memcmp(rx_payload, tx_payload, cnt_bytes);

                    if (res)
                    {
                        while (1)
                        {
                            dwt_readfromdevice(RX_BUFFER_0_ID, tx_aes_job.header_len, cnt_bytes, rx_payload);
                        }
                    }

                    if ((aes_results != AES_RES_OK) || (res))
                    {
                        uint8_t i = 0;
                        while (i == 0)
                            ;
                    }

                    /* Clear TX frame sent event. */
                    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                    PN = (PN + 1) % 0xFFFFFFFFFFFF; // PN can be saved as 6 bytes
                    header.seq++;
                    unsigned char str[20];
                    static int cnt;
                    sprintf((char *)str, "AES TX OK %d", cnt++);
                    test_run_info(str);
                }
            }
        }
    }

    return 0;
}
//#endif

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * TODO: Correct and/or required notes for this example can be added later.
 *
 ****************************************************************************************************************************************************/
