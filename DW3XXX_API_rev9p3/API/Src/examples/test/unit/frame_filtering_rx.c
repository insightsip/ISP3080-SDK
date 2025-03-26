/*! ----------------------------------------------------------------------------
 *  @file    frame_filtering_rx.c
 *  @brief   Test each frame filter available on the DW3000
 *
 *           This test code is designed to test all available frame filtering modes on a DW3000. This code will transmit each of the different frame
 *           types to a receiving DW3000 device. The receiving device will apply frame filtering rules to the frame and reply with an
 *           acknowledge frame if the frame filtering is successful. This code will transmit one of each of the eight different frame types.
 *           The different frame types are set in the first three bits of the frame in the 'Frame Type' filed within the 'Frame Control' field,
 *           which in turn is encapsulated within the MAC frame format as defined by IEEE 802.15.4-2015. The frame types this code will be using
 *           are as follows:
 *           |-----------------------------------------------|
 *           | Frame Type Value (b2 b1 b0) | Description     |
 *           |-----------------------------------------------|
 *           | 000                         | Beacon          |
 *           | 001                         | Data            |
 *           | 010                         | Acknowledgement |
 *           | 011                         | MAC command     |
 *           | 100                         | Resevered       |
 *           | 101                         | Multipurpose    |
 *           | 110                         | Fragment or Frak|
 *           | 111                         | Extended        |
 *           |-----------------------------------------------|
 *
 *           Each of these frame types will be sent to the receiver multiple times in sequence. That way the test program can check the
 *           operation and reliability of the frame filtering features.
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
#include <shared_defines.h>
#include <shared_functions.h>
#include <deca_probe_interface.h>

#if defined(TEST_FRAME_FILTERING_RX)

extern void test_run_info(unsigned char *data);

/* Example application name and version to display on LCD screen. */
#define APP_NAME "F FILTER RX v1.0"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5, /* Channel number. */
    DWT_PLEN_128, /* Preamble length. Used in TX only. */
    DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
    9, /* TX preamble code. Used in TX only. */
    9, /* RX preamble code. Used in RX only. */
    1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8, /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64, /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0 /* PDOA mode off */
};

/* PAN ID/EUI/short address. See NOTE 1 and 2 below. */
static uint16_t pan_id = 0xDECA;
static uint8_t eui[] = { 'A', 'C', 'K', 'D', 'A', 'T', 'R', 'X' };
static uint16_t short_addr = 0x5258; /* "RX" */
// static uint16_t tx_short_addr = 0x5458; /*  "TX" */

/* Buffer to store received frame. See NOTE 3 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* ACK request bit mask in DATA and MAC COMMAND frame control's first byte. */
#define FCTRL_ACK_REQ_MASK 0x20

/* Frame version bit mask in DATA and MAC COMMAND frame control's second byte. */
#define FCTRL_FR_VER_MASK 0x30

/* Frame control index to check frame type in received frame */
#define FRAME_FC_IDX 0

/* Frame types */
#define FRAME_TYPE_BEACON       0x0
#define FRAME_TYPE_DATA         0x1
#define FRAME_TYPE_ACK          0x2
#define FRAME_TYPE_MAC          0x3
#define FRAME_TYPE_RESERVED     0x4
#define FRAME_TYPE_MULTIPURPOSE 0x5
#define FRAME_TYPE_FRAG         0x6
#define FRAME_TYPE_EXTENDED     0x7
#define FRAME_TYPE_MASK         0x07

/* Hold copy of count of received frame types here for reference so that it can be examined at a debug breakpoint */
static uint32_t frame_type_count_beacon = 0;
static uint32_t frame_type_count_beacon_std = 0;
static uint32_t frame_type_count_beacon_wrong_pan_id = 0;
static uint32_t frame_type_count_beacon_seq_no_suppressed = 0;
static uint32_t frame_type_count_beacon_long_addrs = 0;

static uint32_t frame_type_count_enhanced_beacon = 0;
static uint32_t frame_type_count_enhanced_beacon_wrong_pan_id = 0;
static uint32_t frame_type_count_enhanced_beacon_seq_no_suppressed = 0;
static uint32_t frame_type_count_enhanced_beacon_long_addrs = 0;

static uint32_t frame_type_count_data = 0;
static uint32_t frame_type_count_data_wrong_pan_id = 0;
static uint32_t frame_type_count_data_seq_no_suppressed = 0;
static uint32_t frame_type_count_data_long_addrs = 0;

static uint32_t frame_type_count_ack = 0;
static uint32_t frame_type_count_enhanced_ack = 0;
static uint32_t frame_type_count_enhanced_ack_wrong_pan_id = 0;
static uint32_t frame_type_count_enhanced_ack_seq_no_suppressed = 0;
static uint32_t frame_type_count_enhanced_ack_long_addrs = 0;

static uint32_t frame_type_count_mac = 0;
static uint32_t frame_type_count_mac_wrong_pan_id = 0;
static uint32_t frame_type_count_mac_seq_no_suppressed = 0;
static uint32_t frame_type_count_mac_long_addrs = 0;

static uint32_t frame_type_count_reserved = 0;
static uint32_t frame_type_count_reserved_wrong_pan_id = 0;
static uint32_t frame_type_count_reserved_seq_no_suppressed = 0;
static uint32_t frame_type_count_reserved_long_addrs = 0;

static uint32_t frame_type_count_multipurpose = 0;
static uint32_t frame_type_count_multi_wrong_pan_id = 0;
static uint32_t frame_type_count_multi_seq_no_suppressed = 0;
static uint32_t frame_type_count_multi_long_addrs = 0;
static uint32_t frame_type_count_multi_short = 0;

static uint32_t frame_type_count_frag = 0;
static uint32_t frame_type_count_extended = 0;

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16_t frame_len = 0;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 5 below. */
extern dwt_txconfig_t txconfig_options;

/**
 * Application entry point.
 */
int frame_filtering_rx(void)
{
    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) { };
    }

    /* Configure DW IC. See NOTE 9 below. */
    dwt_configure(&config);

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Set PAN ID, EUI and short address. See NOTE 1 below. */
    dwt_setpanid(pan_id);
    dwt_seteui(eui);
    dwt_setaddress16(short_addr);

    /* Set the frame pending registers */
    //    dwt_write16bitoffsetreg(LE_PEND_01_ID, 0, tx_short_addr);

    /* Configure frame filtering. */
    /* Keeping multiple options here commented out so that user can easily switch between them. */
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_BEACON_EN); /* Beacon / Enhanced Beacon frame filtering not working on B0 */
    //    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
    //    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_ACK_EN);
    //    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_MAC_EN);
    //    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_RSVD_EN); /* Reserved frame filtering not working on B0 */
    //    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_MULTI_EN);
    //    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_FRAG_EN);
    //    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_EXTEND_EN);
    //
    //    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_BEACON_EN | DWT_FF_DATA_EN | DWT_FF_ACK_EN | DWT_FF_MAC_EN | DWT_FF_RSVD_EN | DWT_FF_MULTI_EN
    //    | DWT_FF_FRAG_EN | DWT_FF_EXTEND_EN);

    /* Activate auto-acknowledgement. Time is set to 0 so that the ACK is sent as soon as possible after reception of a frame. */
    /* Frame filtering must be enabled for Auto ACK to work. */
    /* Just because auto ACK is enabled here, does not mean you will automatically send the ACK frame.
     * First, the frame being received must have ACK Request bit set in the Frame Control (See IEEE 802.14.5-2015 for details). */
    /*
     * The API call below should only be used if either the DATA or MAC frame filter is in use.
     * Comment the code out if any other frame filter is in use (or you simply do not want to enable auto ACK.
     */
    dwt_enableautoack(0, 1);

    /* can enable TX/RX states output on GPIOs 5 and 6 to help debug */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    /* Loop forever receiving frames. */
    while (1)
    {
        /* Activate reception immediately. See NOTE 6 below. */
        dwt_rxenable(0);

        /* Poll until a frame is properly received or an RX error occurs. See NOTE 7 below.
         * STATUS register is 5 bytes long but we are not interested in the high byte here, so we read a more manageable 32-bits with this API call. */
        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR), 0);

        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        {
            /* Clear good RX frame event in the DW IC status register. */
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_getframelength();
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* TESTING BREAKPOINT LOCATION #1 */

            /* Since the auto ACK feature is enabled, an ACK should be sent if the received frame requests it, so we await the ACK TX completion
             * before taking next action. See NOTE 8 below. */
            /* It is also important to check the frame filter applied to see if the program should wait for an ACK to be sent or not.
             * Only data and MAC frames with a frame version of '0' or '1' are going to send Auto ACKs */
            /*
             * The code block below should only be used if either the DATA or MAC frame filter is in use.
             * Comment the code out if any other frame filter is in use (or you simply do not want to wait for a auto ACk to be received.
             */
            //            if ((rx_buffer[0] & FCTRL_ACK_REQ_MASK)
            //                    && ((dwt_getframefilter() & DWT_FF_DATA_EN) || (dwt_getframefilter() & DWT_FF_MAC_EN))
            //                    && ((rx_buffer[1] & FCTRL_FR_VER_MASK == 0 << 4) || (rx_buffer[1] & FCTRL_FR_VER_MASK == 1 << 4)))
            //            {
            //                /* Poll DW IC until confirmation of transmission of the ACK frame. */
            //                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & DWT_INT_TXFRS_BIT_MASK))
            //                { };
            //
            //                /* Clear TXFRS event. */
            //                dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            //            }

            /*
             * Now check the number and type of frames that have been received
             */
            if ((rx_buffer[FRAME_FC_IDX] & FRAME_TYPE_MASK) == FRAME_TYPE_BEACON)
            {
                frame_type_count_beacon++;
                /* Check for standard beacon frame */
                if ((rx_buffer[FRAME_FC_IDX] == 0x00) && (rx_buffer[FRAME_FC_IDX + 1] == 0x80))
                {
                    frame_type_count_beacon_std++;
                    /* Check for wrong PAN ID frame */
                    if ((rx_buffer[FRAME_FC_IDX + 3] == 0xCA) && (rx_buffer[FRAME_FC_IDX + 4] == 0x00))
                    {
                        frame_type_count_beacon_wrong_pan_id++;
                    }
                    /* Check for frames with sequence number suppressed */
                    else if (rx_buffer[FRAME_FC_IDX + 1] & 0x01)
                    {
                        frame_type_count_beacon_seq_no_suppressed++;
                    }
                    /* Check for frames with long addresses */
                    else if (rx_buffer[FRAME_FC_IDX + 1] & 0xC0)
                    {
                        frame_type_count_beacon_long_addrs++;
                    }
                }
                else if ((rx_buffer[FRAME_FC_IDX] == 0x00) && (rx_buffer[FRAME_FC_IDX + 1] == 0xA0))
                {
                    frame_type_count_enhanced_beacon++;
                    /* Check for wrong PAN ID frame */
                    if ((rx_buffer[FRAME_FC_IDX + 3] == 0xCA) && (rx_buffer[FRAME_FC_IDX + 4] == 0x00))
                    {
                        frame_type_count_enhanced_beacon_wrong_pan_id++;
                    }
                    /* Check for frames with sequence number suppressed */
                    else if (rx_buffer[FRAME_FC_IDX + 1] & 0x01)
                    {
                        frame_type_count_enhanced_beacon_seq_no_suppressed++;
                    }
                    /* Check for frames with long addresses */
                    else if (rx_buffer[FRAME_FC_IDX + 1] & 0xE0)
                    {
                        frame_type_count_enhanced_beacon_long_addrs++;
                    }
                }
            }
            else if ((rx_buffer[FRAME_FC_IDX] & FRAME_TYPE_MASK) == FRAME_TYPE_DATA)
            {
                frame_type_count_data++;
                /* Check for wrong PAN ID frame */
                if ((rx_buffer[FRAME_FC_IDX + 3] == 0xCA) && (rx_buffer[FRAME_FC_IDX + 4] == 0x00))
                {
                    frame_type_count_data_wrong_pan_id++;
                }
                /* Check for frames with sequence number suppressed */
                else if (rx_buffer[FRAME_FC_IDX + 1] & 0x01)
                {
                    frame_type_count_data_seq_no_suppressed++;
                }
                /* Check for frames with long addresses */
                else if (rx_buffer[FRAME_FC_IDX + 1] & 0xCC)
                {
                    frame_type_count_data_long_addrs++;
                }
            }
            else if ((rx_buffer[FRAME_FC_IDX] & FRAME_TYPE_MASK) == FRAME_TYPE_ACK)
            {
                if ((rx_buffer[FRAME_FC_IDX] == 0x02) && (rx_buffer[FRAME_FC_IDX + 1] == 0x00))
                {
                    frame_type_count_ack++;
                }
                else if ((rx_buffer[FRAME_FC_IDX] == 0x42) && (rx_buffer[FRAME_FC_IDX + 1] == 0xA8))
                {
                    frame_type_count_enhanced_ack++;
                    /* Check for wrong PAN ID frame */
                    if ((rx_buffer[FRAME_FC_IDX + 3] == 0xCA) && (rx_buffer[FRAME_FC_IDX + 4] == 0x00))
                    {
                        frame_type_count_enhanced_ack_wrong_pan_id++;
                    }
                    /* Check for frames with sequence number suppressed */
                    else if (rx_buffer[FRAME_FC_IDX + 1] & 0x01)
                    {
                        frame_type_count_enhanced_ack_seq_no_suppressed++;
                    }
                    /* Check for frames with long addresses */
                    else if (rx_buffer[FRAME_FC_IDX + 1] & 0xCC)
                    {
                        frame_type_count_enhanced_ack_long_addrs++;
                    }
                }
            }
            else if ((rx_buffer[FRAME_FC_IDX] & FRAME_TYPE_MASK) == FRAME_TYPE_MAC)
            {
                frame_type_count_mac++;
                /* Check for wrong PAN ID frame */
                if ((rx_buffer[FRAME_FC_IDX + 3] == 0xCA) && (rx_buffer[FRAME_FC_IDX + 4] == 0x00))
                {
                    frame_type_count_mac_wrong_pan_id++;
                }
                /* Check for frames with sequence number suppressed */
                else if (rx_buffer[FRAME_FC_IDX + 1] & 0x01)
                {
                    frame_type_count_mac_seq_no_suppressed++;
                }
                /* Check for frames with long addresses */
                else if (rx_buffer[FRAME_FC_IDX + 1] & 0xCC)
                {
                    frame_type_count_mac_long_addrs++;
                }
            }
            else if ((rx_buffer[FRAME_FC_IDX] & FRAME_TYPE_MASK) == FRAME_TYPE_RESERVED)
            {
                frame_type_count_reserved++;
                /* Check for wrong PAN ID frame */
                if ((rx_buffer[FRAME_FC_IDX + 3] == 0xCA) && (rx_buffer[FRAME_FC_IDX + 4] == 0x00))
                {
                    frame_type_count_reserved_wrong_pan_id++;
                }
                /* Check for frames with sequence number suppressed */
                else if (rx_buffer[FRAME_FC_IDX + 1] & 0x01)
                {
                    frame_type_count_reserved_seq_no_suppressed++;
                }
                /* Check for frames with long addresses */
                else if (rx_buffer[FRAME_FC_IDX + 1] & 0xCC)
                {
                    frame_type_count_reserved_long_addrs++;
                }
            }
            else if ((rx_buffer[FRAME_FC_IDX] & FRAME_TYPE_MASK) == FRAME_TYPE_MULTIPURPOSE)
            {
                frame_type_count_multipurpose++;
                /* Check for wrong PAN ID frame */
                if ((rx_buffer[FRAME_FC_IDX + 3] == 0xCA) && (rx_buffer[FRAME_FC_IDX + 4] == 0x00))
                {
                    frame_type_count_multi_wrong_pan_id++;
                }
                /* Check for frames with sequence number suppressed */
                else if (rx_buffer[FRAME_FC_IDX + 1] & 0x04)
                {
                    frame_type_count_multi_seq_no_suppressed++;
                }
                /* Check for frames with long addresses */
                else if (rx_buffer[FRAME_FC_IDX] & 0xF0)
                {
                    frame_type_count_multi_long_addrs++;
                }
                /* Check for short multipurpose frames (one frame control octet) */
                else if (rx_buffer[FRAME_FC_IDX] == 0xA5)
                {
                    frame_type_count_multi_short++;
                }
            }
            else if ((rx_buffer[FRAME_FC_IDX] & FRAME_TYPE_MASK) == FRAME_TYPE_FRAG)
            {
                frame_type_count_frag++;
            }
            else if ((rx_buffer[FRAME_FC_IDX] & FRAME_TYPE_MASK) == FRAME_TYPE_EXTENDED)
            {
                frame_type_count_extended++;
            }
        }
        else
        {
            /* Clear RX error events in the DW IC status register. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        }
    }
}
#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. PAN ID, EUI and short address are hard coded constants to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW IC unique ID by combining the Lot ID & Part Number values programmed into the DW IC
 *    during its manufacture. However there is no guarantee this will not conflict with someone else's implementation. We recommended that customers
 *    buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW IC User Manual.
 * 2. EUI64 is not actually used in this example but the DW IC is set up with this dummy value, to have it set to something. This would be required
 *    for a real application, i.e. because short addresses (and PAN ID) are typically assigned by a PAN coordinator.
 * 3. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW IC supports an extended frame
 *    length (up to 1023 bytes long) mode which is not used in this example.
 * 4. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 20 MHz can be used
 *     thereafter.
 * 5. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 6. Manual reception activation is performed here but DW IC offers several features that can be used to handle more complex scenarios or to
 *    optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 7. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts".
 * 8. This is the purpose of the AAT bit in DW IC's STATUS register but because of an issue with the operation of AAT, it is simpler to directly
 *    check in the frame control if the ACK request bit is set. Please refer to DW IC User Manual for more details on Auto ACK feature and the AAT
 *    bit.
 * 9. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *    configuration.
 ****************************************************************************************************************************************************/
