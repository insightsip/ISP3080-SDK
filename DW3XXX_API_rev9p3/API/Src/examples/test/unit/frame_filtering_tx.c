/*! ----------------------------------------------------------------------------
 *  @file    frame_filtering_tx.c
 *  @brief   Test function to test each of the frame filter modes.
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

#if defined(TEST_FRAME_FILTERING_TX)

extern void test_run_info(unsigned char *data);

/* Example application name and version to display on LCD screen. */
#define APP_NAME "F FILTER TX v1.0"

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

/*
 * The frame sent in this example is an enhanced beacon frame encoded as per the IEEE 802.15.4-2015 standard.
 * It is composed of the following fields:
 *     - byte 0/1: frame control (0x8000)
 *       - bits 0-2: Frame Type: 000 - Beacon frame
 *       - bit 3: Security Enabled: 0 - No security enabled
 *       - bit 4: Frame Pending: 0 - No additional data for recipient
 *       - bit 5: AR: 0 - No ACk frame required
 *       - bit 6: PAN ID Compression: 0 - No compression
 *       - bit 7: Reserved: 0
 *       - bit 8: Sequence Number Suppression: 0 - Sequence number field is present
 *       - bit 9: IE Present: 0 - No IEs contained in frame
 *       - bits 10-11: Destination Addressing Mode: 00 - No Destination address or PAN ID
 *       - bits 12-13: Frame Version: 00 - Using IEEE Std 802.15.4-2003 frames
 *       - bits 14-15: Source Addressing Mode: 10 - Include source address and source PAN ID in frame
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: Source PAN ID (0xDECA)
 *     - byte 5/6: source address, see NOTE 2 below.
 *     - byte 7/8: Superframe Specification
 *       - bits 0-3: Beacon Order: 0000
 *       - bits 4-7: Superframe Order: 0000
 *       - bits 8-11: Final CAP Slot: 0000
 *       - bit 12: Battery Life Extension (BLE): 0
 *       - bit 13: Reserved: 0
 *       - bit 14: PAN Coordinator: 0
 *       - bit 15: Association Permit: 0
 *     - byte 9/10: frame check-sum, automatically set by DW IC.
 */
static uint8_t beacon_frame[] = { 0x00, 0x80, 0x00, 0xCA, 0xDE, 'X', 'T', 0x00, 0x00, 0x00, 0x00 };
/* Beacon frame with wrong PAN ID */
static uint8_t beacon_frame_wrong_pan_id[] = { 0x00, 0x80, 0x00, 0xCA, 0x00, 'X', 'T', 0x00, 0x00, 0x00, 0x00 };
/* Beacon frame with long addresses */
static uint8_t beacon_frame_long_addrs[] = { 0x00, 0xC0, 0x00, 0xCA, 0xDE, 'X', 'T', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x00, 0x00, 0x00 };

/*
 * The frame sent in this example is an enhanced beacon frame encoded as per the IEEE 802.15.4-2015 standard.
 * It is composed of the following fields:
 *     - byte 0/1: frame control (0xA860)
 *       - bits 0-2: Frame Type: 000 - Beacon frame
 *       - bit 3: Security Enabled: 0 - No security enabled
 *       - bit 4: Frame Pending: 0 - No additional data for recipient
 *       - bit 5: AR: 0 - No ACK required.
 *       - bit 6: PAN ID Compression: 0 - No PAN ID compression.
 *       - bit 7: Reserved: 0
 *       - bit 8: Sequence Number Suppression: 0 - Sequence number field is present
 *       - bit 9: IE Present: 0 - IEs contained in frame
 *       - bits 10-11: Destination Addressing Mode: 00 - No destination address or PAN ID.
 *       - bits 12-13: Frame Version: 10 - Using IEEE Std 802.15.4-2015 frames
 *       - bits 14-15: Source Addressing Mode: 10 - Include source address and source PAN ID in frame
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA)
 *     - byte 5/6: destination address, see NOTE 2 below.
 *     - byte 7/8: source address, see NOTE 2 below.
 *     - byte 9/10: frame check-sum, automatically set by DW IC.
 */
static uint8_t enchanced_beacon_frame[] = { 0x00, 0xA0, 0x00, 0xCA, 0xDE, 'X', 'T', 0x00, 0x00 };
/* Enhanced beacon frame with wrong PAN ID */
static uint8_t enchanced_beacon_frame_wrong_pan_id[] = { 0x00, 0xA0, 0x00, 0xCA, 0x00, 'X', 'T', 0x00, 0x00 };
/* Enhanced beacon frame with seq no. suppression */
static uint8_t enhanced_beacon_frame_seq_no_suppressed[] = { 0x00, 0xA1, 0xCA, 0xDE, 'X', 'T', 0x00, 0x00 };
/* Enhanced beacon frame with long addresses */
static uint8_t enhanced_beacon_frame_long_addrs[] = { 0x00, 0xE0, 0x00, 0xCA, 0xDE, 'X', 'T', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x00 };

/* The frame sent in this example is a data frame encoded as per the IEEE 802.15.4-2011 standard. It is a 21-byte frame composed of the following
 * fields:
 *     - byte 0/1: frame control (0x8861 to indicate a data frame using 16-bit addressing and requesting ACK).
 *       - bits 0-2: Frame Type: 001 - Data frame
 *       - bit 3: Security Enabled: 0 - No security enabled
 *       - bit 4: Frame Pending: 0 - No additional data for recipient
 *       - bit 5: AR: 1 - ACK frame required from recipient device on receipt of data frame
 *       - bit 6: PAN ID Compression: 1 - PAN IDs are identical, Source PAN ID field shall be omitted from transmitted frame
 *       - bit 7: Reserved: 0
 *       - bit 8: Sequence Number Suppression: 0 - Sequence number field is present
 *       - bit 9: IE Present: 0 - No IEs contained in frame
 *       - bits 10-11: Destination Addressing Mode: 10 - Address field contains short address
 *       - bits 12-13: Frame Version: 00 - Using IEEE Std 802.15.4-2003 frames
 *       - bits 14-15: Source Addressing Mode: 10 - Include source address in frame
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA)
 *     - byte 5/6: destination address, see NOTE 2 below.
 *     - byte 7/8: source address, see NOTE 2 below.
 *     - byte 9 to 18: MAC payload, see NOTE 1 below.
 *     - byte 19/20: frame check-sum, automatically set by DW IC. */
static uint8_t data_frame[] = { 0x61, 0x88, 0, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 'm', 'a', 'c', 'p', 'a', 'y', 'l', 'o', 'a', 'd', 0, 0 };
/* Data frame with wrong PAN ID */
static uint8_t data_frame_wrong_pan_id[] = { 0x61, 0x88, 0, 0xCA, 0x00, 'X', 'R', 'X', 'T', 'm', 'a', 'c', 'p', 'a', 'y', 'l', 'o', 'a', 'd', 0, 0 };
/* Data frame with sequence number suppressed */
static uint8_t data_frame_seq_no_suppressed[] = { 0x61, 0xA9, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 'm', 'a', 'c', 'p', 'a', 'y', 'l', 'o', 'a', 'd', 0, 0 };
/* Data frame with long addresses */
static uint8_t data_frame_long_addrs[] = { 0x61, 0xCC, 0, 0xCA, 0xDE, 'X', 'R', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 'X', 'T', 0x01, 0x02, 0x03, 0x04, 0x05,
    0x06, 'm', 'a', 'c', 'p', 'a', 'y', 'l', 'o', 'a', 'd', 0, 0 };

/*
 * The frame sent in this example is an immediate acknowledgement frame encoded as per the IEEE 802.15.4-2015 standard.
 * It is composed of the following fields:
 *     - byte 0/1: frame control (0x0002)
 *       - bits 0-2: Frame Type: 010 - ACK frame
 *       - bit 3: Security Enabled: 0 - No security enabled
 *       - bit 4: Frame Pending: 0 - No additional data for recipient
 *       - bit 5: AR: 0 - ACK frame required from recipient device on receipt of data frame
 *       - bit 6: PAN ID Compression: 0 - PAN IDs are identical, Source PAN ID field shall be omitted from transmitted frame
 *       - bit 7: Reserved: 0
 *       - bit 8: Sequence Number Suppression: 0 - Sequence number field is present
 *       - bit 9: IE Present: 0 - No IEs contained in frame
 *       - bits 10-11: Destination Addressing Mode: 00 - Address field contains short address
 *       - bits 12-13: Frame Version: 00 - Using IEEE Std 802.15.4-2003 frames
 *       - bits 14-15: Source Addressing Mode: 00 - Include source address in frame
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: frame check-sum, automatically set by DW IC.
 */
static uint8_t imm_ack_frame[] = { 0x02, 0x00, 0, 0, 0 };

/*
 * The frame sent in this example is an enhanced acknowledgement frame encoded as per the IEEE 802.15.4-2015 standard.
 * It is composed of the following fields:
 *     - byte 0/1: frame control (0xA842)
 *       - bits 0-2: Frame Type: 010 - ACK frame
 *       - bit 3: Security Enabled: 0 - No security enabled
 *       - bit 4: Frame Pending: 0 - No additional data for recipient
 *       - bit 5: AR: 0 - ACK frame not required from recipient device on receipt of data frame
 *       - bit 6: PAN ID Compression: 1 - PAN IDs are identical, Source PAN ID field shall be omitted from transmitted frame
 *       - bit 7: Reserved: 0
 *       - bit 8: Sequence Number Suppression: 0 - Sequence number field is present
 *       - bit 9: IE Present: 0 - No IEs contained in frame
 *       - bits 10-11: Destination Addressing Mode: 10 - Address field contains short address
 *       - bits 12-13: Frame Version: 10 - Using IEEE Std 802.15.4-2015 frames
 *       - bits 14-15: Source Addressing Mode: 10 - Include source address in frame
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: frame check-sum, automatically set by DW IC.
 */
static uint8_t enhanced_ack_frame[] = { 0x42, 0xA8, 0x00, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 0x00, 0x00 };
/* Enhanced ACK frame with wrong PAN ID */
static uint8_t enhanced_ack_frame_wrong_pan_id[] = { 0x42, 0xA8, 0x00, 0xCA, 0x00, 'X', 'R', 'X', 'T', 0x00, 0x00 };
/* Enhanced ACK frame with sequence number suppressed */
static uint8_t enhanced_ack_frame_seq_no_suppressed[] = { 0x42, 0xA9, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 0x00, 0x00 };
/* Enhanced ACK frame with long addresses */
static uint8_t enhanced_ack_frame_long_addrs[]
    = { 0x42, 0xEC, 0x00, 0xCA, 0xDE, 'X', 'R', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 'X', 'T', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x00 };

/*
 * The frame sent in this example is a MAC command frame encoded as per the IEEE 802.15.4-2015 standard.
 * It is composed of the following fields:
 *     - byte 0/1: frame control (0x8863 to indicate a MAC command frame using 16-bit addressing and requesting ACK).
 *       - bits 0-2: Frame Type: 011 - MAC command frame
 *       - bit 3: Security Enabled: 0 - No security enabled
 *       - bit 4: Frame Pending: 0 - No additional data for recipient
 *       - bit 5: AR: 1 - ACK frame required from recipient device on receipt of data frame
 *       - bit 6: PAN ID Compression: 1 - PAN IDs are identical, Source PAN ID field shall be omitted from transmitted frame
 *       - bit 7: Reserved: 0
 *       - bit 8: Sequence Number Suppression: 0 - Sequence number field is present
 *       - bit 9: IE Present: 0 - No IEs contained in frame
 *       - bits 10-11: Destination Addressing Mode: 10 - Address field contains short address
 *       - bits 12-13: Frame Version: 00 - Using IEEE Std 802.15.4-2003 frames
 *       - bits 14-15: Source Addressing Mode: 10 - Include source address in frame
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA)
 *     - byte 5/6: destination address, see NOTE 2 below.
 *     - byte 7/8: source address, see NOTE 2 below.
 *     - byte 9: Command ID field, see NOTE 9 below.
 *     - byte 10/11: frame check-sum, automatically set by DW IC.
 */
static uint8_t mac_frame[] = { 0x63, 0x88, 0x00, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 0x04, 0x00, 0x00 };
/* MAC frame with wrong PAN ID */
static uint8_t mac_frame_wrong_pan_id[] = { 0x63, 0x88, 0x00, 0xCA, 0x00, 'X', 'R', 'X', 'T', 0x04, 0x00, 0x00 };
/* MAC frame with sequence number suppressed */
static uint8_t mac_frame_seq_no_suppressed[] = { 0x63, 0xA9, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 0x04, 0x00, 0x00 };
/* MAC frame with long addresses */
static uint8_t mac_frame_long_addrs[]
    = { 0x63, 0xCC, 0x00, 0xCA, 0xDE, 'X', 'R', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 'X', 'T', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x04, 0x00, 0x00 };

/*
 * The frame sent in this example is a 'reserved' frame encoded as per the IEEE 802.15.4-2015 standard.
 * It is composed of the following fields:
 *     - byte 0/1: frame control (0x88E4 to indicate a reserved frame using 16-bit addressing and requesting ACK).
 *       - bits 0-2: Frame Type: 100 - reserved frame
 *       - bit 3: Security Enabled: 0 - No security enabled
 *       - bit 4: Frame Pending: 0 - No additional data for recipient
 *       - bit 5: AR: 1 - ACK frame required from recipient device on receipt of data frame
 *       - bit 6: PAN ID Compression: 1 - PAN IDs are identical, Source PAN ID field shall be omitted from transmitted frame
 *       - bit 7: Reserved: 1
 *       - bit 8: Sequence Number Suppression: 0 - Sequence number field is present
 *       - bit 9: IE Present: 0 - No IEs contained in frame
 *       - bits 10-11: Destination Addressing Mode: 10 - Address field contains short address
 *       - bits 12-13: Frame Version: 00 - Using IEEE Std 802.15.4-2003 frames
 *       - bits 14-15: Source Addressing Mode: 10 - Include source address in frame
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA)
 *     - byte 5/6: destination address, see NOTE 2 below.
 *     - byte 7/8: source address, see NOTE 2 below.
 *     - byte 9: 'Data' field, see NOTE 10 below.
 *     - byte 10/11: frame check-sum, automatically set by DW IC.
 */
static uint8_t reserved_frame[] = { 0xE4, 0x88, 0x00, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 0x46, 0x00, 0x00 };
/* Reserved frame with wrong PAN ID */
static uint8_t reserved_frame_wrong_pan_id[] = { 0xE4, 0x88, 0x00, 0xCA, 0x00, 'X', 'R', 'X', 'T', 0x46, 0x00, 0x00 };
/* Reserved frame with sequence number suppressed */
static uint8_t reserved_frame_seq_no_suppressed[] = { 0xE4, 0xA9, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 0x46, 0x00, 0x00 };
/* Reserved frame with long addresses */
static uint8_t reserved_frame_long_addrs[]
    = { 0xE4, 0xCC, 0x00, 0xCA, 0xDE, 'X', 'R', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 'X', 'T', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x46, 0x00, 0x00 };

/*
 * The frame sent in this example is a multipurpose frame encoded as per the IEEE 802.15.4-2015 standard.
 * It is composed of the following fields:
 *     - byte 0/1: frame control (0x41AD to indicate a multipurpose frame using 16-bit addressing and requesting ACK).
 *       - bits 0-2: Frame Type: 101 - multipurpose frame
 *       - bit 3: Long Frame Control: 1 - MP Long Frame Control
 *       - bits 4-5: Destination Addressing Mode: 10 - Address field contains short address
 *       - bits 6-7: Source Addressing Mode: 10 - Include source address in frame
 *       - bit 8: PAN ID Present: 1 - PAN ID is present in Multipurpose Header (MHR)
 *       - bit 9: Security Enabled: 0 - Security disabled
 *       - bit 10: Sequence Number Suppression: 0 - Sequence number field is present
 *       - bit 11: Frame Pending: 0 - No additional data for recipient
 *       - bits 12-13: Frame Version: 00 - Using IEEE Std 802.15.4-2003 frames
 *       - bit 14 Ack Request: 1 - ACK frame required from recipient device on receipt of multipurpose frame
 *       - bit 15 IE Present: 0 - No IEs present.
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA)
 *     - byte 5/6: destination address, see NOTE 2 below.
 *     - byte 7/8: source address, see NOTE 2 below.
 *     - byte 9: MAC payload, see NOTE 1 below.
 *     - byte 10/11: frame check-sum, automatically set by DW IC.
 */
static uint8_t multipurpose_frame[] = { 0xAD, 0x41, 0x00, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 0x46, 0x00, 0x00 };
/* Multipurpose frame with wrong PAN ID */
static uint8_t multipurpose_frame_wrong_pan_id[] = { 0xAD, 0x41, 0x00, 0xCA, 0x00, 'X', 'R', 'X', 'T', 0x46, 0x00, 0x00 };
/* Multipurpose frame with sequence number suppressed */
static uint8_t multipurpose_frame_seq_no_suppressed[] = { 0xAD, 0x46, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 0x46, 0x00, 0x00 };
/* Multipurpose frame with long addresses */
static uint8_t multipurpose_frame_long_addrs[]
    = { 0xFD, 0x41, 0x00, 0xCA, 0xDE, 'X', 'R', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 'X', 'T', 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x46, 0x00, 0x00 };
/* Short multipurpose frame */
static uint8_t multipurpose_short_frame[] = { 0xA5, 'X', 'R', 'X', 'T', 0x46, 0x00, 0x00 };

/*
 * The frame in this example is a fragment frame encoded as per the IEEE 802.15.4-2015 standard.
 * It is composed of the following fields:
 *     - byte 0/1: Fragment Header (0x020E)
 *       - bits 0-2: Frame Type: 110 - Fragmented packet
 *       - bits 3-9: Transaction Identifier (TID): 0000001
 *       - bits 10-15: Fragment Number: 000001
 *     - byte 2: Fragment Data: 0x01
 *     - byte 3/4: frame check-sum, automatically set by DW IC.
 */
static uint8_t fragment_frame[] = { 0x0E, 0x02, 0x01, 0x00, 0x00 };

/*
 * The frame in this example is an extended frame encoded as per the IEEE 802.15.4-2015 standard.
 * It is composed of the following fields:
 *     - byte 0/2: frame control (0x8861 to indicate a data frame using 16-bit addressing and requesting ACK).
 *       - bits 0-2: Frame Type: 111 - Extended frame
 *       - bits 3-5: Extended Frame Format: 001 - Reserved
 *       - bits 6-24: Extended Frame Payload: 1111 1111 11
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: frame check-sum, automatically set by DW IC.
 */
static uint8_t extended_frame[] = { 0xCF, 0xFF, 0x00, 0x00, 0x00 };

/* Index to access the sequence number and frame control fields in frames sent and received. */
#define FRAME_FC_IDX 0
#define FRAME_SN_IDX 2
/* ACK frame control value. */
#define ACK_FC_0 0x02
#define ACK_FC_1 0x00

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

/* Receive response timeout, expressed in UWB microseconds (UUS, 1 uus = 512/499.2 us). See NOTE 3 below. */
#define RX_RESP_TO_UUS 2200

/* Number of frames to send to rx device */
#define TX_FRAME_COUNT 20

/* Buffer to store received frame. See NOTE 4 below. */
#define ACK_FRAME_LEN 5
static uint8_t rx_buffer[ACK_FRAME_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16_t frame_len = 0;

/* ACK status for last transmitted frame. */
static int tx_frame_acked = 0;

/* Counters of frames sent, frames ACKed and frame retransmissions. See NOTE 1 below. */
static uint32_t tx_frame_nb = 0;
static uint32_t tx_frame_ack_nb = 0;
static uint32_t tx_frame_retry_nb = 0;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 5 below. */
extern dwt_txconfig_t txconfig_options;

/*
 * Send the specified frame
 */
void send_frames(uint8_t *frame, uint32_t frameSize, uint32_t count)
{
    uint8_t tx_count = 0;
    tx_frame_nb = tx_frame_ack_nb = tx_frame_retry_nb = tx_frame_acked = 0;
    tx_count = 0;
    /* Next the program will send 'count' data frames and check how many ACKs are received. */
    while (tx_count < count)
    {
        /* TESTING BREAKPOINT LOCATION #1 */

        /* Write frame data to DW IC and prepare transmission. See NOTE 7 below.*/
        dwt_writetxdata(frameSize, frame, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(frameSize, 0, 0); /* Zero offset in TX buffer, no ranging. */

        /* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* We assume that the transmission is achieved normally, now poll for reception of a frame or error/timeout. See NOTE 8 below. */
        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        {
            /* Clear good RX frame event in the DW IC status register. */
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

            /* A frame has been received, check frame length is correct for ACK, then read and verify the ACK. */
            frame_len = dwt_getframelength();
            if (frame_len == ACK_FRAME_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                /* Check if it is the expected ACK. */
                if ((rx_buffer[FRAME_FC_IDX] == ACK_FC_0) && (rx_buffer[FRAME_FC_IDX + 1] == ACK_FC_1) && (rx_buffer[FRAME_SN_IDX] == frame[FRAME_SN_IDX]))
                {
                    tx_frame_acked = 1;
                }
            }
        }
        else
        {
            /* Clear RX error/timeout events in the DW IC status register. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        /* Update number of frames sent. */
        tx_frame_nb++;
        tx_count++;

        if (tx_frame_acked)
        {
            /* Execute a delay between transmissions. See NOTE 6 below. */
            Sleep(TX_DELAY_MS);

            /* Increment the sent frame sequence number (modulo 256). */
            frame[FRAME_SN_IDX]++;

            /* Update number of frames acknowledged. */
            tx_frame_ack_nb++;
        }
        else
        {
            Sleep(TX_DELAY_MS / 5);
            /* Update number of retransmissions. */
            tx_frame_retry_nb++;
        }

        /* Reset acknowledged frame flag. */
        tx_frame_acked = 0;
    }
}

/**
 * Application entry point.
 */
int frame_filtering_tx(void)
{
    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Ti-me needed for DW3000 to start up (transition from INIT_RC to IDLE_RC

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) { };
    }

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure DW IC. See NOTE 11 below. */
    dwt_configure(&config);

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Set delay to turn reception on immediately after transmission of the frame. See NOTE 6 below. */
    dwt_setrxaftertxdelay(0);

    /* Set RX frame timeout for the response. */
    dwt_setrxtimeout(RX_RESP_TO_UUS);
    /* can enable TX/RX states output on GPIOs 5 and 6 to help debug */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    /* First the program will send 100 beacon frames and check how many ACKs are received. */
    send_frames(beacon_frame, sizeof(beacon_frame), TX_FRAME_COUNT);
    send_frames(beacon_frame_wrong_pan_id, sizeof(beacon_frame_wrong_pan_id), TX_FRAME_COUNT);
    send_frames(beacon_frame_long_addrs, sizeof(beacon_frame_long_addrs), TX_FRAME_COUNT);

    send_frames(enchanced_beacon_frame, sizeof(enchanced_beacon_frame), TX_FRAME_COUNT);
    send_frames(enchanced_beacon_frame_wrong_pan_id, sizeof(enchanced_beacon_frame_wrong_pan_id), TX_FRAME_COUNT);
    send_frames(enhanced_beacon_frame_seq_no_suppressed, sizeof(enhanced_beacon_frame_seq_no_suppressed), TX_FRAME_COUNT);
    send_frames(enhanced_beacon_frame_long_addrs, sizeof(enhanced_beacon_frame_long_addrs), TX_FRAME_COUNT);

    /* Next the program will send 100 data frames and check how many ACKs are received. */
    send_frames(data_frame, sizeof(data_frame), TX_FRAME_COUNT);
    send_frames(data_frame_wrong_pan_id, sizeof(data_frame_wrong_pan_id), TX_FRAME_COUNT);
    send_frames(data_frame_seq_no_suppressed, sizeof(data_frame_seq_no_suppressed), TX_FRAME_COUNT);
    send_frames(data_frame_long_addrs, sizeof(data_frame_long_addrs), TX_FRAME_COUNT);
    /* Next send 100 ACK frames (usually these are sent automatically, but we send them manually for this test case). */
    send_frames(imm_ack_frame, sizeof(imm_ack_frame), TX_FRAME_COUNT);

    send_frames(enhanced_ack_frame, sizeof(enhanced_ack_frame), TX_FRAME_COUNT);
    send_frames(enhanced_ack_frame_wrong_pan_id, sizeof(enhanced_ack_frame_wrong_pan_id), TX_FRAME_COUNT);
    send_frames(enhanced_ack_frame_seq_no_suppressed, sizeof(enhanced_ack_frame_seq_no_suppressed), TX_FRAME_COUNT);
    send_frames(enhanced_ack_frame_long_addrs, sizeof(enhanced_ack_frame_long_addrs), TX_FRAME_COUNT);

    /* Next the program will send 100 mac frames and check how many ACKs are received. */
    send_frames(mac_frame, sizeof(mac_frame), TX_FRAME_COUNT);
    send_frames(mac_frame_wrong_pan_id, sizeof(mac_frame_wrong_pan_id), TX_FRAME_COUNT);
    send_frames(mac_frame_seq_no_suppressed, sizeof(mac_frame_seq_no_suppressed), TX_FRAME_COUNT);
    send_frames(mac_frame_long_addrs, sizeof(mac_frame_long_addrs), TX_FRAME_COUNT);
    /* Next the program will send 100 reserved frames and check how many ACKs are received. */
    send_frames(reserved_frame, sizeof(reserved_frame), TX_FRAME_COUNT);
    send_frames(reserved_frame_wrong_pan_id, sizeof(reserved_frame_wrong_pan_id), TX_FRAME_COUNT);
    send_frames(reserved_frame_seq_no_suppressed, sizeof(reserved_frame_seq_no_suppressed), TX_FRAME_COUNT);
    send_frames(reserved_frame_long_addrs, sizeof(reserved_frame_long_addrs), TX_FRAME_COUNT);
    /* Next the program will send 100 multipurpose frames and check how many ACKs are received. */
    send_frames(multipurpose_frame, sizeof(multipurpose_frame), TX_FRAME_COUNT);
    send_frames(multipurpose_frame_wrong_pan_id, sizeof(multipurpose_frame_wrong_pan_id), TX_FRAME_COUNT);
    send_frames(multipurpose_frame_seq_no_suppressed, sizeof(multipurpose_frame_seq_no_suppressed), TX_FRAME_COUNT);
    send_frames(multipurpose_frame_long_addrs, sizeof(multipurpose_frame_long_addrs), TX_FRAME_COUNT);
    send_frames(multipurpose_short_frame, sizeof(multipurpose_short_frame), TX_FRAME_COUNT);
    /* Next the program will send 100 fragment frames. */
    send_frames(fragment_frame, sizeof(fragment_frame), TX_FRAME_COUNT);
    /* Next the program will send 100 multipurpose frames and check how many ACKs are received. */
    send_frames(extended_frame, sizeof(extended_frame), TX_FRAME_COUNT);

    return 0;
}
#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. This example can be turned into a high speed data transfer test by removing the delay executed between two successful transmissions. The
 *    communication configuration and MAC payload size in the message can be changed to see the effect of the different parameters on the throughput
 *    (which can be computed using the different counters provided in the application). For example using the debugger to stop at the start of the
 *    while loop, and then timing from the "GO" for a few minutes before breaking in again, and examining the frame counters.
 * 2. Source and destination addresses are hard coded constants to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW IC unique ID by combining the Lot ID & Part Number values programmed into the DW IC
 *    during its manufacture. However there is no guarantee this will not conflict with someone else's implementation. We recommended that customers
 *    buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW IC User Manual.
 * 3. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive a complete ACK frame sent by the "ACK DATA RX" example
 *    at the 110k data rate used (around 2 ms).
 * 4. In this example, receive buffer is set to the exact size of the only frame we want to handle but 802.15.4 UWB standard maximum frame length is
 *    127 bytes. DW IC also supports an extended frame length (up to 1023 bytes long) mode which is not used in this example..
 * 5. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 6. TX to RX delay is set to 0 to activate reception immediately after transmission, as the companion "ACK DATA RX" example is configured to send
 *    the ACK immediately after reception of the frame sent by this example application.
 * 7. dwt_writetxdata() takes the full size of tx_msg as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our tx_msg could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts".
 * 9. MAC commands are sent as part of the MAC frame format. In this example, the transmitting device sends a '0x04' which is a 'Data Request command'
 *     as defined by the IEEE 802.15.4-2015 standard. Please see Sections 7.2 and 7.5 of that document for more information.
 * 10. When using the reserved frame format, the 'data' field is solely used for testing purposes (0x46 - Byte 9). It has no real function and is not
 *     defined in the IEEE 802.15.4-2015 standard. This frame type is reserved in case it is required for another use in the future (or proprietary use
 *     in applications). The receiver DW3000 device will simply check against it as an added check to see if the frame has been received correctly.
 * 11. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/
