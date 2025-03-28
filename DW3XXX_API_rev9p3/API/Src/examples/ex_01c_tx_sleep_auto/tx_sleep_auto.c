/*! ----------------------------------------------------------------------------
 *  @file    tx_sleep_auto.c
 *  @brief   TX with auto sleep example code
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include "deca_probe_interface.h"
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_functions.h>

#if defined(TEST_TX_SLEEP_AUTO)

extern void test_run_info(unsigned char *data);

/* Example application name and version to display on LCD screen. */
#define APP_NAME "TX AUTO SLP v1.0"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10/11: frame check-sum, automatically set by DW IC.  */
static uint8_t tx_msg[] = { 0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0 };
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 3 below. */
extern dwt_txconfig_t txconfig_options;

/**
 * Application entry point.
 */
int tx_sleep_auto(void)
{
    uint32_t reg = 0;
    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED");
        while (1) { };
    }

    /* Configure DW IC. See NOTE 5 below. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Configure sleep and wake-up parameters. */
    /* DWT_PGFCAL is added to make sure receiver is re-enabled on wake. */
    dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL, DWT_PRES_SLEEP | DWT_WAKE_WUP | DWT_SLP_EN);

    /* Configure DW IC to automatically enter sleep mode after transmission of a frame. */
    dwt_entersleepaftertx(1);

    reg = dwt_readsysstatuslo();

    /* need to disable default interrupts, device will not go to sleep if interrupt line is high */
    dwt_setinterrupt(DWT_INT_SPIRDY_BIT_MASK, 0, DWT_DISABLE_INT);

    dwt_writesysstatuslo(reg); // clear interrupt/events

    /* Loop forever sending frames periodically. */
    while (1)
    {
        /* Write frame data to DW IC and prepare transmission. See NOTE 4 below. */
        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_msg), 0, 0);     /* Zero offset in TX buffer, no ranging. */

        /* Start transmission. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* It is not possible to access DW IC registers once it has sent the frame and gone to sleep, and therefore we do not try to poll for TX
         * frame sent, but instead simply wait sufficient time for the DW IC to wake up again before we loop back to send another frame.
         * If interrupts are enabled, (e.g. if MTXFRS bit is set in the SYS_MASK register) then the TXFRS event will cause an active interrupt and
         * prevent the DW IC from sleeping. */

        /* Execute a delay between transmissions. */
        Sleep(TX_DELAY_MS);

        /* Wake DW IC up. See NOTE 2 below. */
        dwt_wakeup_ic();

        Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

        /* Need to make sure DW IC is in IDLE_RC before proceeding */
        while (!dwt_checkidlerc()) { };

        /* Restore the required configurations on wake */
        dwt_restoreconfig(1);

        /* Increment the blink frame sequence number (modulo 256). */
        tx_msg[BLINK_FRAME_SN_IDX]++;
    }
}
#endif
/*****************************************************************************************************************************************************
 * NOTES: TODO review these
 *
 * 1. The device ID is a hard coded constant in the blink to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW IC unique ID by combining the Lot ID & Part Number values programmed into the
 *    DW IC during its manufacture. However there is no guarantee this will not conflict with someone else's implementation. We recommended that
 *    customers buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW IC User Manual.
 * 2. The chosen method for waking the DW IC up here is by maintaining SPI chip select line low for at least 500 us. This means that we need a buffer
 *    to collect the data that DW IC outputs during this dummy SPI transaction. The length of the transaction, and then the time for which the SPI
 *    chip select is held low, is determined by the buffer length given to dwt_spicswakeup() so this length must be chosen high enough so that the
 *    DW IC has enough time to wake up.
 * 3. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 4. dwt_writetxdata() takes the full size of tx_msg as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our tx_msg could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 5. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *    configuration.
 ****************************************************************************************************************************************************/
