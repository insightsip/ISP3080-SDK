/*! ----------------------------------------------------------------------------
 *  @file    tx_sleep.c
 *  @brief   TX with sleep example code
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include "deca_device_api.h"
#include "deca_spi.h"
#include "example_selection.h"
#include "port.h"
#include "shared_defines.h"

#if defined(TEST_TX_SLEEP_TEST)

/* Example application name and version to display on LCD screen. */
#define APP_NAME "TX SLEEP v1.0"

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
    DWT_STS_MODE_OFF, /* Cipher disabled */
    DWT_STS_LEN_64, /* Cipher length*/
    DWT_PDOA_M0 /* PDOA mode off */
};

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     -   */
static uint8_t tx_msg[] = { 0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E' };
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

#define FRAME_LENGTH sizeof(tx_msg) + FCS_LEN // The real length that is going to be transmitted

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

/* Dummy buffer for DW IC wake-up SPI read. See NOTE 2 below. */
#define DUMMY_BUFFER_LEN 600
static uint8_t dummy_buffer[DUMMY_BUFFER_LEN];

extern int dwt_configure_x(dwt_config_t *config);
/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

/**
 * Application entry point.
 */
int tx_sleep(void)
{
    uint32_t devID, status_reg, state[55], status[50];

    /* Display application name on LCD. */
    lcd_display_str(APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 40 MHz */
    port_set_dw_ic_spi_fastrate();
start:
    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR)
    {
        lcd_display_str("INIT FAILED     ");
        while (1) { };
    }

    /* Configure DW IC. See NOTE 2 below. */
    dwt_configure(&config);

    /* Configure the TX spectrum parameters (power and PG delay) */
    dwt_configuretxrf(&txconfig_options);

    /* To help with debug or as information we can enable DW IC TX/RX states to drive LEDs
     * so that as frames are transmitted the TX LED flashes.
     * dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
     */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    // dwt_setleds(DWT_LEDS_ENABLE /*| DWT_LEDS_INIT_BLINK*/);
    dwt_setrxtimeout(1000);

    /* Configure sleep and wake-up parameters. */
    /* DWT_PGFCAL is added to make sure receiver is re-enabled on wake. */
    dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL, DWT_WAKE_CSN | DWT_WAKE_WUP | DWT_SLP_EN);

    /* Loop forever sending frames periodically. */
    while (TRUE)
    {
        int i = 0, j = 0;
        /* Write frame data to DW IC and prepare transmission. See NOTE 4 below. */
        dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. Data does not include the CRC*/
        /* In this example since the length of the transmitted frame does not change,
         * nor the other parameters of the dwt_writetxfctrl function, the
         * dwt_writetxfctrl call could be outside the main while(1) loop.
         */
        dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */

        memset(state, 0, sizeof(state));
        state[i++] = dwt_read32bitreg(SYS_STATE_LO_ID);
        /* Start transmission. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* Poll DW IC until TX frame sent event set. See NOTE 5 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it.*/
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        {
            status_reg = dwt_read32bitreg(SYS_STATUS_ID);

            if ((status_reg & SYS_STATUS_PLL_HILO) && !(status_reg & SYS_STATUS_CPLOCK)) // PLL not locked
            {
                goto start;
            }

            if (i < 20)
            {
                i++;
                state[i] = dwt_read32bitreg(SYS_STATE_LO_ID);
            }

            if ((state[i] == 0x50000)) // PMSC in TX but no TX
            {
                if (j++ == 5)
                    goto start;
            }

            if ((state[i] == 0x00000) && !(status_reg & SYS_STATUS_CPLOCK))
            {
                if (j++ == 5)
                    goto start;
            }
        }

        /* Clear TX frame sent event. */
        // dwt_writesysstatuslo(SYS_STATUS_TXFRS);

        // Sleep(200); /* If using LEDs we need to add small delay to see the TX LED blink */

        /* Put DW IC to sleep. */
        dwt_entersleep(DWT_DW_IDLE);

        /* Execute a delay between transmissions. */
        Sleep(50); // TX_DELAY_MS);

        /* Wake DW IC up. See NOTE 2 below. */
        // port_set_dw_ic_spi_slowrate();
        // dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);
        // port_set_dw_ic_spi_fastrate();
        port_wakeup_dw_ic();

        // Sleep(1); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC

        i = 0;
        while (!dwt_checkidlerc())
        {
            status[i++] = dwt_read32bitreg(SYS_STATUS_ID);

            if (i == 50)
                break;
        }

        // Sleep(1);
        if (dwt_pllpgfcal(config.chan) == DWT_SUCCESS)
        {

            // dwt_pgf_cal(1);
            // dwt_configure_x(&config);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            i = 25;
            j = 0;
            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)))
            {
                if (i < 50)
                {
                    i++;
                    state[i] = dwt_read32bitreg(SYS_STATE_LO_ID);
                }
                if ((state[i] == 0x00000) && !(status_reg & SYS_STATUS_CPLOCK))
                {
                    if (j++ == 5)
                    {
                        status_reg = dwt_read32bitreg(PLL_STATUS_ID);
                        status_reg = dwt_read32bitreg(PLL_COARSE_CODE_ID);
                        goto start;
                    }
                }
            }

            dwt_writesysstatuslo(status_reg);
        }
        else
        {
            status_reg = dwt_read32bitreg(PLL_STATUS_ID);
        }
        /* Put DW IC to sleep. */
        dwt_entersleep(DWT_DW_IDLE);

        /* Execute a delay between transmissions. */
        Sleep(50); // TX_DELAY_MS);

#if 1
        port_wakeup_dw_ic();

        // Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC
        i = 0;
        while (!dwt_checkidlerc())
        {
            status[i++] = dwt_read32bitreg(SYS_STATUS_ID);

            if (i == 50)
                break;
        };

        state[52] = dwt_read32bitreg(SYS_STATE_LO_ID);
        // re-cal the PLL
        {
            // dwt_configure(&config);
            // dwt_setidlemode(DWT_DW_IDLE); // Set the AUTO2IDLE bit so that DW3000 enters IDLE mode before switching clocks to system
            // dwt_pllpgfcal();  //the upload of configuration to AON above means we do not need to re-configure the device post sleep each time
            // status_reg = dwt_read32bitoffsetreg(PLL_STATUS_ID, 0x0);
            /*if(status_reg != DW_PLL_LOCKED)
            {
                //dwt_use_prev_code();
                goto start;
                //break;
            }*/
        }
#endif
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
 * 5. We use polled mode of operation here to keep the example as simple as possible but the TXFRS status event can be used to generate an interrupt.
 *    Please refer to DW IC User Manual for more details on "interrupts".
 ****************************************************************************************************************************************************/
