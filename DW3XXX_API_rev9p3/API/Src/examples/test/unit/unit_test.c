/*! ----------------------------------------------------------------------------
 *  @file    unit_test.c
 *  @brief   DW1000 API unit test main
 *
 *           Unit testing of a number of functions not exercised by the examples or other functional tests. Once all tests are run, the general result
 *           (FAIL or OK) is displayed on EVB1000's LCD screen. To have more information about which test is failing, the user can use a debugger to
 *           access the variable storing the result of each sub-test.
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */
#include <config_options.h>
#include <deca_device_api.h>
#include <port.h>
#include <shared_functions.h>
#include "deca_probe_interface.h"
#include "deca_private.h"

extern void test_run_info(unsigned char *data);

/* String Size for STDOUT buffer*/
#define STR_SIZE 256

/* Test fail bitfield definition. */
#define TEST_FAIL_SYS_TIME  0x01UL
#define TEST_FAIL_WAKE_UP   0x02UL
#define TEST_FAIL_TX_RX_ERR 0x04UL
#define TEST_FAIL_FEAT_CTRL 0x08UL
#define TEST_FAIL_PWR_COMP  0x10UL
#define TEST_FAIL_VTBAT     0x20UL
#define TEST_FAIL_LOCPTR    0x40UL
#define TEST_FAIL_AND_OR    0x80UL
#define TEST_FAIL_TXP_ADJ   0x100UL

#define EUI_64_LO_ID    0x4
#define SYS_TIME_ID     0x1c
#define DX_TIME_ID      0x2c
#define DREF_TIME_ID    0x30
#define EVC_COUNT1_ID   0xf0008
#define EVC_COUNT4_ID   0xf0014
#define SYS_STATE_LO_ID 0xf0030

/* Bitfield used to store the result of the different tests run.
 * Can be examined using a debugger to have more information on which test(s) is(are) failing when LCD displays "TEST FAIL" */
static uint32_t test_fail = 0UL;

extern void dwt_readsystime(uint8_t *timestamp);
extern void make_very_short_wakeup_io(void);
extern void aes_sts_check(void);


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn test_sys_time()
 *
 * @brief This is used to test functions related to system time management
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
static void test_sys_time(void)
{
    /* Time difference between two consecutive read should not be more than this value.
     * Expressed in multiples of 256 device time units. This is 3000 * 256 / 63898 ~= 12 �s. */
    const uint32_t time_max_diff = 3000;

    uint8_t sys_time[5];
    uint32_t sys_time_converted_1, sys_time_32_1, sys_time_converted_2, sys_time_32_2;
    int fail = 0;

    /* Read system time using the two different functions. */
    dwt_readsystime(sys_time);
    sys_time_32_1 = dwt_readsystimestamphi32();

    /* Check that the result from both functions are consistent with each other. */
    sys_time_converted_1 = (sys_time[4] << 24) + (sys_time[3] << 16) + (sys_time[2] << 8) + sys_time[1];
    if ((sys_time_32_1 - sys_time_converted_1) > time_max_diff)
    {
        fail = 1;
    }

    /* Read system time again. */
    dwt_readsystime(sys_time);
    sys_time_32_2 = dwt_readsystimestamphi32();

    /* Check that results are still consistent and that time has evolved since first reading. */
    sys_time_converted_2 = (sys_time[4] << 24) + (sys_time[3] << 16) + (sys_time[2] << 8) + sys_time[1];
    if ((sys_time_32_2 - sys_time_converted_2) > time_max_diff)
    {
        fail = 1;
    }
    if ((int32_t)(sys_time_converted_2 - sys_time_converted_1) <= 0)
    {
        fail = 1;
    }

    if (fail)
    {
        test_fail |= TEST_FAIL_SYS_TIME;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn test_wake_up()
 *
 * @brief This is used to test SPI wake up function
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
static void test_wake_up(void)
{
    int ret;
    int fail = 0;

    /* Put the DW3000 to sleep. */
    dwt_configuresleep(0, DWT_WAKE_CSN | DWT_SLP_EN);
    dwt_entersleep(DWT_DW_IDLE);

    /* Try to wake DW3000 with a length deliberately too short. */
    make_very_short_wakeup_io();
    ret = dwt_check_dev_id();
    if (ret != DWT_ERROR)
    {
        fail = 1;
    }

    /* Now really wake the DW3000 up. */
    dwt_wakeup_ic();
    ret = dwt_check_dev_id();
    if (ret != DWT_SUCCESS)
    {
        fail = 1;
    }

    if (fail)
    {
        test_fail |= TEST_FAIL_WAKE_UP;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn test_tx_rx_err()
 *
 * @brief This is used to test error case when starting TX or activating RX
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
static uint8_t tx_msg[] = { 0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E' };

static void test_tx_rx_err(void)
{
    uint32_t sys_time;
    int ret, no_reponse;
    int fail = 0;
    int tx_cmd_test = 0;

    /* Check that we get an error when we try to write data out of the TX buffer memory. */
    ret = dwt_writetxdata(1200, NULL, 0);
    if (ret != DWT_ERROR)
    {
        fail = 1;
    }
    ret = dwt_writetxdata(0, NULL, 1200);
    if (ret != DWT_ERROR)
    {
        fail = 1;
    }
    ret = dwt_writetxdata(600, NULL, 600);
    if (ret != DWT_ERROR)
    {
        fail = 1;
    }

    /* Read system time and set time read for delayed TX/RX so that it is in the past for next TX/RX. */
    sys_time = dwt_readsystimestamphi32();
    dwt_setdelayedtrxtime(sys_time);

    /* Check that we get an error when trying to do delayed TX/RX with a date in the past. */
    ret = dwt_starttx(DWT_START_TX_DELAYED);
    if (ret != DWT_ERROR)
    {
        fail = 1;
    }
    ret = dwt_rxenable(DWT_START_RX_DELAYED | DWT_IDLE_ON_DLY_ERR);
    if (ret != DWT_ERROR)
    {
        fail = 1;
    }

    /* Write frame data to DW IC and prepare transmission. See NOTE 3 below.*/
    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg) + FCS_LEN, 0, 0); /* Zero offset in TX buffer, no ranging. */

    // Test Fast Commands: TX, RX and their various flavors
    // Test: DWT_START_TX_IMMEDIATE, DWT_START_TX_DLY_TS, DWT_START_TX_DLY_REF, DWT_START_TX_IMMEDIATE+W4R, DWT_START_TX_DLY_REF+W4R
    //       DWT_START_RX_DLY_REF, DWT_START_RX_DLY_RS, DWT_START_RX_DLY_TS, DWT_START_RX_IMMEDIATE, DWT_START_RX_DELAYED
    {
        uint32_t status, states[50], systime;
        uint32_t tx_time1, tx_time2, rx_time, rx_time_dref, /*rx_time1, */ rx_time2, rx_time2_dref;
        uint32_t dx_time = 0x20000;
        uint32_t systime_dref = 0;
        int i, err;

        int tx_count, rx_count;

        dwt_configeventcounters(1); // enable and clear

        for (i = 0; i < 50; i++)
        {
            states[i] = 0;
        }

        dwt_starttx(DWT_START_TX_IMMEDIATE);
        tx_cmd_test = 0x1;
        i = 0;
        while (!((status = dwt_readsysstatuslo()) & (DWT_INT_TXFRS_BIT_MASK)))
        {
            dwt_readfromdevice(SYS_STATE_LO_ID, 0, 4, (uint8_t *)&states[i]);

            if (i++ == 50)
                i = 0;
        }

        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        tx_time1 = dwt_readtxtimestamphi32();

        dwt_writetodevice(DX_TIME_ID, 0, 4, (uint8_t *)&dx_time);

        dwt_starttx(DWT_START_TX_DLY_TS);
        while (!((status = dwt_readsysstatuslo()) & (DWT_INT_TXFRS_BIT_MASK)))
        {
            dwt_readfromdevice(SYS_STATE_LO_ID, 0, 4, (uint8_t *)&states[i]);

            if (i++ == 50)
                i = 0;
        }
        tx_cmd_test |= 0x2;
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        tx_time2 = dwt_readtxtimestamphi32();

        if (tx_time2 != (tx_time1 + dx_time))
        {
            fail = 1;
        }

        dwt_readfromdevice(SYS_TIME_ID, 0, 4, (uint8_t *)&systime);
        systime_dref = (systime + 0x10000);
        dwt_writetodevice(DREF_TIME_ID, 0, 4, (uint8_t *)&systime_dref);
        dwt_starttx(DWT_START_TX_DLY_REF);
        tx_time1 = systime + 0x10000;
        tx_cmd_test |= 0x4;
        while (!((status = dwt_readsysstatuslo()) & (DWT_INT_TXFRS_BIT_MASK)))
        {
            dwt_readfromdevice(SYS_STATE_LO_ID, 0, 4, (uint8_t *)&states[i]);

            if (i++ == 50)
                i = 0;
        }

        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        tx_time2 = dwt_readtxtimestamphi32();

        if (tx_time2 != (tx_time1 + dx_time))
        {
            fail = 1;
        }

        dwt_setrxtimeout(0); // clear RX timeout
        // Need to enable other device to TX frames (as we are waiting for response here)
        // NOTE: as the default SFD timeout is set to 80 symbols - the far side should send Ipatov Preamble of 64 or less.

        //!!!!! place a break point here (on the while(no_response) below) - and start a tag on PC DecaRanging
        // then place a break point on if(!(tx_count >= 6) || !(rx_count == 7))

        no_reponse = 1;
        while (no_reponse)
        {
            dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
            tx_cmd_test |= 0x8;
            i = 0;
            while (!((status = dwt_readsysstatuslo()) & ((DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO))))
            {
                dwt_readfromdevice(SYS_STATE_LO_ID, 0, 4, (uint8_t *)states[i]);

                if (i++ == 50)
                    i = 0;
            }

            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
            tx_time2 = dwt_readtxtimestamphi32();

            if (status & DWT_INT_RXFCG_BIT_MASK)
            {
                /*rx_time1 = dwt_readrxtimestamphi32();*/
                err = dwt_rxenable(DWT_START_RX_DLY_RS);
                i = 0;
                status = dwt_readsysstatuslo();
                if (err != -1)
                {
                    while (!((status = dwt_readsysstatuslo()) & ((DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO))))
                    {
                        dwt_readfromdevice(SYS_STATE_LO_ID, 0, 4, (uint8_t *)&states[i]);

                        if (i++ == 50)
                            i = 0;
                    }
                }
                else
                {
                    continue;
                }

                dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
                if (status & DWT_INT_RXFCG_BIT_MASK)
                {
                    rx_time = dwt_readrxtimestamphi32();
                    rx_time_dref = rx_time + 0x20000;
                    dwt_writetodevice(DREF_TIME_ID, 0, 4, (uint8_t *)&rx_time_dref); // (0x10000 ~ 262 us)
                    err = dwt_rxenable(DWT_START_RX_DLY_REF);
                    status = dwt_readsysstatuslo();
                    i = 0;
                    if (err != -1)
                        while (!((status = dwt_readsysstatuslo()) & ((DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO))))
                        {
                            dwt_readfromdevice(SYS_STATE_LO_ID, 0, 4, (uint8_t *)&states[i]);

                            if (i++ == 50)
                                i = 0;
                        }

                    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
                    if (status & DWT_INT_RXFCG_BIT_MASK)
                    {
                        rx_time2 = dwt_readrxtimestamphi32();
                        rx_time2_dref = rx_time2 + 0x20000;
                        dwt_writetodevice(DREF_TIME_ID, 0, 4, (uint8_t *)&rx_time2_dref); // (0x10000 ~ 262 us)
                        err = dwt_starttx(DWT_START_TX_DLY_REF | DWT_RESPONSE_EXPECTED);
                        status = dwt_readsysstatuslo();
                        i = 0;
                        if (!err)
                            while (!((status = dwt_readsysstatuslo()) & ((DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO))))
                            {
                                dwt_readfromdevice(SYS_STATE_LO_ID, 0, 4, (uint8_t *)&states[i]);

                                if (i++ == 50)
                                    i = 0;
                            }
                        tx_cmd_test |= 0x10;

                        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
                        if (status & DWT_INT_RXFCG_BIT_MASK)
                        {
                            dwt_starttx(DWT_START_TX_IMMEDIATE);
                            uint32_t dx_time = 0x20000;
                            dwt_writetodevice(DX_TIME_ID, 0, 4, (uint8_t *)&dx_time);
                            while (!((status = dwt_readsysstatuslo()) & ((DWT_INT_TXFRS_BIT_MASK)))) { }
                            tx_cmd_test |= 0x20;
                            err = dwt_rxenable(DWT_START_RX_DLY_TS);
                            i = 0;
                            status = dwt_readsysstatuslo();
                            if (err != -1)
                                while (!((status = dwt_readsysstatuslo()) & ((DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO))))
                                {
                                    dwt_readfromdevice(SYS_STATE_LO_ID, 0, 4, (uint8_t *)&states[i]);

                                    if (i++ == 50)
                                        i = 0;
                                }

                            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
                            if (status & DWT_INT_RXFCG_BIT_MASK)
                            {
                                rx_time2 = dwt_readrxtimestamphi32();
                                uint32_t rx_time2_dx = rx_time2 + 0x20000;
                                dwt_writetodevice(DX_TIME_ID, 0, 4, (uint8_t *)&rx_time2_dx);
                                err = dwt_rxenable(DWT_START_RX_DELAYED);
                                i = 0;
                                status = dwt_readsysstatuslo();
                                if (err != -1)
                                    while (!((status = dwt_readsysstatuslo()) & ((DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO))))
                                    {
                                        dwt_readfromdevice(SYS_STATE_LO_ID, 0, 4, (uint8_t *)&states[i]);

                                        if (i++ == 50)
                                            i = 0;
                                    }

                                dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
                                if (status & DWT_INT_RXFCG_BIT_MASK)
                                {
                                    err = dwt_rxenable(DWT_START_RX_IMMEDIATE);
                                    i = 0;
                                    status = dwt_readsysstatuslo();
                                    if (err != -1)
                                        while (!((status = dwt_readsysstatuslo()) & ((DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO))))
                                        {
                                            dwt_readfromdevice(SYS_STATE_LO_ID, 0, 4, (uint8_t *)&states[i]);

                                            if (i++ == 50)
                                                i = 0;
                                        }

                                    if (status & DWT_INT_RXFCG_BIT_MASK)
                                    {
                                        no_reponse = 0;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                // got error - need to re-send
                Sleep(100); // add some delay
            }
        }

        dwt_readfromdevice(EVC_COUNT4_ID, 0, 4, (uint8_t *)&tx_count);
        tx_count >>= 16;
        dwt_readfromdevice(EVC_COUNT1_ID, 0, 4, (uint8_t *)&rx_count);
        rx_count &= 0xFFF;

        if (!(tx_count >= 6) || !(rx_count == 7))
        {
            fail = 1;
        }
        // Just using 'states' in order to suppress a -Wunused-but-set-variable
        memset(states, sizeof(states), sizeof(uint32_t));
    }

    if (fail)
    {
        test_fail |= TEST_FAIL_TX_RX_ERR;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn test_feature_control()
 *
 * @brief This is used to test feature activation/deactivation
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
static void test_feature_control(void)
{
    //    int fail = 0;
    //
    //    /* Check that activation/deactivation of the following features is correctly performed. */
    //    dwt_setsmarttxpower(0);
    //    if (!(dwt_read32bitreg(SYS_CFG_ID) & SYS_CFG_DIS_STXP))
    //    {
    //        fail = 1;
    //    }
    //    dwt_setsmarttxpower(1);
    //    if (dwt_read32bitreg(SYS_CFG_ID) & SYS_CFG_DIS_STXP)
    //    {
    //        fail = 1;
    //    }
    //
    //    dwt_setdblrxbuffmode(0);
    //    if (!(dwt_read32bitreg(SYS_CFG_ID) & SYS_CFG_DIS_DRXB))
    //    {
    //        fail = 1;
    //    }
    //    dwt_setdblrxbuffmode(1);
    //    if (dwt_read32bitreg(SYS_CFG_ID) & SYS_CFG_DIS_DRXB)
    //    {
    //        fail = 1;
    //    }
    //
    //    dwt_entersleepaftertx(1);
    //    if (!(dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET) & PMSC_CTRL1_ATXSLP))
    //    {
    //        fail = 1;
    //    }
    //    dwt_entersleepaftertx(0);
    //    if (dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET) & PMSC_CTRL1_ATXSLP)
    //    {
    //        fail = 1;
    //    }
    //
    //    if (fail)
    //    {
    //        test_fail |= TEST_FAIL_FEAT_CTRL;
    //    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn test_pwr_comp()
 *
 * @brief This is used to test the power compensation over temperature
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
#if 0
static void test_pwr_comp(void)
{
    uint32_t result = 0;
    int i, fail = 0;

    struct testcase {
        uint8_t channel;
        double curr_temp;
        double ref_temp;
        uint32_t txpower;
        uint32_t expected;
    };

    struct testcase testcases[5] = {
        {
            .channel = 2,
            .curr_temp = 40.0,
            .ref_temp = 20.0,
            .txpower = 0x67676767,
            .expected = 0x68686868
        },
        {
            .channel = 5,
            .curr_temp = -40.0,
            .ref_temp = 20.0,
            .txpower = 0x67676767,
            .expected = 0x85858585
        },
        {
            .channel = 2,
            .curr_temp = 65.0,
            .ref_temp = 20.0,
            .txpower = 0x67676767,
            .expected = 0x6A6A6A6A
        },
        {
            .channel = 5,
            .curr_temp = 105.0,
            .ref_temp = 20.0,
            .txpower = 0x67676767,
            .expected = 0x72727272
        },
        {
            .channel = 2,
            .curr_temp = -25.0,
            .ref_temp = 20.0,
            .txpower = 0x67676767,
            .expected = 0x64646464
        }
    };

    for(i = 0; i < 5; ++i)
    {
        result = dwt_calcpowertempadj(testcases[i].channel, testcases[i].txpower, testcases[i].curr_temp, testcases[i].ref_temp);
        if(result != testcases[i].expected)
        {
            fail = 1;
        }
    }

    if(fail)
    {
        test_fail |= TEST_FAIL_PWR_COMP;
    }
}
#endif

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn test_vtbat()
 *
 * @brief This is used to read the Vmeas and Tmeas values already read in the dwt_initialise function
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
#if 0
static void test_vtbat(void)
{
    /* The Vmeas and Tmeas are programmed in the factory, thus these values should never be 0x00 or 0xFF */

    uint8_t vmeas = dwt_getvmeas();
    uint8_t tmeas = dwt_gettmeas();

    int fail = 0;

    if(((vmeas & 0xff) == 0x0) || ((vmeas & 0xff) == 0xff))
    {
        fail = 1;
    }

    if(((tmeas & 0xff) == 0x0) || ((tmeas & 0xff) == 0xff))
    {
        fail = 1;
    }

    if(fail)
    {
        test_fail |= TEST_FAIL_VTBAT;
    }

}
#endif

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn test_localdata()
 *
 * @brief This is used to test that the dwt_setlocaldataptr works correctly,
 * if index is out of range then this function should return  DWT_ERROR
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
#if 0
static void test_localdata(void)
{
    int fail = 0;
    int i = 0;

    while (i < DWT_NUM_DW_DEV)
    {
        if (dwt_setlocaldataptr(i++))
        {
            fail = 1;
        }
    }

    if (!dwt_setlocaldataptr(i))
    {
        fail = 1;
    }

    if (fail)
    {
        test_fail |= TEST_FAIL_LOCPTR;
    }
}
#endif

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn test_and_or()
 *
 * @brief This is used to test that the AND_OR spi command works correctly
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
static void test_and_or(void)
{
    int fail = 0;

    /* Test for AND_OR */
    char s[80] = { 0 };
    uint32_t tmp = 0;
    test_run_info((unsigned char *)"                 ");
    uint32_t eui_64_lo_id_buf = 0xFF00FF00;

    /* 32bit AND_OR */
    dwt_writetodevice(EUI_64_LO_ID, 0, 4, (uint8_t *)&eui_64_lo_id_buf);
    dwt_readfromdevice(EUI_64_LO_ID, 0, 4, (uint8_t *)&tmp);
    sprintf(s, "32: 0x%8lX", (unsigned long)tmp);
    test_run_info((unsigned char *)s);
    Sleep(500);
    tmp = 0;
    uint8_t write_buf[8];
    write_buf[0] = (uint8_t)0xEEFFEEFF; // &0xFF;
    write_buf[1] = (uint8_t)(0xEEFFEEFF >> 8); // &0xFF;
    write_buf[2] = (uint8_t)(0xEEFFEEFF >> 16); // &0xFF;
    write_buf[3] = (uint8_t)(0xEEFFEEFF >> 24); // &0xFF;
    write_buf[4] = (uint8_t)0x00010001; // &0xFF;
    write_buf[5] = (uint8_t)(0x00010001 >> 8); // &0xFF;
    write_buf[6] = (uint8_t)(0x00010001 >> 16); // &0xFF;
    write_buf[7] = (uint8_t)(0x00010001 >> 24); // &0xFF;
    dwt_writetodevice(EUI_64_LO_ID, 0, sizeof(write_buf), write_buf);
    dwt_readfromdevice(EUI_64_LO_ID, 0, 4, (uint8_t *)&tmp);
    if (tmp == 0xEE01EE01)
    {
        sprintf(s, "32-1: 0x%8lX", (unsigned long)tmp);
        test_run_info((unsigned char *)s);
    }
    else
    {
        fail = (1);
    }

    Sleep(1000);

    /* 16bit AND_OR */
    eui_64_lo_id_buf = 0xAAEFFF55;
    dwt_writetodevice(EUI_64_LO_ID, 4, 4, (uint8_t *)&eui_64_lo_id_buf);
    dwt_readfromdevice(EUI_64_LO_ID, 4, 4, (uint8_t *)&tmp);
    sprintf(s, "16: 0x%8lX", (unsigned long)tmp);
    test_run_info((unsigned char *)s);
    Sleep(500);
    tmp = 0;
    write_buf[0] = (uint8_t)0xF0E0; // &0xFF;
    write_buf[1] = (uint8_t)(0xF0E0 >> 8); // &0xFF;
    write_buf[2] = (uint8_t)0x0101; // &0xFF;
    write_buf[3] = (uint8_t)(0x0101 >> 8); // &0xFF;
    dwt_writetodevice(EUI_64_LO_ID, 5, 4, write_buf);
    dwt_readfromdevice(EUI_64_LO_ID, 5, 4, (uint8_t *)&tmp);
    if (tmp == 0xAAE1E155)
    {
        sprintf(s, "16-1: 0x%8lX", (unsigned long)tmp);
        test_run_info((unsigned char *)s);
    }
    else
    {
        fail = (1);
    }

    /* 8bit AND_OR */
    eui_64_lo_id_buf = 0xAA55FF55;
    dwt_writetodevice(EUI_64_LO_ID, 4, 4, (uint8_t *)&eui_64_lo_id_buf);
    dwt_readfromdevice(EUI_64_LO_ID, 4, 4, (uint8_t *)&tmp);
    sprintf(s, "8: 0x%8lX", (unsigned long)tmp);
    test_run_info((unsigned char *)s);
    Sleep(500);
    tmp = 0;
    write_buf[0] = 0xE0;
    write_buf[1] = 0x01;
    dwt_writetodevice(EUI_64_LO_ID, 5, 2, (uint8_t *)write_buf);
    dwt_readfromdevice(EUI_64_LO_ID, 4, 4, (uint8_t *)&tmp);
    if (tmp == 0xAA55E155)
    {
        sprintf(s, "8-1: 0x%8lX", (unsigned long)tmp);
        test_run_info((unsigned char *)s);
    }
    else
    {
        fail = (1);
    }

    if (fail)
    {
        test_fail |= TEST_FAIL_AND_OR;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn test_dwt_adjust_tx_power()
 *
 * @brief This is used to generate all the possible solution returned by the api dwt_adjust_tx_power.
 * This function should be used to validate dwt_adjust_tx_power after any update of the API. In particular
 * it should be verified that:
 * 1. The core loop does not lock.
 * 2. The difference between the boost and applied boost is within the MARGIN. If not, it should be understood why.
 * It is expected that for some input combination, the required boost will not be achieved, because it is out of the
 * specification of the DW3000.
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
static void test_dwt_adjust_tx_power(void)
{
    int fail = 0;
    int err;
    uint32_t ref_tx_power = 0x36363636;
    uint32_t adj_tx_power;
    uint16_t boost;
    uint16_t applied_boost;
    uint8_t txp_byte;
    uint8_t chan;
    unsigned char str[STR_SIZE];

    test_run_info((unsigned char *)"Channel, Boost, Applied_Boost, Reference_Tx_Power, Adjusted_Tx_Power \r\n");
    Sleep(100); // Delay to print of STDOUT

    chan = 5;

    for(txp_byte=0; txp_byte < 255; txp_byte++)
    {
        ref_tx_power = ((uint32_t) txp_byte) << 24 | ((uint32_t) txp_byte) << 16 | ((uint32_t) txp_byte) << 8 | ((uint32_t) txp_byte);

        // Maximum boost supported by DW3XXX device is 35dB (350).
        // If the boost parameter is > 350, then the API will apply the maximum boost.
        for(boost = 0 ; boost < 400 ; boost++)
        {
            err = dwt_adjust_tx_power(boost, ref_tx_power, chan, &adj_tx_power, &applied_boost);

            if(err==DWT_ERROR)
            {
                fail = 1;
            }

            memset(str, 0, STR_SIZE);
            snprintf((char*) str, STR_SIZE,"%d, %d, %d, %lx, %lx\r\n", chan, boost, applied_boost, ref_tx_power, adj_tx_power);
            test_run_info(str);
            Sleep(1);
        }
    }

    test_run_info((unsigned char *)"\r\nChannel, Boost, Applied_Boost, Reference_Tx_Power, Adjusted_Tx_Power \r\n");
    Sleep(100);  //Delay to print of STDOUT
    chan = 9;

    for(txp_byte=0; txp_byte < 255; txp_byte++)
    {
        ref_tx_power = ((uint32_t) txp_byte) << 24 | ((uint32_t) txp_byte) << 16 | ((uint32_t) txp_byte) << 8 | ((uint32_t) txp_byte);

        // Maximum boost supported by DW3XXX device is 35dB (350).
        // If the boost parameter is > 350, then the API will apply the maximum boost.
        for(boost = 0 ; boost < 400 ; boost++)
        {
            err = dwt_adjust_tx_power(boost, ref_tx_power, chan, &adj_tx_power, &applied_boost);

            if(err==DWT_ERROR)
            {
                fail = 1;
            }

            memset(str, 0, STR_SIZE);
            snprintf((char*) str, STR_SIZE,"%d, %d, %d, %lx, %lx\r\n", chan, boost, applied_boost, ref_tx_power, adj_tx_power);
            test_run_info(str);
            Sleep(1);
        }
    }

    if (fail)
    {
        test_fail |= TEST_FAIL_TXP_ADJ;
    }
}

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
    DWT_PHRMODE_STD, /* PHY header mode: standard, up to 127 in the payload */
    DWT_PHRRATE_STD, (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS mode*/
    DWT_STS_LEN_64, /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0 /* pdoa mode*/
};

/**
 * Application entry point.
 */
int unit_test_main(void)
{
    int ret;

    // tx_rx_aes_verification();//In case of an error the function does not return.

    aes_sts_check();

    /* Start with board specific hardware init. */
    peripherals_init();

    /* Display application name on LCD. */
    test_run_info((unsigned char *)"UNIT TEST    ");

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    // port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC
    Sleep(2);

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf);

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) { };
    }
    /* Configure DW3000. */
    dwt_configure(&config);

    // configure the tx spectrum parameters (power and PG delay)
    dwt_configuretxrf(&txconfig_options);

    test_sys_time();
    test_tx_rx_err();
    test_feature_control();
#if 0
    test_pwr_comp();
    test_vtbat();
    test_localdata();
#endif
    test_and_or();
    test_wake_up();
    test_dwt_adjust_tx_power();

    /* Display and return result of the test. */
    if (test_fail)
    {
        sprintf(dist_str, "UNIT TST FAIL%02x", (unsigned int)test_fail);
        test_run_info((unsigned char *)&dist_str);
        ret = -1;
    }
    else
    {
        test_run_info((unsigned char *)"UNIT TEST OK");
        ret = 0;
    }

    return ret;
}
