/******************************************************************************
 * @file    drv_uwb_range.c
 * @author  Insight SiP
 * @brief   Range driver implementation
 *
 * @attention
 *    THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *    OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *    DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *    GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include "drv_uwb_range.h"
#include "deca_device_api.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"
#include "port.h"
#include "sdk_macros.h"
#include <string.h>

#define NRF_LOG_MODULE_NAME drv_uwb_range
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

extern const struct dwt_probe_s dw3000_probe_interf;

/**@brief Initialization struct for uwb sw driver.
 */
typedef struct
{
    drv_uwb_range_role_t role; /**< Role of the device */
    bool sleep_enabled;        /**< is sleep mode enabled */
    bool filter_enabled;       /**< is frame filter enabled */
    bool tx_rx_leds_enabled;   /**< are RX/TX LEDs enabled */
    uint8_t src_address[8];    /**< Address of the device */
    uint8_t dest_address[8];   /**< Address of the device to range with */
} drv_uwb_range_sw_cfg_t;

srd_msg_dlsl init_msg, resp_msg;                /**< Message buffers. */
static drv_uwb_range_evt_handler_t evt_handler; /**< Event handler called by gpiote_evt_sceduled. */
static drv_uwb_range_sw_cfg_t sw_cfg;           /**< SW configuration. */
static double m_last_range = -1.0;
static volatile uint8_t is_uwb_sleeping = 0;
static uint32_t poll_tx_timestamp_u32, poll_rx_timestamp_u32, resp_tx_timestamp_u32, resp_rx_timestamp_u32;
static uint64_t poll_rx_timestamp_u64;
static uint64_t resp_tx_timestamp_u64;
static uint32_t frame_sn; /**< Frame counter */
static uint16_t tx_ant_dly;
static uint16_t rx_ant_dly;

static dwt_config_t uwb_config = {
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
    DWT_STS_MODE_OFF, /* STS enabled */
    DWT_STS_LEN_256,  /* Cipher length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode 3 */
};

static dwt_txconfig_t tx_config = {
    DEFAULT_CH5_PGDLY,  /* PG delay. */
    DEFAULT_CH5_PWR,    /* TX power. */
    0x0                 /* PG count. */
};

static uint64_t get_rx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;

    dwt_readrxtimestamp(ts_tab, 0);
    for (i = 4; i > -1; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }

    return ts;
}

static uint64_t get_tx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;

    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i > -1; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }

    return ts;
}

static uint64_t get_sys_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;

    dwt_readsystime(ts_tab);
    for (i = 4; i > -1; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }

    return ts;
}

static void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts) {
    int i;

    *ts = 0;
    for (i = 0; i < 4; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

static void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts) {
    int i;

    for (i = 0; i < 4; i++) {
        ts_field[i] = (ts >> (i * 8)) & 0xFF;
    }
}

static uint32_t uwb_wake_up(void) {
    uint16_t wake_up_cpt = 0;

    dwt_wakeup_ic();

    /* Wait for device to wake up */
    while (is_uwb_sleeping) {
        nrf_delay_us(10);
        wake_up_cpt++;
        if (wake_up_cpt >= 200) {
            NRF_LOG_ERROR("uwb_wake_up failed");
            return NRF_ERROR_INTERNAL;
        }
    };

    return NRF_SUCCESS;
}

static void uwb_sleep(void) {
    dwt_entersleep(DWT_DW_IDLE_RC);
    is_uwb_sleeping = 1;
}

/**@brief TX done event
 */
static void cb_tx_done(const dwt_cb_data_t *txd) {
    NRF_LOG_DEBUG("cb_tx_done event received");

    switch (sw_cfg.role) {
    case TWR_RESPONDER:
        m_last_range = -1;
        break;

    case TWR_INITIATOR:
        break;

    default:
        break;
    }
}

/**@brief RX done event
 */
static void cb_rx_done(const dwt_cb_data_t *rxd) {
    srd_msg_dlsl rxmsg_ll;
    uint16_t rx_msg_length;

    NRF_LOG_DEBUG("cb_rx_done event received");

    // Read Data Frame
    rx_msg_length = rxd->datalength;
    dwt_readrxdata((uint8_t *)&rxmsg_ll, rx_msg_length, 0);

    // Check frame control bytes - must be 0x41dc
    if (rxmsg_ll.frameCtrl[0] != 0x41 || rxmsg_ll.frameCtrl[1] != 0xdc) {
        // unexpected frame control
        if (sw_cfg.role == TWR_RESPONDER) {
            //immediate rx enable
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        } else {
            //put device into low power mode
            if (sw_cfg.sleep_enabled)
                uwb_sleep();
        }
        return;
    }

    // Action depending on the role.....
    switch (sw_cfg.role) {
    case TWR_INITIATOR: {
        if (rxmsg_ll.messageData[0] == SIMPLE_MSG_ANCH_RESP) {
            int32_t rtd_init, rtd_resp;
            float clock_offset_ratio;
            double tof;

            // Retrieve poll transmission and response reception timestamps.
            poll_rx_timestamp_u32 = dwt_readrxtimestamplo32(0);
            poll_tx_timestamp_u32 = dwt_readtxtimestamplo32();

            /* Read carrier integrator value and calculate clock offset ratio */
            clock_offset_ratio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

            // Go to sleep
            if (sw_cfg.sleep_enabled) {
                uwb_sleep();
            }

            // Get timestamps embedded in response message
            resp_msg_get_ts(&rxmsg_ll.messageData[POLL_RX_TS], &resp_rx_timestamp_u32);
            resp_msg_get_ts(&rxmsg_ll.messageData[RESP_TX_TS], &resp_tx_timestamp_u32);

            /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
            rtd_init = poll_rx_timestamp_u32 - poll_tx_timestamp_u32;
            rtd_resp = resp_tx_timestamp_u32 - resp_rx_timestamp_u32;
            tof = (((rtd_init - rtd_resp * (1.0f - clock_offset_ratio)) / 2.0F) * DWT_TIME_UNITS);
            m_last_range = tof * SPEED_OF_LIGHT;

            // Generate data event
            if (evt_handler != NULL) {
                drv_uwb_range_evt_t evt;
                evt.type = DRV_RANGE_EVT_DATA;
                evt.range = m_last_range;
                evt_handler(&evt);
            }
        } else // No expected message received
        {
            // Go to sleep
            if (sw_cfg.sleep_enabled) {
                uwb_sleep();
            }
        }
    } break;

    case TWR_RESPONDER: {
        if (rxmsg_ll.messageData[0] == SIMPLE_MSG_TAG_POLL) {
            uint32_t resp_tx_time;
            int ret;

            /* Retrieve poll reception timestamp. */
            poll_rx_timestamp_u64 = get_rx_timestamp_u64();

            /* Compute response message transmission time. */
            resp_tx_time = (poll_rx_timestamp_u64 + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);

            /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
            resp_tx_timestamp_u64 = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + tx_ant_dly;

            // Prepare and send ANCHOR RESP msg to TWR_INITIATOR
            resp_msg.seqNum = frame_sn++;
            resp_msg_set_ts(&resp_msg.messageData[POLL_RX_TS], poll_rx_timestamp_u64); // Poll message reception timestamp
            resp_msg_set_ts(&resp_msg.messageData[RESP_TX_TS], resp_tx_timestamp_u64); // Response message transmission timestamp

            uint16_t length = SIMPLE_MSG_ANCH_RESP_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;

            // Write the frame data
            dwt_writetxdata(length, (uint8_t *)&resp_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(length, 0, 1);                   /* Zero offset in TX buffer, ranging. */
            // Send frame
            ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
            if (ret == DWT_ERROR) {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                NRF_LOG_ERROR("dwt_starttx failed");
            }

            // Now we can check if there is a valid Range in the payload
            if (rx_msg_length == (SIMPLE_MSG_TAG_POLL_WITH_RG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)) {
                memcpy(&m_last_range, &rxmsg_ll.messageData[POLL_MSG_TOF_POS], 8);

                // Generate data event
                if ((evt_handler != NULL) && (m_last_range != -1)) {
                    drv_uwb_range_evt_t evt;
                    evt.type = DRV_RANGE_EVT_DATA;
                    evt.range = m_last_range;
                    evt_handler(&evt);
                }
            }
        } else { // No expected message received
            //immediate rx enable
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
    } break;

    default:
        break;
    }
}

/**@brief RX timeout event
 */
static void cb_rx_to(const dwt_cb_data_t *rxd) {
    NRF_LOG_DEBUG("cb_rx_to event received");

    switch (sw_cfg.role) {
    case TWR_INITIATOR:
        // go to sleep mode
        if (sw_cfg.sleep_enabled) {
            uwb_sleep();
        }

        // generate timeout event
        if (evt_handler != NULL) {
            drv_uwb_range_evt_t evt;
            evt.type = DRV_RANGE_EVT_TIMEOUT;
            evt_handler(&evt);
        }
        break;

    case TWR_RESPONDER:
        //immediate rx enable
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        break;

    default:
        break;
    }
}

/**@brief RX error event
 */
static void cb_rx_err(const dwt_cb_data_t *rxd) {
    NRF_LOG_DEBUG("cb_rx_err event received");

    switch (sw_cfg.role) {
    case TWR_INITIATOR:
        // go to sleep mode
        if (sw_cfg.sleep_enabled) {
            uwb_sleep();
        }

        // generate event
        if (evt_handler != NULL) {
            drv_uwb_range_evt_t evt;
            evt.type = DRV_RANGE_EVT_ERROR;
            evt_handler(&evt);
        }

        break;

    case TWR_RESPONDER:
        //immediate rx enable
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        break;

    default:
        break;
    }
}

/**@brief SPI ready event
 */
static void cb_spi_ready(const dwt_cb_data_t *cb_data) {
    (void)cb_data;
    /* Need to make sure DW IC is in IDLE_RC before proceeding */
    while (!dwt_checkidlerc()) {
    };

    /* Restore the required configurations on wake */
    dwt_restoreconfig(1);

    //set EUI as it will not be preserved unless the EUI is programmed and loaded from NVM
    dwt_seteui(sw_cfg.src_address);

    is_uwb_sleeping = 0; // device is awake
}

uint32_t drv_uwb_range_init(drv_uwb_range_init_t *p_params) {
    uint32_t err_code;
    uint8_t channel;
    uint32_t otp_memory[OTP_MEMORY_MAX_ADDR];
    dwt_callbacks_s cbs = {NULL};

    VERIFY_PARAM_NOT_NULL(p_params);
    VERIFY_PARAM_NOT_NULL(p_params->evt_handler);

    if (p_params->role >= NUM_ROLES) {
        NRF_LOG_ERROR("Unsuported UWB role");
        return NRF_ERROR_INTERNAL;
    }

    if (p_params->channel != 5 && p_params->channel != 9) {
        NRF_LOG_ERROR("Unsuported UWB channel");
        return NRF_ERROR_INTERNAL;
    }

    evt_handler = p_params->evt_handler;
    sw_cfg.role = p_params->role;
    sw_cfg.sleep_enabled = p_params->enable_sleep;
    sw_cfg.filter_enabled = p_params->enable_filter;
    sw_cfg.tx_rx_leds_enabled = p_params->enable_tx_rx_leds;
    memcpy(&sw_cfg.src_address, p_params->own_address, 8);
    memcpy(&sw_cfg.dest_address, p_params->reply_address, 8);
    channel = p_params->channel;

    // Configure Wake up pin
    if (DW3000_WAKEUP_Pin != DRV_UWB_RANGE_PIN_NOT_USED) {
        nrf_gpio_pin_clear(DW3000_WAKEUP_Pin);
        nrf_gpio_cfg_output(DW3000_WAKEUP_Pin);
    }

    // Initialise GPIOs
    gpio_init();

    // Initialise the SPI
    nrf52833_dk_spi_init();

    // Configuring interrupt
    dw_irq_init();

    // Configure SPI rate, DW3000 supports up to 36 MHz
    port_set_dw_ic_spi_fastrate();

    // Reset DW IC
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    // Probe for the correct device driver.
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    // Need to make sure DW IC is in IDLE_RC before proceeding
    while (!dwt_checkidlerc()) {
    };

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR) {
        NRF_LOG_ERROR("dwt_initialise failed");
        return NRF_ERROR_INTERNAL;
    }

    // Clearing the SPI ready interrupt
    dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);

    // Configure interrupts
    dwt_setinterrupt(DWT_INT_ARFE_BIT_MASK | DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK,
        0, DWT_ENABLE_INT);

    // Define all the callback functions that will be called by the DW IC driver as a result of DW IC events.
    cbs.cbTxDone = cb_tx_done;
    cbs.cbRxOk = cb_rx_done;
    cbs.cbRxTo = cb_rx_to;
    cbs.cbRxErr = cb_rx_err;
    cbs.cbSPIRdy = cb_spi_ready;
    dwt_setcallbacks(&cbs);

    // Install DW IC IRQ handler.
    port_set_dwic_isr(dwt_isr);

    // Configure uwb signal
    uwb_config.chan = channel;
    if (dwt_configure(&uwb_config)) {
        NRF_LOG_ERROR("dwt_configure failed");
        return NRF_ERROR_INTERNAL;
    }
    NRF_LOG_DEBUG("UWB Channel: %s", uwb_config.chan);

    // Read OTP memory
    dwt_otpread(OTP_EUID_ADDR_L, otp_memory, OTP_MEMORY_MAX_ADDR);

    // Configure uwb tx power & pulse shape
    if (channel == 5) {
        tx_config.power = otp_memory[OTP_CH5_PWR_ADDR] != EMPTY_OTP_VAL? otp_memory[OTP_CH5_PWR_ADDR] : DEFAULT_CH5_PWR;
        tx_config.PGdly = DEFAULT_CH5_PGDLY;
        dwt_configuretxrf(&tx_config);
        dwt_set_alternative_pulse_shape(0);
    } else if (channel == 9) {
        tx_config.power = otp_memory[OTP_CH9_PWR_ADDR] != EMPTY_OTP_VAL? otp_memory[OTP_CH9_PWR_ADDR] : DEFAULT_CH9_PWR;
        tx_config.PGdly = DEFAULT_CH9_PGDLY;
        dwt_configuretxrf(&tx_config);
        dwt_set_alternative_pulse_shape(1);
    } else {
        NRF_LOG_ERROR("dwt_configuretxrf failed");
        return NRF_ERROR_INTERNAL;
    }
    NRF_LOG_DEBUG("UWB Tx power: %x", tx_config.power);

    // Configure antenna delays
    if (channel == 5) {
        rx_ant_dly = otp_memory[OTP_CH5_ANT_DLY_ADDR] != EMPTY_OTP_VAL? (otp_memory[OTP_CH5_ANT_DLY_ADDR] >> 16) & 0xFFFF : DEFAULT_RX_ANT_DLY;
        tx_ant_dly = otp_memory[OTP_CH5_ANT_DLY_ADDR] != EMPTY_OTP_VAL? otp_memory[OTP_CH5_ANT_DLY_ADDR] & 0xFFFF : DEFAULT_TX_ANT_DLY;
        dwt_setrxantennadelay(rx_ant_dly);
        dwt_settxantennadelay(tx_ant_dly);
    } else if (channel == 9) {
        rx_ant_dly = otp_memory[OTP_CH9_ANT_DLY_ADDR] != EMPTY_OTP_VAL? (otp_memory[OTP_CH9_ANT_DLY_ADDR] >> 16) & 0xFFFF : DEFAULT_RX_ANT_DLY;
        tx_ant_dly = otp_memory[OTP_CH9_ANT_DLY_ADDR] != EMPTY_OTP_VAL? otp_memory[OTP_CH9_ANT_DLY_ADDR] & 0xFFFF : DEFAULT_TX_ANT_DLY;
        dwt_setrxantennadelay(rx_ant_dly);
        dwt_settxantennadelay(tx_ant_dly);
    } else {
        NRF_LOG_ERROR("dwt_setrxantennadelay/dwt_settxantennadelay failed");
        return NRF_ERROR_INTERNAL;
    }
    NRF_LOG_DEBUG("UWB Rx Ant delay: 0x%08x, Tx Ant delay 0x%08x", rx_ant_dly, tx_ant_dly);

    // Configure filter
    if (sw_cfg.filter_enabled) {
        dwt_setpanid(0xdeca);
        dwt_seteui(sw_cfg.src_address);
        dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
        NRF_LOG_INFO("UWB Filter enabled");
    }

    // Configure LEDs
    if (sw_cfg.tx_rx_leds_enabled) {
        dwt_setleds(0x3);
        NRF_LOG_INFO("UWB TX/RX LEDs enabled");
    }

    // Configure sleep mode
    if (sw_cfg.sleep_enabled) {
        dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL, DWT_SLP_EN | DWT_WAKE_WUP | DWT_PRES_SLEEP | DWT_SLEEP);
        uwb_sleep();
        NRF_LOG_INFO("UWB Sleep enabled");
    }

    // Pre fill initiator message
    init_msg.frameCtrl[0] = 0x41;
    init_msg.frameCtrl[1] = 0xdc;
    init_msg.seqNum = 0;
    init_msg.panID[0] = (0xdeca) & 0xff;
    init_msg.panID[1] = (0xdeca) >> 8;
    memcpy(&init_msg.sourceAddr[0], sw_cfg.src_address, ADDR_BYTE_SIZE_L);
    memcpy(&init_msg.destAddr[0], sw_cfg.dest_address, ADDR_BYTE_SIZE_L);
    init_msg.messageData[FCODE_POS] = SIMPLE_MSG_TAG_POLL;

    // Pre fill responder message
    resp_msg.frameCtrl[0] = 0x41 /*frame type 0x1 == data*/ /*PID comp*/;
    resp_msg.frameCtrl[1] = 0xdc /*dest extended address (64bits)*/ /*src extended address (64bits)*/;
    resp_msg.seqNum = 0;
    resp_msg.panID[0] = (0xdeca) & 0xff;
    resp_msg.panID[1] = (0xdeca) >> 8;
    memcpy(&resp_msg.sourceAddr[0], sw_cfg.src_address, ADDR_BYTE_SIZE_L);
    memcpy(&resp_msg.destAddr[0], sw_cfg.dest_address, ADDR_BYTE_SIZE_L);
    resp_msg.messageData[FCODE_POS] = SIMPLE_MSG_ANCH_RESP;

    return NRF_SUCCESS;
}

uint32_t drv_uwb_range_request(void) {
    srd_msg_dlsl msg;
    uint16_t length;
    uint32_t err;

    if (sw_cfg.role != TWR_INITIATOR) {
        return NRF_ERROR_INVALID_STATE;
    }

    // Increment frame counter
    init_msg.seqNum = frame_sn++;

    // Add last range result
    if (m_last_range != -1) {
        memcpy(&init_msg.messageData[POLL_MSG_TOF_POS], &m_last_range, 8);
        length = SIMPLE_MSG_TAG_POLL_WITH_RG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
    } else {
        length = SIMPLE_MSG_TAG_POLL_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
    }

    // wake up
    if (sw_cfg.sleep_enabled)
        uwb_wake_up();

    // Write the frame data
    dwt_writetxdata(length, (uint8_t *)&init_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(length, 0, 1);                   /* Zero offset in TX buffer, ranging. */

    // Set the delayed rx on time (the response message will be sent after this delay)
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)) {
        NRF_LOG_ERROR("dwt_starttx failed");
    }

    return NRF_SUCCESS;
}

uint32_t drv_uwb_range_scan_start(void) {
    switch (sw_cfg.role) {
    case TWR_RESPONDER: {
        // wake up
        if (sw_cfg.sleep_enabled) {
            uwb_wake_up();
        }

        //immediate rx enable
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    } break;

    case TWR_INITIATOR:
    default:
        return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}

uint32_t drv_uwb_range_scan_stop(void) {
    switch (sw_cfg.role) {
    case TWR_RESPONDER: {
        // Stop rx
        //dwt_rxreset();
        dwt_softreset(1);

        // go to sleep
        if (sw_cfg.sleep_enabled) {
            uwb_sleep();
        }
    } break;

    case TWR_INITIATOR:
    default:
        return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}

uint32_t drv_uwb_range_destination_address_set(uint8_t *p_address) {
    VERIFY_PARAM_NOT_NULL(p_address);

    memcpy(&sw_cfg.dest_address, p_address, 8);

    return NRF_SUCCESS;
}

uint32_t drv_uwb_range_destination_address_get(uint8_t *p_address) {
    VERIFY_PARAM_NOT_NULL(p_address);

    memcpy(p_address, &sw_cfg.dest_address, 8);

    return NRF_SUCCESS;
}

uint32_t drv_uwb_range_source_address_set(uint8_t *p_address) {
    VERIFY_PARAM_NOT_NULL(p_address);

    memcpy(&sw_cfg.src_address, p_address, 8);

    return NRF_SUCCESS;
}

uint32_t drv_uwb_range_source_address_get(uint8_t *p_address) {
    VERIFY_PARAM_NOT_NULL(p_address);

    memcpy(p_address, &sw_cfg.src_address, 8);

    return NRF_SUCCESS;
}