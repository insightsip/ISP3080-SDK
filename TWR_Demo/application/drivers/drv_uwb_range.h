/******************************************************************************
 * @file    drv_uwb_range.h
 * @author  Insight SiP
 * @brief   Range driver header
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

#ifndef __DRV_UWB_RANGE_H__
#define __DRV_UWB_RANGE_H__

#include "deca_device_api.h"
#include "drv_uwb_range.h"
#include "nrf_drv_spi.h"
#include <stdint.h>

#define DRV_UWB_RANGE_PIN_NOT_USED 0xFF

// Simple 2WR function codes
#define SIMPLE_MSG_TAG_POLL (0x51)  // ISP Tag poll message
#define SIMPLE_MSG_ANCH_RESP (0x52) // ISP Anchor response to poll

// lengths including the ranging Message Function Code byte
#define SIMPLE_MSG_TAG_POLL_LEN 1         // FunctionCode(1),
#define SIMPLE_MSG_TAG_POLL_WITH_RG_LEN 9 // FunctionCode(1), Range(8),
#define SIMPLE_MSG_ANCH_RESP_LEN 9        // FunctionCode(1), poll message reception timestamp(4), response message transmission timestamp(4)

#define STANDARD_FRAME_SIZE 127
#define ADDR_BYTE_SIZE_L (8)
#define ADDR_BYTE_SIZE_S (2)
#define FRAME_CONTROL_BYTES 2
#define FRAME_SEQ_NUM_BYTES 1
#define FRAME_PANID 2
#define FRAME_CRC 2
#define FRAME_SOURCE_ADDRESS_S (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP (FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID)                                        //5
#define FRAME_CRTL_AND_ADDRESS_L (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP)                       //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)                       //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)                      //15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL (STANDARD_FRAME_SIZE - FRAME_CRTL_AND_ADDRESS_L - TAG_FINAL_MSG_LEN - FRAME_CRC)  //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS (STANDARD_FRAME_SIZE - FRAME_CRTL_AND_ADDRESS_S - TAG_FINAL_MSG_LEN - FRAME_CRC)  //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS (STANDARD_FRAME_SIZE - FRAME_CRTL_AND_ADDRESS_LS - TAG_FINAL_MSG_LEN - FRAME_CRC) //127 - 15 - 16 - 2 = 94

// Function code byte offset (valid for all message types).
#define FCODE_POS 0        // Function code is 1st byte of messageData
#define POLL_MSG_TOF_POS 1 // ToF is 2nd to 6th byte of messageData

// Simple anchor response byte offsets.
#define POLL_RX_TS 1 // Poll message reception timestamp(2)
#define RESP_TX_TS 5 // Response message transmission timestamp(2)

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor. 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. Should be fine tuned to optimize consumption */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW3000's delayed TX function. (Used by Responder)*/
#define POLL_RX_TO_RESP_TX_DLY_UUS 650
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW3000's wait for response feature. (Used by Initiator) */
#define POLL_TX_TO_RESP_RX_DLY_UUS 340
/* Receive response timeout. */
#define RESP_RX_TIMEOUT_UUS 400

/*Should be accurately calculated during calibration*/
#if defined(BOARD_ISP3080_UX_TG)
#define TX_ANT_DLY 14445
#define RX_ANT_DLY 18385
#elif defined(BOARD_ISP3080_UX_AN)
#define TX_ANT_DLY 14453
#define RX_ANT_DLY 18394
#endif

// OTP memory address list
#define OTP_EUID_ADDR_L 0x00     /**< First 4 Bytes of 64 bit EUID OTP address */
#define OTP_EUID_ADDR_H 0x01     /**< Last 4 Bytes of 64 bit EUID OTP address */
#define OTP_ALT_EUID_ADDR_L 0x02 /**< First 4 Bytes of alternative 64 bit EUID OTP address */
#define OTP_ALT_EUID_ADDR_H 0x03 /**< Last 4 Bytes of alternative 64 bit EUID OTP address */
//Reserved
#define OTP_CH5_ANT_DLY_ADDR 0x10 /**< Channel 5 antenna delay OTP address */
#define OTP_CH9_ANT_DLY_ADDR 0x11 /**< Channel 9 antenna delay OTP address */
#define OTP_CUST12_ADDR 0x12      /**< Customer OTP address */
#define OTP_CUST13_ADDR 0x13      /**< Customer OTP address */
#define OTP_CH5_PWR_ADDR 0x14     /**< Channel 5 TX Power OTP address */
#define OTP_CH9_PWR_ADDR 0x15     /**< Channel 9 TX Powe OTP address */
#define OTP_CUST16_ADDR 0x16      /**< Customer OTP address */
#define OTP_CUST17_ADDR 0x17      /**< Customer OTP address */
#define OTP_CUST18_ADDR 0x18      /**< Customer OTP address */
#define OTP_CUST19_ADDR 0x19      /**< Customer OTP address */
#define OTP_CUST1A_ADDR 0x1A      /**< Customer OTP address */
#define OTP_CUST1B_ADDR 0x1B      /**< Customer OTP address */
#define OTP_CUST1C_ADDR 0x1C      /**< Customer OTP address */
#define OTP_CUST1D_ADDR 0x1D      /**< Customer OTP address */
#define OTP_XTAL_ADDR 0x1E        /**< XTAL trim OTP address */
#define OTP_REV_ADDR 0x1E         /**< Revision OTP address */
#define OTP_MEMORY_MAX_ADDR 0x1F
#define EMPTY_OTP_VAL 0

// simple 802.15.4 frame structure - using long addresses
typedef struct
{
    uint8_t frameCtrl[2];    /**<  frame control bytes 00-01 */
    uint8_t seqNum;          /**<  sequence_number 02 */
    uint8_t panID[2];        /**<  PAN ID 03-04 */
    uint8_t destAddr[8];     /**<  05-12 using 64 bit addresses */
    uint8_t sourceAddr[8];   /**<  13-20 using 64 bit addresses */
    uint8_t messageData[88]; /**<  22-124 (application data and any user payload) */
    uint8_t fcs[2];          /**<  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes. */
} srd_msg_dlsl;

/**@brief uwb roles.
 */
typedef enum {
    TWR_INITIATOR,
    TWR_RESPONDER,
    NUM_ROLES
} drv_uwb_range_role_t;

/**@brief uwb event types.
 */
typedef enum {
    DRV_RANGE_EVT_DATA,    /**< new range */
    DRV_RANGE_EVT_TIMEOUT, /**< Time out */
    DRV_RANGE_EVT_ERROR    /**< Error in frame*/
} drv_uwb_range_evt_type_t;

/**@brief uwb event struct.
 */
typedef struct
{
    drv_uwb_range_evt_type_t type;
    double range;
} drv_uwb_range_evt_t;

/**@brief range driver event handler callback type.
 */
typedef void (*drv_uwb_range_evt_handler_t)(drv_uwb_range_evt_t const *p_evt);

/**@brief Initialization struct for uwb driver.
 */
typedef struct
{
    drv_uwb_range_evt_handler_t evt_handler; /**< Event handler - called after a pin interrupt has been detected. */
    drv_uwb_range_role_t role;
    uint8_t channel;
    uint8_t const *own_address;
    uint8_t const *reply_address;
    bool enable_sleep;
    bool enable_filter;
    bool enable_tx_rx_leds;
} drv_uwb_range_init_t;

/**@brief Function for initializing the uwb driver.
 *
 * @param[in] p_params                  Pointer to init parameters.
 *
 * @retval NRF_SUCCESS                  If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE      If the driver is in invalid state.
 */
uint32_t drv_uwb_range_init(drv_uwb_range_init_t *p_params);

/**@brief Function for requesting a range (TAG only function)
 *
 * @retval NRF_SUCCESS                  If initialization was successful.
 * @retval NRF_ERROR_FORBIDDEN          If the driver is in invalid role.
 */
uint32_t drv_uwb_range_request(void);

/**@brief Function for starting a scan for range request (ANCHOR only function)
 *
 * @retval NRF_SUCCESS                  If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE      If the driver is in invalid role.
 */
uint32_t drv_uwb_range_scan_start(void);

/**@brief Function for stopping a scan for range request (ANCHOR only function)
 *
 * @retval NRF_SUCCESS                  If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE      If the driver is in invalid role.
 */
uint32_t drv_uwb_range_scan_stop(void);

/**@brief Function for setting a new destination address
 *
 * @param[in] p_config                  Pointer to the address
  *
 * @retval NRF_SUCCESS                  If initialization was successful.
 */
uint32_t drv_uwb_range_destination_address_set(uint8_t *p_address);

/**@brief Function for getting a new destination address
 *
 * @param[in] p_config                  Pointer to the address
  *
 * @retval NRF_SUCCESS                  If initialization was successful.
 */
uint32_t drv_uwb_range_destination_address_get(uint8_t *p_address);

/**@brief Function for setting a new source address
 *
 * @param[in] p_config                  Pointer to the address
  *
 * @retval NRF_SUCCESS                  If initialization was successful.
 */
uint32_t drv_uwb_range_source_address_set(uint8_t *p_address);

/**@brief Function for getting a new source address
 *
 * @param[in] p_config                  Pointer to the address
  *
 * @retval NRF_SUCCESS                  If initialization was successful.
 */
uint32_t drv_uwb_range_source_address_get(uint8_t *p_address);
#endif