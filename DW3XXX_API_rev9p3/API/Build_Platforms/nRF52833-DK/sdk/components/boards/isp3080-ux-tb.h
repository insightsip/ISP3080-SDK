 /******************************************************************************
 * @file    isp3080-ux-tb.h
 * @author  Insight SiP
 * @brief   ISP3080-UX-TB board specific file.
 *
 * @attention
 *  THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef ISP3010_UX_TB_H
#define ISP3010_UX_TB_H

#include "isp3080-ux.h"

// Boards specific pinout
#define PIN_CURR_MODE 255
#define PIN_BATT_MEAS 255
#define PIN_GREEN_LED 255
#define PIN_RED_LED 255
#define PIN_UART_RX NRF_GPIO_PIN_MAP(0, 8)
#define PIN_UART_TX NRF_GPIO_PIN_MAP(0, 6)

// Battery measurement
// Vmeas = R2/(R2+R1)*Vbatt
#define BATT_VOLTAGE_DIVIDER_R1 0
#define BATT_VOLTAGE_DIVIDER_R2 0

#endif




