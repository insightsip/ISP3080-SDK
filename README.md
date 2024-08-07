# ISP3080-SDK
UWB examples using ISP3080-UX

## Overview

The aim of this archive is to show examples of UWB applications using the ISP3080-UX module.

The ISP3080-UX module is based on Qorvo / DecaWave QM33110 single-chip UWB transceiver and nRF52833 Nordic Semiconductor 2.4GHz wireless System on Chip (SoC). It integrates a 32-bit ARM Cortex™ M4 CPU, 512 kB flash memory, 128 kB RAM as well as analog and digital peripherals. Despite the small size of 12 x 12 x 1.5 mm, the module integrates decoupling capacitors, 38.4 MHz crystal for UWB, 32 MHz and 32.768kHz crystals for BLE, DC-DC converters, RF matching circuits and two antennas in addition to the wireless SoCs. Low power consumption and advanced power management enables battery lifetimes up to several months on a coin cell battery.

For more information about ISP3080-UX go to https://www.insightsip.com/products/combo-smart-modules/isp3080.

## Environment

The examples are ready to use with the Segger Embedded Studio (SES) IDE. Using SES version 5.42a is recommended.

SES provide a free license for nRF52833 development. Therefore it can be used freely for development on ISP3080-UX.
Licenses can be requested at https://license.segger.com/Nordic.cgi

For more information regarding Segger Embedded Studio, please visit https://www.segger.com/products/development-tools/embedded-studio/

## Getting started

- Install SES version 5.42a.
- Download the SDK package using the command:

        git clone --recursive https://github.com/insightsip/ISP3080-SDK.git

- With SES open the file:
  - *dw3000_api.emProject* located in  ISP3080-SDK\DW3XXX_API_rev9p3\API\Build_Platforms\nRF52833-DK to access the list of basic UWB examples.
  - *TWR_Demo.emProject* located in ISP3080-UWB\Software\ISP3080-SDK\TWR_Demo\application\ses to access insight SiP demo code.
- Build and run the project.
