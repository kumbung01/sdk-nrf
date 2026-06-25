nRF Connect SDK: sdk-nrf
########################

.. contents::
   :local:
   :depth: 2

This repository contains the core of nRF Connect SDK, including subsystems,
libraries, samples, and applications.
It is also the SDK's west manifest repository, containing the nRF Connect SDK
manifest (west.yml).

Documentation
*************

Official latest documentation at https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html

For earlier versions, open the latest version and use the drop-down under the title header.


ESB-TDMA-FHSS
#############

This stack is built on top of the ESB source, implementing TDMA and FHSS.
It provides two key features:

1. **TDMA** - Manages data from peripherals on a per-slot basis to prevent collisions.
2. **FHSS** - Changes the channel per slot to prevent channel saturation.

   - TX power is adjusted per channel based on TX RSSI.

Channels 1 through 79 are supported.

Slot duration depends on the polling rate — 500µs at 2kHz, 8000µs at 125Hz.

The effective latency per peripheral scales with the number of connected peripherals.
Example: at 2kHz with 2 peripherals, each peripheral has a typical latency of 500µs × 2 = 1ms
(under good radio conditions).

**currently only tested on nrf52840**

Configuration
*************

Core
====

.. code-block:: cfg

   # Enable ESB
   CONFIG_ESB=y

ESB Options
===========

.. list-table::
   :header-rows: 1
   :widths: 40 15 45

   * - Config
     - Default
     - Description
   * - ``CONFIG_ESB_CENTRAL``
     - ``n``
     - Enable ESB central mode.
   * - ``CONFIG_ESB_MAX_PAYLOAD_LENGTH``
     - ``32``
     - Maximum payload length (bytes).
   * - ``CONFIG_ESB_TX_FIFO_SIZE``
     - ``8``
     - TX FIFO depth.
   * - ``CONFIG_ESB_RX_FIFO_SIZE``
     - ``8``
     - RX FIFO depth.
   * - ``CONFIG_ESB_TX_RINGBUF``
     - ``y``
     - Use ring buffer instead of FIFO for TX.
   * - ``CONFIG_ESB_TX_RINGBUF_SIZE``
     - ``256``
     - TX ring buffer size (bytes).
   * - ``CONFIG_ESB_HFCLK_OFF_EVERY_TX``
     - ``n``
     - turn off hfclk after every tx. reduces power consumption.
   * - ``CONFIG_ESB_FAST_SWITCHING_PERIPHERAL``
     - ``n``
     - fast switching between rx/tx. experimental.

Available Polling Rates
=======================

Only one polling rate can be enabled at a time.

.. list-table::
   :header-rows: 1
   :widths: 40 15 45

   * - Config
     - Default
     - Description
   * - ``CONFIG_ESB_POLLING_RATE_2KHZ``
     - ``n``
     - Set central polling rate to 2kHz.
   * - ``CONFIG_ESB_POLLING_RATE_1KHZ``
     - ``y``
     - Set central polling rate to 1kHz (default).
   * - ``CONFIG_ESB_POLLING_RATE_500HZ``
     - ``n``
     - Set central polling rate to 500Hz.
   * - ``CONFIG_ESB_POLLING_RATE_250HZ``
     - ``n``
     - Set central polling rate to 250Hz.
   * - ``CONFIG_ESB_POLLING_RATE_125HZ``
     - ``n``
     - Set central polling rate to 125Hz.
