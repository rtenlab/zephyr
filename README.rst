.. raw:: html

   <a href="https://www.zephyrproject.org">
     <p align="center">
       <img src="doc/_static/images/logo-readme.png">
     </p>
   </a>

   <a href="https://bestpractices.coreinfrastructure.org/projects/74"><img
   src="https://bestpractices.coreinfrastructure.org/projects/74/badge"></a>
   <a href="https://buildkite.com/zephyr/zephyr">
   <img
   src="https://badge.buildkite.com/f5bd0dc88306cee17c9b38e78d11bb74a6291e3f40e7d13f31.svg?branch=main"></a>


The Zephyr Project is a scalable real-time operating system (RTOS) supporting
multiple hardware architectures, optimized for resource constrained devices,
and built with security in mind.

The Zephyr OS is based on a small-footprint kernel designed for use on
resource-constrained systems: from simple embedded environmental sensors and
LED wearables to sophisticated smart watches and IoT wireless gateways.

The Zephyr kernel supports multiple architectures, including ARM Cortex-M,
Intel x86, ARC, Nios II, Tensilica Xtensa, and RISC-V, and a large number of
`supported boards`_.

.. below included in doc/introduction/introduction.rst


Notes on Adafruit Feather nRF52840 Sense
****************************************

The information below introduces how to use the Adafruit's default bootloader to 
download a Zephyr image to the board. This is convenient as it does not require 
an external J-Link programmer and the Adafruit Feather nRF52840 Sense board does not 
have a built-in socket to connect the J-Link programmer. 

You need to follow the Adafruit's instruction to setup the Arduino environment first:
https://learn.adafruit.com/adafruit-feather-sense
`adafruit-nrfutil` will be installed as part of this procedure. 

Then follow the Getting Started Guide of Zephyr (https://docs.zephyrproject.org/latest/) and replace ``~/zephyrproject/zephyr`` with this repository.

Compile and run Blinky
----------------------

* Compile:

    west build -b adafruit_feather_nrf52840_sense samples/basic/blinky
   
* Download:
  
  Enter the DFU mode by double-clicking the *Reset button* of the Sense board. When ready, a large Green LED will turn on and the Sense device will be detected as a USB Mass storage (FTHRSNSBOOT) on your host machine. Then,
  
    adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x00B6 --application build/zephyr/zephyr.hex build/zephyr/zephyr.zip
      
    adafruit-nrfutil --verbose dfu serial -pkg build/zephyr/zephyr.zip -p /dev/ttyACM0 -b 115200 --singlebank 

  ``/dev/ttyACM0`` is the serial port for the Sense board. 
  
  If this fails, try updating the bootloader: https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/updating-the-bootloader


* Change kernel configuration (similar to Linux kernel menuconfig; don't use if not needed): 

    west build -t menuconfig
   

Other programs 
----------------------

You may need to delete the ``build`` folder before compiling another application.

* Button

  Turns on the RED led while the User SW button is being pressed. No printk output is visible from this program.

    west build -b adafruit_feather_nrf52840_sense samples/basic/blinky
    
    adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x00B6 --application build/zephyr/zephyr.hex build/zephyr/zephyr.zip
      
    adafruit-nrfutil --verbose dfu serial -pkg build/zephyr/zephyr.zip -p /dev/ttyACM0 -b 115200 --singlebank 


* USB Console

    west build -b adafruit_feather_nrf52840_sense samples/subsys/usb/console/
    
    adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x00B6 --application build/zephyr/zephyr.hex build/zephyr/zephyr.zip
      
    adafruit-nrfutil --verbose dfu serial -pkg build/zephyr/zephyr.zip -p /dev/ttyACM0 -b 115200 --singlebank 

  This program enables USB-Serial, so you can check printk output on your host computer. Use the Arduino IDE's serial monitor or `sudo minicom` in a terminal.
  
  Refer to the source code (``samples/subsys/usb/console/src/main.c``) and the project config (``samples/subsys/usb/console/prj.conf``) of this sample to enable printk in your program.


* BLE Beacon

    west build -b adafruit_feather_nrf52840_sense samples/bluetooth/beacon/
    
    adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x00B6 --application build/zephyr/zephyr.hex build/zephyr/zephyr.zip
      
    adafruit-nrfutil --verbose dfu serial -pkg build/zephyr/zephyr.zip -p /dev/ttyACM0 -b 115200 --singlebank 

  See the Adafruit's beacon for testing.


* BLE Beacon + LED + UART

    west build -b adafruit_feather_nrf52840_sense samples/bluetooth/beacon_led_uart/
    
    adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x00B6 --application build/zephyr/zephyr.hex build/zephyr/zephyr.zip
      
    adafruit-nrfutil --verbose dfu serial -pkg build/zephyr/zephyr.zip -p /dev/ttyACM0 -b 115200 --singlebank 
    
  A naive integration of the three sample programs. The program blinks and prints every second when the serial monitor is on; otherwise, the device goes to sleep mode and LED blinks very slowly.  


Getting Started
***************

Welcome to Zephyr! See the `Introduction to Zephyr`_ for a high-level overview,
and the documentation's `Getting Started Guide`_ to start developing.

.. start_include_here

Community Support
*****************

Community support is provided via mailing lists and Slack; see the Resources
below for details.

.. _project-resources:

Resources
*********

Here's a quick summary of resources to help you find your way around:

* **Help**: `Asking for Help Tips`_
* **Documentation**: http://docs.zephyrproject.org (`Getting Started Guide`_)
* **Source Code**: https://github.com/zephyrproject-rtos/zephyr is the main
  repository; https://elixir.bootlin.com/zephyr/latest/source contains a
  searchable index
* **Releases**: https://github.com/zephyrproject-rtos/zephyr/releases
* **Samples and example code**: see `Sample and Demo Code Examples`_
* **Mailing Lists**: users@lists.zephyrproject.org and
  devel@lists.zephyrproject.org are the main user and developer mailing lists,
  respectively. You can join the developer's list and search its archives at
  `Zephyr Development mailing list`_. The other `Zephyr mailing list
  subgroups`_ have their own archives and sign-up pages.
* **Nightly CI Build Status**: https://lists.zephyrproject.org/g/builds
  The builds@lists.zephyrproject.org mailing list archives the CI
  (buildkite) nightly build results.
* **Chat**: Zephyr's Slack workspace is https://zephyrproject.slack.com.  Use
  this `Slack Invite`_ to register.
* **Contributing**: see the `Contribution Guide`_
* **Wiki**: `Zephyr GitHub wiki`_
* **Issues**: https://github.com/zephyrproject-rtos/zephyr/issues
* **Security Issues**: Email vulnerabilities@zephyrproject.org to report
  security issues; also see our `Security`_ documentation. Security issues are
  tracked separately at https://zephyrprojectsec.atlassian.net.
* **Zephyr Project Website**: https://zephyrproject.org

.. _Slack Invite: https://tinyurl.com/2vue8666
.. _supported boards: http://docs.zephyrproject.org/latest/boards/index.html
.. _Zephyr Documentation: http://docs.zephyrproject.org
.. _Introduction to Zephyr: http://docs.zephyrproject.org/latest/introduction/index.html
.. _Getting Started Guide: http://docs.zephyrproject.org/latest/getting_started/index.html
.. _Contribution Guide: http://docs.zephyrproject.org/latest/contribute/index.html
.. _Zephyr GitHub wiki: https://github.com/zephyrproject-rtos/zephyr/wiki
.. _Zephyr Development mailing list: https://lists.zephyrproject.org/g/devel
.. _Zephyr mailing list subgroups: https://lists.zephyrproject.org/g/main/subgroups
.. _Sample and Demo Code Examples: http://docs.zephyrproject.org/latest/samples/index.html
.. _Security: http://docs.zephyrproject.org/latest/security/index.html
.. _Asking for Help Tips: https://docs.zephyrproject.org/latest/getting_started/index.html#asking-for-help
