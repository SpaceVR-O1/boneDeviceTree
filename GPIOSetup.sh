#!/bin/bash
# file: GPIOSetup.sh
# http://elinux.org/Jetson/GPIO
# 

$ sudo su
$ cat /sys/kernel/debug/gpio 

$ echo 57 > /sys/class/gpio/export
$ echo out > /sys/class/gpio/gpio57/direction
$ echo 1 > /sys/class/gpio/gpio57/value
$ cat /sys/kernel/debug/gpi

$ echo 57 > /sys/class/gpio/unexport       
$ cat /sys/kernel/debug/gpi

$  exit

# AFTER cat /sys/kernel/debug/gpio RUNS
# GPIOs 0-255, platform/6000d000.gpio, tegra-gpio:
# gpio-58  (vdd-lcd-bl-en       ) out lo
# gpio-59  (panel rst           ) out lo
# gpio-63  (avdd-hdmi-pll       ) out lo
# gpio-70  (temp_alert          ) in  hi
# gpio-82  (raydium-irq         ) in  lo
# gpio-84  (raydium-reset       ) out lo
# gpio-86  (vdd-hdmi            ) out hi
# gpio-108 (usb0-vbus           ) in  lo
# gpio-109 (usb1-usb2-vbus      ) in  hi
# gpio-111 (hdmi_hpd            ) in  hi
# gpio-122 (vdd-lcd-bl          ) out lo
# gpio-128 (Power               ) in  hi
# gpio-132 (sdhci_wp            ) in  hi
# gpio-136 (sdmmc-en-supply     ) out lo
# gpio-138 (reg-dcdc-1v2        ) out hi
# gpio-143 (headphone detect    ) in  hi
# gpio-170 (sdhci_cd            ) in  hi
# GPIOs 1016-1023, platform/as3722-pinctrl, as3722-gpio, can sleep:
# gpio-1018 (as3722-gpio2-supply ) out hi
# gpio-1020 (as3722-gpio4-supply ) out lo
#  

# AFTER echo 57 > /sys/class/gpio/export RUNS 
# gpio-57  (sysfs               ) out hi    #gpio-57 NOW SHOWS UP IN TABLE
