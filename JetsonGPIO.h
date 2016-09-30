/* JetsonGPIO.h 
 * A program that creates custom sized General Purpose Input / Output 
 * (GPIO) pin group on TK1 connectors J3A2 and J3A1 and controls their 
 * state (Input or Output and High, Low, or Pulse Width Modulation). 
 * 
 * Author: RidgeRun 2011
 * Editted: Blaze Sanders SpaceVR(TM) June 2016
 * Link: http://elinux.org/Jetson/GPIO
 * 
 * Note: The GPIO pins on Jetson TK1 are all 1.8V logic and can only 
 * supply a few milli-amps of current, so you can't simply attach common 
 * 5V or 3.3V logic signals or devices. One-way opto-isolated level shifter
 * are the preffered method for connecting the TK1 to external devices, 
 * since its a more rugged method (www.sparkfun.com/products/9118).
 */
 
#ifndef JETSON_GPIO_H
#define JETSON_GPIO_H

#define DEBUG 1   //Toggle error messages on and off

//Pin value/state constants
#define LOW   0   
#define HIGH  1
//TO-DO: #define PWM_TWO_PERCENT to PWM_HUNDRED_PERCENT in two percent steps

//Pin direction constants
#define INPUT_PIN 0
#define OUTPUT_PIN 1
//TO-DO: define PWM


//NVIDIA specific GPIO pin constants
#define MAX_GPIO_PINS 9  //GPIO_PU0 to GPIO_PU6 + GPIO_PH1 + GPIO_PH2 
#define GPIO_PU0 0x0160  //Expansion Connector J3A2 pin 40 Available for general use (INPUT OR OUTPUT)
#define GPIO_PU1 0x160   //Expansion Connector J3A2 pin 43 Available for general use (INPUT OR OUTPUT)
#define GPIO_PU2 0x162   //Expansion Connector J3A2 pin 46 Available for general use (INPUT OR OUTPUT)  
#define GPIO_PU3 0x163   //Expansion Connector J3A2 pin 49 Available for general use (INPUT OR OUTPUT)
#define GPIO_PU4 0x164   //Expansion Connector J3A2 pin 52 Available for general use (INPUT OR OUTPUT)
#define GPIO_PU5 0x165   //Expansion Connector J3A2 pin 55 Available for general use (INPUT OR OUTPUT)
#define GPIO_PU6 0x166   //Expansion Connector J3A2 pin 58 Available for general use (INPUT OR OUTPUT)
#define GPIO_PH1 0x57    //Expansion Connector J3A1 pin 50 Backlight PWM    (INPUT) LCD_BL_PWM 
//TO-DO DETERMINE PORT NUMBER #define GPIO_PH2 ??   //Expansion Connector J3A1 pin 48 Backlight enable (INPUT) LCD_BL_EN
//string  GPIO_PU0  = "gpio160";       //Defaults to 1.8V
//const char * GPIO_PU0  = "gpio160";

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT 3000 // 3000 milliseconds = 3 seconds 
#define MAX_BUF 64
	
/** Low level driver to control up to 9 K1 GPIO pins.
 * @link http://elinux.org/Jetson/GPIO
 * @link https://github.com/derekmolloy/boneDeviceTree/tree/master/gpio
 */ 
class JetsonGPIO
{ 
  public: 
    JetsonGPIO();
    ~JetsonGPIO();
    JetsonGPIO(unsigned int name, unsigned int direction, unsigned int initValue);

    //Wrapper functions for private functions
    unsigned int ReadPinState();
    void WritePinState(unsigned int value); 
    void UnitTest();
    
  private:
    unsigned int name;           //NVIDIA pin name (i.e. GPIO_PU0)
    unsigned int inputPinValue;  //Stored state of input pin
    
    //Linux specific system calls to get GPIO information
    int gpio_export(unsigned int gpio);
    int gpio_unexport(unsigned int gpio);
    int gpio_set_dir(unsigned int gpio, unsigned int out_flag);
    int gpio_set_value(unsigned int gpio, unsigned int value);
    int gpio_get_value(unsigned int gpio, unsigned int *value);
    int gpio_set_edge(unsigned int gpio, char *edge);
    int gpio_fd_open(unsigned int gpio);
    int gpio_fd_close(int fd);
};

#endif //JETSON_GPIO_H

/* Copyright Derek Molloy, School of Electronic Engineering, Dublin City University
 * www.derekmolloy.ie
 *
 * Based on Software by RidgeRun
 * Copyright (c) 2011, RidgeRun
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 * must display the following acknowledgement:
 * This product includes software developed by the RidgeRun.
 * 4. Neither the name of the RidgeRun nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY RIDGERUN ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL RIDGERUN BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
