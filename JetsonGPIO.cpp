/* JetsonGPIO.cpp 
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

#include <iostream>   //Standard input/output stream objects, needed to use cout()
#include <chrono>     //High accuracy microsecond timing

#include <string.h>   //Functions to manipulate C strings and arrays, needed to use write()
#include "JetsonGPIO.h"

//Header files to use system close(), open() and ioctl() functions.
#include <unistd.h>  
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace std;

/**Default Constructor with error message.
 */
JetsonGPIO::JetsonGPIO(){
  cout << "I'm sorry Dave, I'm afraid I can't do that. K1 General Purpose Input / Output (GPIO) pin was NOT created!" << endl;
}

/**Default Destructor to close all General Purpose Input / Output (GPIO) pins. 
 */
JetsonGPIO::~JetsonGPIO(){
  if (DEBUG) cout << "All General Purpose Input / Output (GPIO) pin were deleted." << endl;
  
  gpio_unexport(GPIO_PU0);
  gpio_unexport(GPIO_PU1);
  gpio_unexport(GPIO_PU2);
  gpio_unexport(GPIO_PU3);
  gpio_unexport(GPIO_PU4);
  gpio_unexport(GPIO_PU5);
  gpio_unexport(GPIO_PU6);
  gpio_unexport(GPIO_PH1);
  //TO-DO: DETERMINE PORT NUMBER gpio_unexport(GPIO_PH2);
}

/**Standard Constructor to create a single GPIO pin 
 * @param name Name of the GPIO pin to create and open on the NVIVDA K1 (i.e GPIO_PU0 or GPIO_PH1)
 * @param direction Direction of IO pin (i.e. INPUT, OUTPUT, or TO-DO:PWM) 
 * @param initValue Initial value of an output pin. 
 * @see   Note: Input pins default to LOW before first attempted read in Constructor
 */ 
JetsonGPIO::JetsonGPIO(unsigned int name, unsigned int direction, unsigned int initValue){
  
  //Check that NVIDIA K1 specific GPIO name constant is being used
  if(gpio_export(name) && DEBUG){
    cout << "Failed to create GPIO PIN #" << name << " on NVIVDA K1." << endl;
    cout << "Please use one of the following six GPIO pin name constants:" << endl;
    cout << "GPIO_PU0 (connected to gpio160 on Expansion Connector J3A2 pin 40)" << endl;     
    cout << "GPIO_PU1 (connected to gpio161 on Expansion Connector J3A2 pin 43)" << endl;
    cout << "GPIO_PU2 (connected to gpio162 on Expansion Connector J3A2 pin 46)" << endl;
    cout << "GPIO_PU3 (connected to gpio163 on Expansion Connector J3A2 pin 49)" << endl;
    cout << "GPIO_PU4 (connected to gpio164 on Expansion Connector J3A2 pin 52)" << endl;
    cout << "GPIO_PU5 (connected to gpio165 on Expansion Connector J3A2 pin 55)" << endl;
    cout << "GPIO_PH6 (connected to gpio166 on Expansion Connector J3A2 pin 58)" << endl;
    cout << "GPIO_PH1 (connected to gpio57  on Expansion Connector J3A1 pin 50)" << endl; 
    //TO-DO: cout << "GPIO_PH2 (connected to gpio??  on Expansion Connector J3A1 pin 48)" << endl;  
  }
  else{
    //Check that valid direction constant (INPUT_PIN or OUTPUT_PIN) is being used
    if(gpio_set_dir(name, direction) && DEBUG){
	  cout << "Failed to change direction type of GPIO pin #" << name << " on NVIVDA K1." << endl;
      cout << "Please use one of the following three GPIO direction type constants:" << endl;
      cout << "INPUT" << endl;    
      cout << "OUTPUT" << endl; 
      //TO-DO: cout << "PWM" << endl; 	
	}
	else{
	  //Determine if creating an input pin or ouput pin
	  switch(direction){
	    case OUTPUT_PIN:
	      //Check that valid value/state constant (INPUT_PIN or OUTPUT_PIN) is being used
	      if(gpio_set_value(name, initValue) && DEBUG){
	        cout << "Failed to set state of GPIO PIN #" << name << " on NVIVDA K1." << endl;
            cout << "Please use one of the following three GPIO direction type constants:" << endl;
            cout << "LOW" << endl;    
            cout << "HIGH" << endl; 
            //TO-DO: cout << "???_PERCENT" << endl;  	  
	      } 
	      if(DEBUG) cout << "The " << name << " GPIO pin was created, set as OUTPUT and has a value of " << initValue << "." << endl; 
	      break;
	    case INPUT_PIN:
	      //Get input state on GPIO pin   
	      inputPinValue = 0; //Default input pin state to LOW
	      if(gpio_get_value(name, &inputPinValue) && DEBUG){
	        cout << "Failed to get state of GPIO PIN #" << name << " on NVIVDA K1." << endl;	  
	      }
	      if(DEBUG) cout << "The " << name << " GPIO pin was created, set as INPUT and has a value of " << inputPinValue << "." << endl;  
	      break;
	    /* TO-DO: case PWN_PIN:
	     * Is PWM cose enough to Output Pin Type case PWM:
	     * //TO-DO: ???
	     * if(DEBUG) cout << "The " << name << " GPIO pin was created, set as OUTPUT PWM and has a duty cycle of " << initValue << "." << endl; 
	     * break;
	     */  
	    default:
	      if(DEBUG) cout << "Invalid direction type for GPIO pin #" << name << endl; 
      }//END SWITCH
	       	
	}//END 2nd ELSE
  }//END 1st ELSE 
}//END JetsonGPIO() function 

/**Read the logic level on the choosen GPIO pin 
 * @param name Name of the GPIO pin to read logic level from (i.e GPIO_PU0 or GPIO_PH1)
 * @return State of GPIO input pin 0 (LOW) or 1 (HIGH)
 */ 
unsigned int JetsonGPIO::ReadPinState(){
  
  if(gpio_get_value(name, &inputPinValue) && DEBUG){
    cout << "Failed to get state of GPIO PIN #" << name << " on NVIVDA K1." << endl;	  
  }
  return inputPinValue;
}

/**Set the value/state of an output pin to a new level
 * @param value Value to output on the GPIO pin
 */  
void JetsonGPIO::WritePinState(unsigned int value){
  
  if(gpio_set_value(name, value) && DEBUG){
    cout << "Failed to set state of GPIO PIN #" << name << " on NVIVDA K1." << endl;
    cout << "Please use one of the following three GPIO direction type constants:" << endl;
    cout << "LOW" << endl;    
    cout << "HIGH" << endl; 
    //TO-DO: cout << "???_PERCENT" << endl;  	   
  } 
}

/** Unit Test: Create and test an array of three GPIO pins (two outputs and one input) 
 *  @see Test #1 - Time stamp a rising edge trigger and PWM exposure event to a resolution of 1 microsecond
 *  @see Test #2 - Get state of inoput pint and print value
 */ 
void JetsonGPIO::UnitTest(){

  //Make sure to run the compiled executable under admin / sudo privileges 
  
  unsigned int NUMBER_OF_GPIO_PINS = 3;
  JetsonGPIO K1_GPIO_Pin[NUMBER_OF_GPIO_PINS];
  K1_GPIO_Pin[0] = JetsonGPIO(GPIO_PU0, OUTPUT_PIN, LOW);
  K1_GPIO_Pin[1] = JetsonGPIO(GPIO_PU1, OUTPUT_PIN, LOW);
  K1_GPIO_Pin[2] = JetsonGPIO(GPIO_PU2, INPUT_PIN, LOW);
  
  time_t now;
  struct tm MissionStart;
  double elapsedSeconds;
  
  auto start = chrono::high_resolution_clock::now();
  time(&now);  /* get current time; same as: now = time(NULL)  */
  
  //Setup time variables to track Mission Elapsed Time (MET)
  MissionStart = *localtime(&now);
  MissionStart.tm_hour = 0; MissionStart.tm_min = 0; MissionStart.tm_sec = 0;
  MissionStart.tm_mon = 0;  MissionStart.tm_mday = 1;

  elapsedSeconds = difftime(now,mktime(&MissionStart));
  auto elapsed = chrono::high_resolution_clock::now() - start;
  long long elapsedMircoSeconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  
  cout << "TEST #1:" << endl;	
  printf("Mission Elaspe Time (MET) TRIGGER timeStamp (i.e Month-MonthDay-Hour-Minutes-Seconds-MilliSeconds) = %d-%d-%d-%d-%f-", 
          MissionStart.tm_mon, MissionStart.tm_mday, MissionStart.tm_hour, MissionStart.tm_min, elapsedSeconds);
  cout << elapsedMircoSeconds << endl;
  
  K1_GPIO_Pin[0].WritePinState(HIGH);  //Set trigger pin high
  
  printf("Mission Elaspe Time (MET) EXPOSURE timeStamp (i.e Month-MonthDay-Hour-Minutes-Seconds-MilliSeconds) = %d-%d-%d-%d-%f-", 
          MissionStart.tm_mon, MissionStart.tm_mday, MissionStart.tm_hour, MissionStart.tm_min, elapsedSeconds);
  cout << elapsedMircoSeconds << endl;
  
  //Toggle exposure pin at 50% duty cycle with period of 32 millisecons
  K1_GPIO_Pin[1].WritePinState(HIGH);
  usleep(16000);
  K1_GPIO_Pin[1].WritePinState(LOW);
  usleep(16000);
  K1_GPIO_Pin[1].WritePinState(HIGH);
  
  cout << "TEST #2:" << endl;
  unsigned int testInputPinState = K1_GPIO_Pin[2].ReadPinState();
  cout << "testInputPinState = " << testInputPinState << endl;
  
  cout << "Test complete!" << endl;
  
}


/**Add GPIO pin to /sys/kernel/debug/gpio table
 * @param gpio Name of the GPIO pin to open on the NVIVDA K1 (i.e GPIO_PU0 or GPIO_PH1)
 * @link https://github.com/derekmolloy/boneDeviceTree/tree/master/gpio
 * @see GPIOSetup.sh file for command line example 
 * @return FALSE = 0 if no errors, TRUE = 1 otherwise
 */ 
int JetsonGPIO::gpio_export(unsigned int gpio){
  int fd, len;
  char buf[MAX_BUF];
  
  fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
  
  if (fd < 0) {
    perror("gpio/export");
    return fd;
  }
 
  len = snprintf(buf, sizeof(buf), "%d", gpio);
  write(fd, buf, len);
  close(fd);
  
  return 0;
}

/**Remove GPIO pin to /sys/kernel/debug/gpio table
 * @param gpio Name of the GPIO pin to open on the NVIVDA K1 (i.e GPIO_PU0 or GPIO_PH1)
 * @link https://github.com/derekmolloy/boneDeviceTree/tree/master/gpio
 * @see GPIOSetup.sh file for command line example 
 * @return FALSE = 0 if no errors, TRUE = 1 otherwise
 */ 
int JetsonGPIO::gpio_unexport(unsigned int gpio){
  int fd, len;
  char buf[MAX_BUF];
  fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
  
  if (fd < 0) {
	perror("gpio/export");
	return fd;
  }

  len = snprintf(buf, sizeof(buf), "%d", gpio);
  write(fd, buf, len);
  close(fd);

  return 0;
}

/**Set direction of GPIO output pin (i.e INPUT_PIN or OUTPUT_PIN)
 * @param gpio Name of the GPIO output pin to change on the NVIVDA K1 (i.e GPIO_PU0 or GPIO_PH1)
 * @param out_flag Desired direction of pin (INPUT_PIN  = 0 or OUTPUT_PIN = 1)
 * @link https://github.com/derekmolloy/boneDeviceTree/tree/master/gpio
 * @see GPIOSetup.sh file for command line example 
 * @see JetsonGPIO.h for pin constants
 * @return FALSE = 0 if no errors, TRUE = 1 otherwise
 */ 
int JetsonGPIO::gpio_set_dir(unsigned int gpio, unsigned int out_flag){
  int fd;
  char buf[MAX_BUF];
  
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);
  fd = open(buf, O_WRONLY);

  if (fd < 0) {
	perror("gpio/direction");
	return fd;
  }

  if (out_flag == OUTPUT_PIN)
	write(fd, "out", 4);
  else
	write(fd, "in", 3);
  
  close(fd);

  return 0;
}

/**Set value of GPIO output pin (i.e LOW, HIGH, or TO-DO: PWM)
 * @param gpio Name of the GPIO output pin to change on the NVIVDA K1 (i.e GPIO_PU0 or GPIO_PH1)
 * @param value Desired value/state of pin (LOW  = 0, HIGH = 1, or TO-DO: TWO_PERCENT = 2)
 * @link https://github.com/derekmolloy/boneDeviceTree/tree/master/gpio
 * @see GPIOSetup.sh file for command line example 
 * @see JetsonGPIO.h for pin constants
 * @return FALSE = 0 if no errors, TRUE = 1 otherwise
 */
int JetsonGPIO::gpio_set_value(unsigned int gpio, unsigned int value){
  int fd;
  char buf[MAX_BUF];
  
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
  fd = open(buf, O_WRONLY);

  if (fd < 0) {
	perror("gpio/set-value");
	return fd;
  }

  if (value==LOW)
    write(fd, "0", 2);
  else
    write(fd, "1", 2);

  close(fd);

  return 0;
}

/**Get value of GPIO input pin (i.e LOW or HIGH)
 * @param gpio Name of the GPIO output pin to change on the NVIVDA K1 (i.e GPIO_PU0 or GPIO_PH1)
 * @param value Current value/state of pin (LOW  = 0 or HIGH = 1)
 * @link https://github.com/derekmolloy/boneDeviceTree/tree/master/gpio
 * @see GPIOSetup.sh file for command line example 
 * @see JetsonGPIO.h for pin constants
 * @return FALSE = 0 if no errors, TRUE = 1 otherwise
 */
int JetsonGPIO::gpio_get_value(unsigned int gpio, unsigned int *value){
  int fd;
  char buf[MAX_BUF];
  char ch;

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
  fd = open(buf, O_RDONLY);

  if (fd < 0) {
	perror("gpio/get-value");
    return fd;
  }

  read(fd, &ch, 1);

  if (ch != '0') {
	*value = 1;
  } else {
	*value = 0;
  }

  close(fd);

  return 0;
}

/**???
 * @param gpio Name of the GPIO output pin to change on the NVIVDA K1 (i.e GPIO_PU0 or GPIO_PH1)
 * @param edge ???
 * @link https://github.com/derekmolloy/boneDeviceTree/tree/master/gpio
 * @see GPIOSetup.sh file for command line example ???
 * @see JetsonGPIO.h for pin constants
 * @return FALSE = 0 if no errors, TRUE = 1 otherwise
 */
int JetsonGPIO::gpio_set_edge(unsigned int gpio, char *edge){
  int fd;
  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);
  fd = open(buf, O_WRONLY);

  if (fd < 0) {
    perror("gpio/set-edge");
    return fd;
  }
  
  write(fd, edge, strlen(edge) + 1);

  close(fd);

  return 0;
}

/**???
 * @param gpio Name of the GPIO output pin to change on the NVIVDA K1 (i.e GPIO_PU0 or GPIO_PH1)
 * @link https://github.com/derekmolloy/boneDeviceTree/tree/master/gpio
 * @see GPIOSetup.sh file for command line example ???
 * @see JetsonGPIO.h for pin constants
 * @return FALSE = 0 if no errors, TRUE = 1 otherwise
 */
int JetsonGPIO::gpio_fd_open(unsigned int gpio){
  int fd;
  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
  fd = open(buf, O_RDONLY | O_NONBLOCK );

  if (fd < 0) {
    perror("gpio/fd_open"); 
  }

  return fd;
}

/**???
 * @param gpio Name of the GPIO output pin to change on the NVIVDA K1 (i.e GPIO_PU0 or GPIO_PH1)
 * @link https://github.com/derekmolloy/boneDeviceTree/tree/master/gpio
 * @see GPIOSetup.sh file for command line example ???
 * @see JetsonGPIO.h for pin constants
 * @return FALSE = 0 if no errors, TRUE = 1 otherwise
 */
int JetsonGPIO::gpio_fd_close(int fd){
  return close(fd);
}

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
