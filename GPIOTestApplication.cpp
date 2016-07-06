/* GPIOTestApplication.cpp 
 * An example main driver program to excerise JetsonGPIO.cpp
 * 
 * Author: Blaze Sanders SpaceVR(TM) June 2016
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
#include <unistd.h>   //Standard symbolic constants & types, needed to use usleep()

#include "JetsonGPIO.h"

using namespace std;

//Compiled using g++ GPIOTestApplication.cpp JetsonGPIO.cpp   -std=c++11   -o GPIOTestApplication

int main(int argc, char *argv[])
{
  //Arrays to hold command line input parameters
  unsigned int DIRECTION_ARRAY[MAX_GPIO_PINS];
  unsigned int VALUE_ARRAY[MAX_GPIO_PINS];
  
  //Example using 3 General Purpose Input/Output (GPIO) pins - 2 Output and 1 Input
  unsigned int NUMBER_OF_GPIO_PINS = 3;
  JetsonGPIO K1_GPIO_Pin[NUMBER_OF_GPIO_PINS];
  K1_GPIO_Pin[0] = JetsonGPIO(GPIO_PU0, OUTPUT_PIN, LOW);
  K1_GPIO_Pin[1] = JetsonGPIO(GPIO_PU1, OUTPUT_PIN, LOW);
  K1_GPIO_Pin[2] = JetsonGPIO(GPIO_PU2, INPUT_PIN, LOW);
  
  //Command line input error parsing 
  if(argc < 4){
    printf("Usage: %s n DIRECTION_ARRAY[n] VALUE_ARRAY[n]\n", argv[0]);
    printf("N: Number of GPIO pins to create (max=%d)\n", MAX_GPIO_PINS);
    printf("DIRECTION_ARRAY[%d]: Array to control direction of pins (INPUT PIN = 0 and OUTPUT PIN =1) \n", MAX_GPIO_PINS);
    printf("VALUE_ARRAY[%d]: Array to state of pins (LOW = 0 and HIGH =1)  \n\n", MAX_GPIO_PINS);
    exit(0);
  }
  
  sscanf(argv[1], "%d", &NUMBER_OF_GPIO_PINS);
  if(NUMBER_OF_GPIO_PINS > MAX_GPIO_PINS){
    printf("ERROR: You tried to create more GPIO pins then possible (MAX = %d) on the NVIDIA K1. \n", MAX_GPIO_PINS);
    printf("For the first command line argument please enter an integer less than or equal to %d.\n", MAX_GPIO_PINS);
    exit(0); 
  }
  
  //Parse the direction command line arguments 2 to (2+NUMBER_OF_GPIO_PINS) 	  
  for(int i = 0; i < NUMBER_OF_GPIO_PINS; i++){
    sscanf(argv[(2+i)], "%d", &DIRECTION_ARRAY[i]);
  }
  
  //Parse the value (output pins only) command line arguments (2+2*NUMBER_OF_GPIO_PINS+1) to (2+2*NUMBER_OF_GPIO_PINS+1)   
  for(int j = 0; j < NUMBER_OF_GPIO_PINS; j++){
    sscanf(argv[(2+(2*NUMBER_OF_GPIO_PINS)+1+j)], "%d", &VALUE_ARRAY[j]);  
  }
  
  //Setup time variables to track Mission Elapsed Time (MET)
  time_t now;
  struct tm MissionStart;
  double elapsedSeconds;
  
  auto start = chrono::high_resolution_clock::now();
  time(&now);  /* get current time; same as: now = time(NULL)  */
  
  MissionStart = *localtime(&now);
  MissionStart.tm_hour = 0; MissionStart.tm_min = 0; MissionStart.tm_sec = 0;
  MissionStart.tm_mon = 0;  MissionStart.tm_mday = 1;

  elapsedSeconds = difftime(now,mktime(&MissionStart));
  auto elapsed = chrono::high_resolution_clock::now() - start;
  long long elapsedMircoSeconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
 
 
  cout << "TEST #1:" << endl;	
  printf("Mission Elaspe Time (MET) rising edge TRIGGER timeStamp (i.e Month-MonthDay-Hour-Minutes-Seconds-MilliSeconds) = %d-%d-%d-%d-%f-", 
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
  
  cout << "Test complete! Have a nice day :)" << endl;
   	
}//END MAIN
