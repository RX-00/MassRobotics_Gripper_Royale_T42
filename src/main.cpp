/* User demo program for the MassRobotics
 * Royale T42 Gripper
 * -Roy Xing, any questions please contact: roy@massrobotics.org
 */

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <string>
#include <unistd.h>

#include "RPMSerialInterface.h" // servos
#include "Utils.h"              // servo utility class for sleep & time methods


//NOTE: modified the RPM library to allow for greater servo range
#define SRVO_MAX   10000
#define SRVO_MIN   2000

void servo_test(RPM::SerialInterface *servosInterface, unsigned char channelNumber){
  std::cout << "Testing servo number: " << 11 << std::endl;
  for (int i = 0; i < 5; i++){
    std::cout << "min position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, SRVO_MIN);
    Utils::sleep(1000);
    std::cout << "middle position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, 6000);
    Utils::sleep(1000);
    std::cout << "max position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, SRVO_MAX);
    Utils::sleep(1000);
  }
  exit(1);
}

// open or close the gripper
void close_and_open(RPM::SerialInterface *servosInterface, unsigned char channelNumber){
  bool cont = true;
  while(cont){
    
  }
}

// move one individual servo
void move_servo(RPM::SerialInterface *servosInterface, unsigned char channelNumber, int servo_num){
  bool cont = true;
  while(cont){
    
  }
}

// moves both servos at the same time
void move_servos(RPM::SerialInterface *servosInterface, unsigned char channelNumber){
  bool cont = true;
  while(cont){
    
  }
}

void servo_control(RPM::SerialInterface *servosInterface, unsigned char channelNumber){
  bool cont = true;
  char option;
  std::cout << "Starting demo for MassRobotics Royale T42 Gripper..." << std::endl;

  while (cont){
    std::cout << "\n\nPlease choose an option" << std::endl;
    std::cout << "A: close & open gripper\n"
              << "B: move 0 servo\n"
              << "C: move 1 servo\n"
              << "D: move both servos\n"
              << "E: exit demo\n"
              << std::endl;
    std::cin >> option;
    switch(option){
    case 'A':
      close_and_open(servosInterface, channelNumber);
      break;
    case 'B':
      move_servo(servosInterface, channelNumber, 0);
      break;
    case 'C':
      move_servo(servosInterface, channelNumber, 1);
      break;
    case 'D':
      move_servos(servosInterface, channelNumber);
      break;
    case 'E':
      cont = false;
      break;
    default:
      std::cout << "Invalid input" << std::endl;
    }
  }

  std::cout << "Terminating demo..." << std::endl;
}

// function to test device over serial w/ sinusoidal signals
void sinusoid_signal(RPM::SerialInterface *serialInterface, unsigned char channelNumber){
  // Generate a sinusoid signal to send to the PololuInterface
  std::cout << "Sending sinusoidal signal to device to test device..." << std::endl;
	const float pi = 3.141592653589793f;
	const unsigned int channelMinValue = SRVO_MIN;
	const unsigned int channelMaxValue = SRVO_MAX;
	const unsigned int channelValueRange = channelMaxValue - channelMinValue;
	const unsigned int signalPeriodInMs = 2000;
	unsigned int time0 = Utils::getTimeAsMilliseconds();
	unsigned int timeSinceStart = 0;
	while ( timeSinceStart < 5000 ){
    float k = sin( (pi*2)/signalPeriodInMs * timeSinceStart ) * (float)(channelValueRange/2);
    float channelValue = (float)channelMinValue + (float)channelValueRange/2 + k;
    printf("\rchannelValue=%d", (unsigned int)channelValue );
    serialInterface->setTargetCP( channelNumber, (unsigned short)channelValue );
    timeSinceStart = Utils::getTimeAsMilliseconds() - time0;
    Utils::sleep(5);
  }
  printf("\n");
}

// function to create serial interface for the maestro servo controller
RPM::SerialInterface * serialInterfaceInit(unsigned char deviceNumber, unsigned char channelNumber, std::string portName){
  // create the interface for the maestro
  std::cout << "Serial interface init..." << std::endl;
	unsigned int baudRate = 115200;
	printf("Creating serial interface '%s' at %d bauds\n", portName.c_str(), baudRate);
	std::string errorMessage;
	RPM::SerialInterface* serialInterface = RPM::SerialInterface::createSerialInterface( portName, baudRate, &errorMessage );
	if ( !serialInterface ){
    printf("Failed to create serial interface. %s\n", errorMessage.c_str());
    std::cout << "Terminating program..." << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::cout << "Serial interface initiated\n" << std::endl;
  return serialInterface;
}


int main(int argc, char** argv){
  // Serial servo interface
  unsigned char deviceNumber = 12;
  unsigned char channelNumber = 11;
  std::string portName = "/dev/ttyACM0";
  RPM::SerialInterface *servosInterface = serialInterfaceInit(deviceNumber, channelNumber, portName);
  servosInterface -> SerialInterface::mMinChannelValue = SRVO_MIN;
  servosInterface -> SerialInterface::mMaxChannelValue = SRVO_MAX;

  servo_control(servosInterface, channelNumber);

  delete servosInterface;
  servosInterface = NULL;
  return 0;
}
