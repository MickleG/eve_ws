#include <stdio.h> 
#include <stdlib.h>
#include <wiringPi.h>   // Include WiringPi library!
#include <time.h>       // For NANOS function
#include <iostream>
#include <cmath>        // For sqrt and other math functions

// #include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>
#include <cmath>
#include "eve_main/Cutter.h"
#include <unistd.h>

using namespace std;

// class Cutter 
// {

    Cutter::Cutter(uint8_t pin1)
    {
        wiringPiSetupGpio(); // Initialize wiringPi -- using Broadcom pin numbers
        cutPin = pin1;

        pinMode(cutPin, OUTPUT);

        maxCutAttempts = 1;
    }

    void Cutter::setAttempts(uint8_t newAttempts)
    {
        maxCutAttempts = newAttempts;
    }
		
	void Cutter::resetCut() {
		digitalWrite(cutPin, 0);
	}

    void Cutter::cutPlant()
    {
        digitalWrite(cutPin, 1);
        usleep(500000);
        digitalWrite(cutPin, 0);
        usleep(500000);
    }

// }
