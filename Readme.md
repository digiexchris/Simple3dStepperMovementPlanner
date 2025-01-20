# Simple3dStepperMovementPlanner

This library is used to generate a vector containing the steps and delays between steps for a 3-dimensional straight line movement for common stepper motors.

## Features

- Generate a 3d vector containing all of the step bits for the full acceleartion, coast, and deceleration profile
- Calculate delays between steps

## Installation

The easiest way to get started with a CMake project is with FetchContent

```
FetchContent_Declare(
  StepTimingPlanner
  GIT_REPOSITORY https://github.com/digiexchris/Simple3dStepperMovementPlanner.git
  GIT_TAG 0.1.0
)

FetchContent_MakeAvailable(StepTimingPlanner)

target_link_libraries(YourProject StepTimingPlanner)

```

## Usage

Hint: Take a look at the tests for more advanced usage.

```cpp
#include "StepTimingPlanner.h"

using namespace StepTimingPlanner;

// First, define the limits for your axes:
// { acceleration in steps per second squared, deceleration, maximum speed in steps per second}
AxisLimits xLimits = {1000, 1000, 500};
AxisLimits yLimits = {1000, 1000, 400};
AxisLimits zLimits = {1000, 1000, 300};

//instantiate the generator:
Generator generator(xLimits, yLimits, zLimits);

//make a start and an end vector (positions are in steps)
//Vector3Int32(X, Y, Z)
Vector<uint16_t> start = {0, 0, 0};
Vector<uint16_t> end = {200, 10, 220};

//make a vector to indicate if the system is already in motion, and if so, what direction it's moving
//to move from a stop, just set this to zero
Vector<uint16_t> startingSpeedDirection({0, 0, 0});

//or if it's already moving along the X axis at 100 steps per second (such as, we're transitioning from another move that did not end in the system completely stopped):
Vector<uint16_t> startingSpeedDirection({100, 0, 0});

//Generate the steps:
uint16_t requestedSpeed = 3000;
uint16_t endingSpeed = 0;
std::vector<StepTiming> steps = generator.GenerateSteps(
      start, end, startingSpeedDirection, requestedSpeed, endingSpeed);


//you can now execute the steps using whatever IO system you use, as long as it expects a pulse followed by a delay to the next pulse. 
//the following makes the assumption that the stepper driver is expecting a 50% duty cycle.

//NOTE: It's up to you to set your driver for the correct direction before this move is executed. This library only determines if a step should be generated, wether the step is a negative or positive step, and how long to wait for the next check.

for(StepTiming step : steps) {

    uint32_t halfDelay = steps.delay/2;

    if(step.deltas[0]) { //uint8_t of either -1, 0 or 1, indicating a step should be executed for this axis
    //it's up to you to handle reversing the driver for -1 if required.
        digitalWrite(xAxisStepPin, HIGH);
    }

    if(step.deltas[1]) { 
        digitalWrite(yAxisStepPin, HIGH);
    }

    if(step.deltas[1]) { 
        digitalWrite(zAxisStepPin, HIGH);
    }

    sleepMicros(halfDelay);

    digitalWrite(xAxisStepPin, LOW);
    digitalWrite(yAxisStepPin, LOW);
    digitalWrite(zAxisStepPin, LOW);
}
```

Also, just because it's using 3d vectors does not mean that you need to use all of them. If you're moving just one axis, feel free to set only one axis's start and end coordinates, just be aware that the system is currently expecting all 3 to be present.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.