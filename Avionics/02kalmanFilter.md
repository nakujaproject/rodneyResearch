# Kalman Filter

As I mentioned earlier, it's nearly impossible to grasp the full meaning of Kalman Filter by starting from definitions and complicated equations (at least for us mere mortals).

Our purpose is to find Estimate of X at state k, the estimate of the signal x. And we wish to find it for each consequent k's.

> Kalman filter finds the most optimum averaging factor for each consequent state. Also somehow remembers a little bit about the past states.

## General rule of thumb

![kalman process](img/kalmanProcess.png)

## Single variable kalman filter

In this example we will be using the bmp180 sensor. This sensor measures the altitude above sea level of its position. It will help us determine the height of the rocket. Out of the box we install the `<Adafruit_BMP085.h` library. Using the example sketch from the library, since we are interested in the altitude we remove the other code and remain with altitude measurements

```c++
#include <Wire.h>
#include <Adafruit_BMP085.h>
#define seaLevelPressure_hPa 1024

Adafruit_BMP085 bmp;
  
void setup() {
    Serial.begin(115200);
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }
}
  
void loop() {
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(seaLevelPressure_hPa * 100));
    Serial.println(" meters");
    delay(500);
}
```

If we plot this values while maintaining a stationary point we can be able to see the noise in the readings.

!TODO Insert image of noise readings from a single point

For a start we will try to estimate a scalar constant, altitude reading from a source. So we assume the constant value is `x metres` and we have some noise in the data above and below `x metres`. The get the standard deviation from the datasheet of the instrument we are trying to read.

## Let's build our model

![Singel variable kalman filter](img/singlevarKalman.png)

In each step we have tried and reduced the equations to a very simple form.
Above all, we have a 1 dimensional signal problem, so every entity in our model is a numerical value, not a matrix.

We have no such control signal `uk`, and it's out of the game. As the signal is a constant value, the constant A is just 1, because we already know that the next value will be same as the previous one. The value H = 1, because we know that the measurement is composed of the state value and some noise. Finally, let's assume that we have the following measurement values:

```txt
1558.04
1558.04
1557.47
1557.95
1557.66
1557.76
1558.04
1558.24
1557.76
```

OK, we should start from somewhere, such as k=0. We should find or assume some initial state. Here, we throw out some initial values. Let's assume estimate of X0 = 0, and P0 = 1. Then why didn't we choose P0 = 0 for example? It's simple. If we chose that way, this would mean that there's no noise in the environment, and this assumption would lead all the consequent Estimate of X at state k to be zero (remaining as the initial state). So we choose P0 something other that zero.

![Iteration values](img/singleUpdates.jpeg)

## Multiple variable kalman

![Mutpliple variable process](img/processKalman.png)