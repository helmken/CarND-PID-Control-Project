## PID Controller Project

The goals / steps of this project are the following:

* implement a PID controller
* tune parameters of PID controller so that the vehicle can drive a lap without without touching road boundaries  
* document parameter tuning

## Rubric Points
* a PID controller should be implemented
* the vehicle should drive one lap without touching road boundaries
 * no minimum speed is defined
* parameter tuning should be documented

---
### Writeup

#### 1. PID controller implementation
A PID controller is implemented in the files PID.h and PID.cpp. 
The initialization is done with default parameters in the constructor.
The PID logic is contained in the function `double CalcSteeringAngle(const double cte)` which is called from the `main()` function. Additionally the function `double PID::CalcThrottle(const double speed)` limits the driving speed to 30 mph by setting the throttle to zero if the speed exceeds 30 mph.

### 2. The vehicle should drive one lap without touching road boundaries
The PID parameters have been tuned, so that the vehicle does not touch road boundaries.
The tuned parameters are P=0.1, I=0.001 and D=15.0.

### 3. Document parameter tuning
In a first attempt, I started with a basic PID implementation and faithfully used Sebastian's parameters from the PID lesson.
Result: The vehicle oscillated strongly and went off the road before reaching the first curve.

I then set the parameters I and D to zero and found that with P=0.1 the vehicle reached successfully the first curve with small oscillations and then went off with oscillations.

To make the parameter tuning a little bit more deterministic I limited the speed of the vehicle to 30 mph by setting throttle to zero after speed exceeded 30 mph. The logic is contained in the function `PID::CalcThrottle(...)`. 

To suppress the oscillations, I started tuning the differential parameter P and increased the value from the lesson D=3.0 to D=15.0 until the vehicle followed the road well enough.

The remaining problem now was that the vehicle was driving on the outer side of curves. To address this, I started tuning the integral parameter I: The value from the lesson I=0.004 caused immediately strong oscillations, so I reduced it to I=0.001.
As explained in the lesson and literature, the I parameter should compensate for systematic or static errors. In our case, it should compensate while driving through curves: Although the vehicle steers proportionally to the distance from the center line, it will not make it to the middle of the road, because of the curvature.
The I parameter caused overshooting through the accumulation of errors. To tackle this, I reset the I-error value to zero, when the sign of the error changes, i.e. when the vehicle crosses the center line.
To limit the effect of the integral error I also added an upper bound of 300 to it.

The code for the error update looks like this:
```
void PID::UpdateError(double cte) 
{
    m_errorP = cte;

    if ( cte * m_lastCTE < 0.0)
    {
        // reset integral error on overshooting
        m_errorI = 0.0;
    }
    else
    {
        m_errorI += cte;
    }

    const double maxIntegralError(300);
    if (m_errorI > maxIntegralError)
    {
        m_errorI = maxIntegralError;
    }
    else if (m_errorI < -maxIntegralError)
    {
        m_errorI = -maxIntegralError;
    }

    m_errorD = cte - m_lastCTE;
    m_lastCTE = cte;
}
```

#### How the parameters contribute to the control
* P=proportional: This is the main steering control, but it causes overshooting and oscillations
* D=differential: This is in my opinion the second most important parameter, it supports the P parameter based on the change of error and suppresses overshooting and oscillations
* I=integral: This is in my opinion a parameter for fine tuning the control. In the given use case for steering a vehicle, it works mainly in curves and helps to keep the vehicle in the center of the road. Through its additive nature it can cause oscillations
