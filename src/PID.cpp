#include <cmath>
#include <stdio.h>


#include "PID.h"

using namespace std;


// TODO: Complete the PID class.

PID::PID(const double p, const double i, const double d)
    : m_errorP(0.0)
    , m_errorI(0.0)
    , m_errorD(0.0)
    , m_coeffP(p)
    , m_coeffI(i)
    , m_coeffD(d)
    , m_lastCTE(0.0)
{
}

PID::~PID() 
{
}

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

double PID::CalcSteeringAngle(const double cte)
{
    // range of steering angle should be [-1, 1]

    UpdateError(cte);

    double steeringAngle =
            m_coeffP * m_errorP
        +   m_coeffI * m_errorI
        +   m_coeffD * m_errorD;
    
    if (steeringAngle > 1.0)
    {
        steeringAngle = 1.0;
    }
    else if (steeringAngle < -1.0)
    {
        steeringAngle = -1.0;
    }
    
    printf("steering=%.3f, p=%.3f, i=%.3f, d=%.3f\n",
        steeringAngle, m_errorP, m_errorI, m_errorD);

    return -steeringAngle;
}

double PID::CalcThrottle(const double speed)
{
    double throttle(0.0);
    
    if (speed < 30.0)
    {
        throttle = 0.3;
    }

    return throttle;
}



