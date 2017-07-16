#ifndef PID_H
#define PID_H

class PID
{
private:
    /*
    * Errors
    */
    double m_errorP;
    double m_errorI;
    double m_errorD;

    /*
    * Coefficients
    */
    const double m_coeffP;
    const double m_coeffI;
    const double m_coeffD;

    /**
     * keep track of last error
     */
    double m_lastCTE;

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);


public:
    /*
    * Constructor, default values taken from lesson
    */
    // tuning for max speed 30
    //PID(const double p = 0.2, const double i = 0.004, const double d = 3.0);
    PID(const double p = 0.1, const double i = 0.001, const double d = 15.0);


    /*
    * Destructor.
    */
    virtual ~PID();

    /**
     * Calculate steering angle based on current PID error values,
     * range of steering angle should be [-1, 1]
     */
    double CalcSteeringAngle(const double cte);

    /**
     * limit speed to 30
     */   
    double CalcThrottle(const double speed);

};

#endif // PID_H
