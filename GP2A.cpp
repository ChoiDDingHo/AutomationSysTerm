#include "GP2A.h"

GP2A::GP2A(PinName vmes, float dMin, float dMax, float slope, float origin) : _sensor(vmes)
{
    m_dMin = dMin;
    m_dMax = dMax;
    m_slope = slope;
    m_origin = origin;
}

double GP2A::getDistance (void)
{
    double vDist = (double)_sensor.read()* 3.3;
    float distance = m_slope/(vDist-m_origin);
    if (distance > m_dMax) return m_dMax;
    if (distance < m_dMin) return m_dMin;
    return distance;
}

double GP2A::getVoltage (void)
{
    return (double)_sensor.read()* 3.3;
}

GP2A::operator double() {
    return getDistance();
}
