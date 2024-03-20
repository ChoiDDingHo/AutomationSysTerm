/**
 * @author Hugues Angelis
 *
 * @section LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * SHARP GP2 Analog sensor familly library
 *
 * Any GP2 analog sensor may be added to this library using the constructor :
 *      minDistance :   is the minimum distance for linear approximation (cm)
 *      maxDistance :   is the maximum distance for linear approximation (cm)
 *      slope :         is the slope of the linear part of the graph V = f(1/d)
 *                      take special care that we use the 1/d instead of d for x
 *                      axis.
 *      origin :        is the value of the origin (Y for X = 0) of the line of 
 *                      V=f(1/d)
 *  The slope parameter must be computed by user unless the GP2 reference is
 *  listed in the list below :
 *      - GP2Y0A02YK0F (Min = 30cm, Max = 150cm, Slope = 60, origin = 0)
 *      - GP2Y0A21YK0F (Min =  7cm, Max =  80cm, Slope = 0.246, origin = -0.297)
 *
 *  You may add others sensors if you wish it
 *
 *  One must be aware that under the minimum distance the sensor will output
 *  a false value of distance leading to strong errors.
 * @endsection
 */

#ifndef GP2A_H
#define GP2A_H

/**
 * Slope definitions for common GP2A sensors
 */
 
#define GP2Y0A02YK0F_SLOPE    60.0
#define GP2Y0A21YK0F_SLOPE    0.23625
#define GP2Y0A02YK0F_ORIGIN   0.0
#define GP2Y0A21YK0F_ORIGIN   -0.297

/**
 * Includes : Mbed library
 */
#include "mbed.h"

/**
 * GP2A Sensor : See sharp website for more informations
 *  http://www.sharp-world.com/products/device/lineup/selection/opto/haca/diagram.html
 *  @param slope = 60 for a GP2Y0A02 and 0.236 for a GP2Y0A21
 *  @param origin = 0 for a GP2Y0A02 and -0.297 for a GP2Y0A21
 */
class GP2A {

public :

    /**
     * Constructor of a Sharp GP2 Familly object.
     *
     * @param vmes (PinName) : the Mbed pin used to connect with GP2A sensor
     * @param dMin (double) : the GP2A sensor min distance to mesure
     * @param dMax (double) : the GP2A sensor max distance to mesure
     * @param slope (double) : the slope of the linear part of the graph V = f(1/d)
     * @param origin (double) : the origin (Y for X=0) of the linear pat of the graph V = f(1/d)
     * @note you better use slope and origin value that I've put above as sybolic constants
     */
    GP2A(PinName vmes, float dMin, float dMax, float slope, float origin = 0);

    /**
     * Return the distance to target mesured by sensor in cm
     *
     * @return Distance in cm
     * @note Distance is (very) approximative, to have a better mesurment I recommand an averaging of at least 5 measures
     * @note you may get a new mesurment every 50ms 
     */
    double getDistance (void);

    /**
     * Return the current voltage on GP2A output
     *
     * @return Voltage between 0 and 3.3V
     */
    double getVoltage (void);
    
    /** 
     * A short hand of getDistance
     */
    operator double();
    
    

private :
    float m_dMin, m_dMax, m_slope, m_origin;

protected :

    AnalogIn    _sensor ;

};
#endif //GP2A_H
