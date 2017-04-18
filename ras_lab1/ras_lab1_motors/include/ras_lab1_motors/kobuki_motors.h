/*
 *  kobuki_motors.h
 *
 *
 *  Created on: Aug 25, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef KOBUKI_MOTORS_H_
#define KOBUKI_MOTORS_H_

#include <vector>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>


class KobukiMotors
{
public:
    KobukiMotors();

    virtual ~KobukiMotors();

    //    updates kobuki motors
    //    input: size 2 vector of pwm signals (signal range between -255 and 255)
    //           pwm[0] --> left wheel
    //           pwm[1] --> right wheel
    //    output: size 2 (0: left wheel, 1: right wheel)
    //            angular_velocities: angular velocities of each wheel [rad]
    //            abs_encoders: absolute encoder values for each wheel
    //            diff_encoders: differential encoder values for each wheel
    void update(const std::vector<int> &pwm,
                std::vector<double> &angular_velocities,
                std::vector<int> &abs_encoders,
                std::vector<int> &diff_encoders);

private:

    // variables used for generating noise
    boost::mt19937 eng_;
    boost::normal_distribution<double> noise_distribution_;
    boost::variate_generator< boost::mt19937, boost::normal_distribution<double> > generator_;

    // stored absolute encoder values
    std::vector<int> abs_encoders_;
};

#endif
