#include "PID.h"
#include "Arduino.h"
// constructor to initialize pid gians
PID::PID(float KP, float KI, float KD, float IMAX) {
    kp = KP;
    ki = KI;
    kd = KD;
    imax = IMAX; // to prevent integral windup
}

float PID::get_pid(float error, float scalar, unsigned long current_millis) {
    unsigned long t_now = current_millis;
    unsigned long dt = t_now - last_t;

    // First run: initialize and return 0
    if (last_t == 0) {
        last_t = t_now;
        last_error = error;
        integrator = 0;
        return 0.0f;
    }

    last_t = t_now;
    float delta_time = dt * 0.001f;

    // --- P term ---
    float output = kp * error;

    // --- D term ---
    float derivative = 0.0f;
    if (kd != 0 && dt > 0) {
        derivative = (error - last_error) / delta_time;
        output += kd * derivative;
    }

    // always update last error
    last_error = error;

    // --- I term ---
    if (ki != 0 && dt > 0) {
        integrator += error * ki * delta_time;

        // clamp integrator
        if (ABS(integrator) > imax) {
            integrator = (integrator > 0 ? imax : -imax);
        }

        output += integrator;
    }

    return output;
}

void PID::update_gains(float newKp, float newKi, float newKd, float newImax) {
    kp = newKp;
    ki = newKi;
    kd = newKd;
    imax = newImax;
}


void PID::reset_I() { integrator = 0.0f; }
