#ifndef PID_H
#define PID_H

#define DEFAULT_KP  1.1
#define DEFAULT_KI  0.00001
#define DEFAULT_KD  0.5

class PID
{
private:
    float Kp;
    float Ki;
    float Kd;
    float int_val;
    float last_err;
    float min_val;
    float max_val;

public:
    // Constructor
    PID();
    // Init
    void init(float Kp, float Ki, float Kd, float min_val, float max_val);
    // Reset the integral term
    void reset();
    // Apply the PID controller
    float step(float err, float sample_time);
};

#endif // PID_H
