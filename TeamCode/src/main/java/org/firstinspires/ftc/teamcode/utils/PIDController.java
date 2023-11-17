package org.firstinspires.ftc.teamcode.utils;

public class PIDController {

    float lastError = 0;

    float integralSum = 0;

    float Kp, Ki, Kd = 0;

    public PIDController(float Kp, float Ki, float Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public float update(float error, float dt) {

        float derivative;
        // rate of change of the error
        if (dt <= 0) {
            derivative = (error - lastError) / dt;
        } else  {
            derivative = 0;
        }
        // sum of all error over time
        if ((float) (error * dt) != 0.0f) integralSum += (error * dt);

        float out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;

        return out;
    }


}
