package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class AnalogGyro {
    double angle = 0;
    double normal = 400;
    double rate = 0.00462;
    AnalogInput gyro;
    ElapsedTime time;
    public AnalogGyro(AnalogInput gyro) {
        this.gyro = gyro;
    }
    public void start()
    {
        time = new ElapsedTime();
        time.reset();
    }
    public void update()
    {
        double newAngle = this.gyro.getVoltage();
        newAngle = (normal-newAngle)/rate;
        newAngle /= time.time(TimeUnit.SECONDS); //TODO: seconds is int, not double
        time.reset();
        angle += newAngle;
    }
}
