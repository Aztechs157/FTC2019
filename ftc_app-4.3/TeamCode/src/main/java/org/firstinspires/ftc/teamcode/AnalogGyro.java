package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class AnalogGyro {
    double angle = 0;
    double normal = 2.26;
    double rate = 0.002;
    AnalogInput gyro;
    ElapsedTime time;
    double test;
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
        this.test = newAngle;
        newAngle = newAngle/((double)time.time(TimeUnit.MILLISECONDS)/1000);
        time.reset();
        angle += newAngle;
    }
}
