package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import android.hardware.SensorEvent;

@TeleOp
public class calibrateGyro  extends LinearOpMode {
    private double sum(List<Double> ar)
    {
        double count = 0;
        for (int i = 0; i < ar.size(); i++)
        {
            count += ar.get(i);
        }
        return count;
    }
    public void runOpMode()
    {
        AndroidAccelerometer accel = new AndroidAccelerometer();

        ElapsedTime time = new ElapsedTime();
        double lastTime = 0;
        Position pos = new Position();
        waitForStart();
        List<Double> values= new ArrayList<Double>();
        List<Double> accelValues = new ArrayList<Double>();
        time.reset();
        accel.startListening();
        boolean toggle = true;
        values.add((double)0);
        while (opModeIsActive())
        {
            //values.add(gyro.getVoltage());
            double deltaTime = (double)time.time(TimeUnit.MILLISECONDS)/(double)1000;
            double test = deltaTime;
            deltaTime = deltaTime-lastTime;
            lastTime = test;
            time.reset();
            values.add((accel.getX()+0.3)*deltaTime);
            double x = values.get(0);
            for (int i = 1; i < values.size()-2; i++)
            {
                if (toggle)
                {
                    x += 4*values.get(i);
                    toggle = !toggle;
                }
                else
                {
                    x += 2*values.get(i);
                }
            }
            x += values.get(values.size()-1);
            x = x * (((double)time.time(TimeUnit.MILLISECONDS)/(double)1000)/3);
            telemetry.addData("x", x);
            telemetry.update();

        }

    }

}
