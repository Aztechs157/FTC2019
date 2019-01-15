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

    double angle = 0;
    double normal = 2.255;
    double rate = 0.002;
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
        class T extends Thread{
            public ArrayList<Double> values = new ArrayList<Double>();
            public ArrayList<Long> times = new ArrayList<Long>();
            public Double normal;
            private AnalogInput gyro;
            private ElapsedTime time;
            public void run(){
                synchronized (normal) {
                    gyro = hardwareMap.get(AnalogInput.class, "gyro");
                    time = new ElapsedTime();
                    normal = gyro.getVoltage();
                }

                while (!Thread.interrupted())
                {
                    synchronized (values){ synchronized (times){
                        values.add(gyro.getVoltage());
                        times.add(time.now(TimeUnit.MILLISECONDS));}}
                    time.reset();
                }
            }
        }
        //
        double sumVal = 0;
        //ElapsedTime time = new ElapsedTime();
        waitForStart();
        //time.reset();
        T thread = new T();
        thread.start();
        while (opModeIsActive())
        {
            telemetry.addData("hello", "world"); //only works if this line is there
            telemetry.update(); //DO NOT DELETE
            sleep(20);
            normal = thread.normal;
            ArrayList<Double> vals = thread.values;
            ArrayList<Long> times = thread.times;
            double sum = 0;
            for (int i = 0; i < vals.size()-1; i++)
            {
                sum += ((normal-vals.get(i))/rate)*times.get(i);
            }
            telemetry.addData("angle?", sum);
        }
        thread.interrupt();
    }
}

