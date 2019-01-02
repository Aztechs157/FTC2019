package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.ArrayList;

@TeleOp
public class calibrateGyro  extends LinearOpMode {
    private double sum(ArrayList<Double> ar)
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
        AnalogInput gyro = hardwareMap.get(AnalogInput.class, "gyro");
        ElapsedTime time = new ElapsedTime();
        waitForStart();
        List<Double> values= new ArrayList<Double>();
        time.reset();
        while (opModeIsActive())
        {
            values.add(gyro.getVoltage());
            telemetry.addData("average", sum(values)/(double)values.size());

        }

    }

}
