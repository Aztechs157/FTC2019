package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.ArrayList;

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
        AnalogGyro gyro = new AnalogGyro(hardwareMap.get(AnalogInput.class, "gyro"));
        ElapsedTime time = new ElapsedTime();
        waitForStart();
        List<Double> values= new ArrayList<Double>();
        time.reset();
        gyro.start();
        while (opModeIsActive())
        {
            //values.add(gyro.getVoltage());
            gyro.update();
            telemetry.addLine("gyro").addData("angle", gyro.angle
            ).addData("test", gyro.test).addData("value", gyro.gyro.getVoltage());
            telemetry.update();

        }

    }

}
