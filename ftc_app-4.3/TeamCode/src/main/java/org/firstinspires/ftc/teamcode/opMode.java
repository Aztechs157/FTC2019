package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class opMode extends LinearOpMode {
    DcMotor driveMotors[] = {null, null, null, null};
    DcMotor miscMotors[] = {null, null};
    boolean inverseControls = false;
    boolean intakeOut = false;
    Gamepad driver = null;
    Gamepad operator = null;

    void drivemode(Gamepad input)
    {
        if (input.back)
        {
            inverseControls = !inverseControls;
        }
    }
    double abs(double x)
    {
        return Math.abs(x);
    }

    double[] setmovement(Gamepad input)
    {
        double motors[] = {0, 0, 0, 0, 0};
        double x = input.right_stick_x;
        double y = input.right_stick_y;
        if (inverseControls)
        {
            x = input.left_stick_x;
            y = input.left_stick_y;
        }
        if (x>=0 && y >= 0)
        {
            motors[0] = y-x;
            motors[1] = Math.min(-x, -y);
            motors[2] = x-y;
            motors[3] = Math.max(x, y);
        }
        else if (x>=0)
        {
            motors[0] = Math.min(-x, y);
            motors[1] = abs(y)-x;
            motors[2] = Math.max(x, -y);
            motors[3] = x-abs(y);
        }
        else if (x<0 && y >= 0)
        {
            motors[0] = Math.max(-x, y);
            motors[1] = abs(x)-y;
            motors[2] = Math.min(x, -y);
            motors[3] = y-abs(x);
        }
        else
        {
            motors[0] = abs(x)-abs(y);
            motors[1] = Math.max(-x, -y);
            motors[2] = abs(y)-abs(x);
            motors[3] = Math.min(x, y);
        }
        motors[4] = motors[0] + motors[1] + motors[2] + motors[3];
        return motors;
    }
    public void drive(double[] motors)
    {
        driveMotors[0].setPower(motors[0]);
        driveMotors[1].setPower(motors[1]);
        driveMotors[2].setPower(motors[2]);
        driveMotors[3].setPower(motors[3]);
    }
    double n(double x)
    {
        return -abs(x);
    }

    @Override
    public void runOpMode() {

    }
}



