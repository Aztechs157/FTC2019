package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp
public class opMode extends LinearOpMode {
    DcMotor driveMotors[] = {null, null, null, null};
    DcMotor miscMotors[] = {null, null};
    boolean inverseControls = false;
    boolean intakeOut = false;
    Gamepad driver = null;
    Gamepad operator = null;
    boolean actuator = false;
    PID actuatorController;
    double motors[] = {null, null, null, null, null};
    double servos[] = {0, 0};

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

    public void intakeposition(Gamepad input, ServoController servos[], boolean intakeOut)
    {
        if (input.y && !intakeOut)
        {
            servos[2].setServoPosition(2, 90);
            servos[3].setServoPosition(3, 90);
            intakeOut = true;
        }
        else if (input.y && intakeOut)
        {
            servos[2].setServoPosition(2, 0);
            servos[3].setServoPosition(3, 0);
            intakeOut = false;
        }
    }

    public void actuator(Gamepad input1)
    {
        float target;
        if (input1.x)
        {
            actuator = !actuator;
        }
        if (actuator)
        {
            target = 100; //TODO: not correct value
        }
        else
        {
            target = 0;
        }
        float val = (float)actuatorController.pidCalculate(target,
                miscMotors[0].getCurrentPosition());
        miscMotors[0].setPower(val);
    }

    public void intake(Gamepad input, ServoController servos[])
    {
        if (input.right_trigger > 0 && input.a)
        {
            servos[0].setServoPosition(0, (1 - input.right_trigger) * 90);
            servos[1].setServoPosition(1, input.right_trigger * 90 + 90);
        }
        else if (input.right_trigger > 0)
        {
            servos[0].setServoPosition(0, input.right_trigger * 90 + 90);
            servos[1].setServoPosition(1, (1 - input.right_trigger) * 90);
        }
    }

    public void augur(Gamepad input, miscMotors[])
    {
        if (input.left_trigger > 0 && input.b)
        {
            miscMotors[1].setPower(-input.left_trigger);
        }
        else if (input.left_trigger > 0)
        {
            miscMotors[1].setPower(input.left_trigger);
        }
    }

    @Override
    public void runOpMode()
    {
        while (opModeIsActive())
        {
            drivemode(driver);
            setmovement(driver, inverseControls);
            //turning(motors[], driver, inverseControls);
            drive(motors[]);
            actuator(driver);
            intakeposition(operator, servos[], intakeOut);
            intake(operator, servos[]);
            augur(operator, miscMotors[]);
            
        }
    }
}



