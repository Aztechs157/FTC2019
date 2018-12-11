package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.sun.tools.javac.comp.Todo;

@TeleOp
public class opMode extends LinearOpMode
{
    private DcMotor driveMotors[] = {null, null, null, null};
    private DcMotor miscMotors[] = {null, null};
    private boolean inverseControls = false;
    private boolean intakeOut = false;
    private Gamepad driver = null;
    private Gamepad operator = null;
    private boolean actuator = false;
    private PID actuatorController;
    private double motors[] = {0, 0, 0, 0, 0};
    private Servo servos[] = {null, null};


    void drivemode(Gamepad input)
    {
        if (input.back)
        {
            inverseControls = !inverseControls;
        }
    }

    private double abs(double x)
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
        if (x >= 0 && y >= 0)
        {
            motors[0] = y - x;
            motors[1] = Math.min(-x, -y);
            motors[2] = x - y;
            motors[3] = Math.max(x, y);
        }
        else if (x >= 0)
        {
            motors[0] = Math.min(-x, y);
            motors[1] = abs(y) - x;
            motors[2] = Math.max(x, -y);
            motors[3] = x - abs(y);
        }
        else if (x < 0 && y >= 0)
        {
            motors[0] = Math.max(-x, y);
            motors[1] = abs(x) - y;
            motors[2] = Math.min(x, -y);
            motors[3] = y - abs(x);
        }
        else
        {
            motors[0] = abs(x) - abs(y);
            motors[1] = Math.max(-x, -y);
            motors[2] = abs(y) - abs(x);
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

    private double n(double x)
    {
        return -abs(x);
    }

    public void intakeposition(Gamepad input, Servo servos[], boolean intakeOut)
    {
        if (input.y && !intakeOut)
        {
            servos[2].setPosition(90);
            servos[3].setPosition(90);
            intakeOut = true;
        }
        else if (input.y && intakeOut)
        {
            servos[2].setPosition(0);
            servos[3].setPosition(0);
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
        float val = (float) actuatorController.pidCalculate(target,
                                                            miscMotors[0].getCurrentPosition());
        miscMotors[0].setPower(val);
    }

    public void intake(Gamepad input, Servo servos[])
    {
        if (input.right_trigger > 0 && input.a)
        {
            servos[0].setPosition(-input.right_trigger);
            servos[1].setPosition(input.right_trigger);
        }
        else if (input.right_trigger > 0)
        {
            servos[0].setPosition(input.right_trigger);
            servos[1].setPosition(-input.right_trigger);
        }
    }

    public void augur(Gamepad input, DcMotor miscMotors[])
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
        driveMotors = new DcMotor[]{hardwareMap.get(DcMotor.class, "drive1"),
                                    hardwareMap.get(DcMotor.class, "drive2"),
                                    hardwareMap.get(DcMotor.class, "drive3"),
                                    hardwareMap.get(DcMotor.class, "drive4")};
        gamepad1 = new Gamepad();
        gamepad2 = new Gamepad();
        //TODO: make sure gamepads are assigned right
        actuatorController = new PID(0.01, 0, 0.00000, 999999,
                                     99999, 999999, 9999999);
        servos = new Servo[]{hardwareMap.get(Servo.class, "intake1"),
                             hardwareMap.get(Servo.class, "intake2")};
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive())
        {
            drivemode(driver);
            double motors[] = setmovement(driver);
            //turning(motors[], driver, inverseControls);
            drive(motors);
            //actuator(driver);
            //intakeposition(operator, servos, intakeOut);
            //intake(operator, servos);
            //augur(operator, miscMotors);

        }
    }
}



