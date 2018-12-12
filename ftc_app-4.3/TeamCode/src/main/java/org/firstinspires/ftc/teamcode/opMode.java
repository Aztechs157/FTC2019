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
    //Initializes the variables
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
        //Sets the drive mode, determines which stick is used to drive.
        if (input.back)
        {
            inverseControls = !inverseControls;
            telemetry.addData("InvertControls", inverseControls);
            telemetry.update();
        }
    }

    private double abs(double x)
    {
        return Math.abs(x);
    }

    double[] setmovement(Gamepad input)
    {
        //Sets the motor values for directional movement.
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

    double[] turning(double[] motors, Gamepad input1, boolean inverseControls)
    {
        //Adjusts the motor values for turning.
        double x;
        double y;
        if (inverseControls) {
            x = input1.left_stick_x;
            y = input1.left_stick_y;
        } else {
            x = input1.right_stick_x;
            y = input1.right_stick_y;
        }

        double turningRate = input1.right_trigger - input1.left_trigger;
        if (turningRate >= 0) {
            if (-x >= abs(y)) {
                motors[0] = motors[0] - 2 * (y - (-abs(x))) * turningRate + (1 + x) * turningRate;
            } else if (y > abs(x)) {
                motors[0] += (1 - y) * turningRate;
            } else if (x >= abs(y)) {
                motors[0] += (1 - x) * turningRate;
            } else {
                motors[0] += (1 + y) * turningRate;
            }


            if (abs(x) <= y) {
                motors[1] = motors[1] + 2 * (y + abs(x)) * turningRate + (1 - y) * turningRate;
            } else if (-y >= abs(x)) {
                motors[0] += (1 + y) * turningRate;
            } else if (x > abs(y)) {
                motors[0] += (1 - x) * turningRate;
            } else {
                motors[0] += (1 + x) * turningRate;
            }


            if (x >= abs(y)) {
                motors[2] = motors[2] + 2 * (abs(y) - x) * turningRate + (1 - x) * turningRate;
            } else if (y > abs(x)) {
                motors[2] += (1 - y) * turningRate;
            } else if (-x >= abs(y)) {
                motors[2] += (1 + x) * turningRate;
            } else {
                motors[3] += (1 - y) * turningRate;
            }

            if (abs(x) <= y) {
                motors[3] = motors[3] - 2 * (y - abs(x)) * turningRate + (1 - y) * turningRate;
            } else if (x > abs(y)) {
                motors[3] += (1 - x) * turningRate;
            } else if (-y >= abs(x)) {
                motors[3] += (1 + y) * turningRate;
            } else {
                motors[3] += (1 + x) * turningRate;
            }
        } else {
            if (-x >= abs(y)) {
                motors[0] = motors[0] + 2 * (-abs(x) - x) * turningRate + (1 + x) * turningRate;
            } else if (-y > abs(x)) {
                motors[0] += (1 + y) * turningRate;
            } else if (x >= abs(y)) {
                motors[0] += (1 - x) * turningRate;
            } else {
                motors[0] += (1 - y) * turningRate;
            }

            if (y >= abs(x)) {
                motors[1] = motors[1] - 2 * (-abs(y) + x) * turningRate + (1 - y) * turningRate;
            } else if (x > abs(y)) {
                motors[1] += (1 - x) * turningRate;
            } else if (-y >= abs(x)) {
                motors[1] += (1 + y) * turningRate;
            } else {
                motors[1] += (1 + x) * turningRate;
            }

            if (x >= abs(y)) {
                motors[2] = motors[2] - 2 * (y - abs(x)) * turningRate + (1 - x) * turningRate;
            } else if (y > abs(x)) {
                motors[2] += (1 - y) * turningRate;
            } else if (-x >= abs(y)) {
                motors[2] += (1 + x) * turningRate;
            } else {
                motors[2] += (1 + x) * turningRate;
            }

            if (-x >= abs(y)) {
                motors[3] = motors[3] - 2 * (-abs(y) - x) * turningRate + (1 + x) * turningRate;
            } else if (-y > abs(x)) {
                motors[3] += (1 + y) * turningRate;
            } else if (x >= abs(y)) {
                motors[3] += (1 - y) * turningRate;
            } else {
                motors[3] += (1 - x) * turningRate;
            }
        }

        return motors;
    }


    public void drive(double[] motors)
    {
        //Sets the power of the motors based on the motor values
        //Creates Telemetry data for the motors.
        driveMotors[0].setPower(motors[0]);
        driveMotors[1].setPower(motors[1]);
        driveMotors[2].setPower(motors[2]);
        driveMotors[3].setPower(motors[3]);
        telemetry.addData("Motor0", motors[0]);
        telemetry.update();
        telemetry.addData("Motor1", motors[1]);
        telemetry.update();
        telemetry.addData("Motor2", motors[2]);
        telemetry.update();
        telemetry.addData("Motor3", motors[3]);
        telemetry.update();
    }

    private double n(double x)
    {
        return -abs(x);
    }

    /*public void intakeposition(Gamepad input, Servo servos[], boolean intakeOut)
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
    }*/

    public void actuator(Gamepad input1)
    {
        miscMotors[0].setPower(input1.right_stick_y);
        /*
        //Controls the position of the actuator, with telemetry.
        float target;
        if (input1.x)
        {
            actuator = !actuator;
            telemetry.addData("actuator", actuator);
            telemetry.update();
        }
        if (actuator)
        {
            target = 100; //TODO: not correct value
        }
        else
        {
            target = 0;
        }
        float val = (float) actuatorController.pidCalculate(target, miscMotors[0].getCurrentPosition());
        miscMotors[0].setPower(val);*/
    }

    /*public void intake(Gamepad input, Servo servos[])
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
    }*/

    /*public void augur(Gamepad input, DcMotor miscMotors[])
    {
        if (input.left_trigger > 0 && input.b)
        {
            miscMotors[1].setPower(-input.left_trigger);
        }
        else if (input.left_trigger > 0)
        {
            miscMotors[1].setPower(input.left_trigger);
        }
    }*/

    public void intake(Gamepad input, DcMotor miscMotors[])
    {
        //Controls the speed and direction of the intake.
        if (input.right_trigger > 0 && input.a)
        {
            //reverse
            miscMotors[1].setPower(-input.right_trigger);
            miscMotors[2].setPower(input.right_trigger);
        }
        else if (input.right_trigger > 0)
        {
            //forward
            miscMotors[1].setPower(input.right_trigger);
            miscMotors[2].setPower(-input.right_trigger);
        }
    }

    @Override
    public void runOpMode()
    {
        //Defines the drive motors
        driveMotors = new DcMotor[]{hardwareMap.get(DcMotor.class, "drive1"),
                                    hardwareMap.get(DcMotor.class, "drive2"),
                                    hardwareMap.get(DcMotor.class, "drive3"),
                                    hardwareMap.get(DcMotor.class, "drive4")};
        //defines the gamepads
        driver = this.gamepad1;
        operator = this.gamepad2;
        //TODO: make sure gamepads are assigned right
        //Sets up a system to hold the actuator in a position.
        actuatorController = new PID(0.01, 0, 0.00000, 999999,
                                     99999, 999999, 9999999);
        //Defines the servos in an array.
        servos = new Servo[]{hardwareMap.get(Servo.class, "intake1"),
                             hardwareMap.get(Servo.class, "intake2")};
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            //Main code for the robot, the stuff that actually does stuff.
            drivemode(driver);
            double motors[] = setmovement(driver);
            telemetry.addData("value", driver.left_stick_x);
            telemetry.update();
            turning(motors, driver, inverseControls);
            drive(motors);
            actuator(operator);
            //intakeposition(operator, servos, intakeOut);
            intake(operator, miscMotors);
            //augur(operator, miscMotors);

        }
    }
}



