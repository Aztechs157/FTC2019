/**
 *  This is the Tele-Op code for the 2018-2019 ftc robot for team 12566. This code was a joint effort
 * between team members Jeremy Patrick and Tyler Silva. This code was written over the course a few
 * weeks. It was written in the Java programming language. We used Github for version control and
 * collaboration. We wrote the design for the code in Notepad++.
 *  We still have plenty of room to improve the code, for both functionality and elegance. We can
 * expand commenting in order to improve the readabiltity of the code. We can also add more
 * functionality to the telemetry section in order to make it more useful.
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.sun.tools.javac.comp.Todo;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

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
    private AndroidOrientation orientation;
    private boolean marker = true;


    void drivemode(Gamepad input)
    {
        //Sets the drive mode, determines which stick is used to drive.
        if (input.back)
        {
            inverseControls = !inverseControls;
        }
    }

    private double abs(double x)
    {
        return Math.abs(x);
    }

    private double[] orient(Gamepad input)
    {
        double angle = Math.atan2(input.right_stick_x, input.right_stick_y);
        angle = -angle+orientation.getPitch();
        double magnitude = Math.sqrt(Math.pow(input.right_stick_x, 2)+Math.pow(input.right_stick_y, 2));
        double x = magnitude*Math.cos(angle);
        double y = magnitude*Math.sin(angle);
        return new double[] {x, y};

    }

    double[] setmovement(double x, double y)
    {
        //Sets the motor values for directional movement.
        double motors[] = {0, 0, 0, 0, 0};
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

    /**
     * modifies motor values to add turns
     * @param motors the values to modify
     * @param x the amount to move forward
     * @param y the amount to move right
     * @param turningRate the amount to turn
     * @return the new motor values
     */
    double[] turning(double[] motors, double x, double y, double turningRate)
    {
        //Adjusts the motor values for turning.

        if (turningRate >= 0)
        {
            if (-x >= abs(y))
            {
                motors[0] = motors[0] - 2 * (y - (-abs(x))) * turningRate + (1 + x) * turningRate;
            }
            else if (y > abs(x))
            {
                motors[0] += (1 - y) * turningRate;
            }
            else if (x >= abs(y))
            {
                motors[0] += (1 - x) * turningRate;
            }
            else
            {
                motors[0] += (1 + y) * turningRate;
            }


            if (abs(x) <= y)
            {
                motors[1] = motors[1] + 2 * (y + abs(x)) * turningRate + (1 - y) * turningRate;
            }
            else if (-y >= abs(x))
            {
                motors[0] += (1 + y) * turningRate;
            }
            else if (x > abs(y))
            {
                motors[0] += (1 - x) * turningRate;
            }
            else
            {
                motors[0] += (1 + x) * turningRate;
            }


            if (x >= abs(y))
            {
                motors[2] = motors[2] + 2 * (abs(y) - x) * turningRate + (1 - x) * turningRate;
            }
            else if (y > abs(x))
            {
                motors[2] += (1 - y) * turningRate;
            }
            else if (-x >= abs(y))
            {
                motors[2] += (1 + x) * turningRate;
            }
            else
            {
                motors[3] += (1 - y) * turningRate;
            }

            if (abs(x) <= y)
            {
                motors[3] = motors[3] - 2 * (y - abs(x)) * turningRate + (1 - y) * turningRate;
            }
            else if (x > abs(y))
            {
                motors[3] += (1 - x) * turningRate;
            }
            else if (-y >= abs(x))
            {
                motors[3] += (1 + y) * turningRate;
            }
            else
            {
                motors[3] += (1 + x) * turningRate;
            }
        }
        else
        {
            if (-x >= abs(y))
            {
                motors[0] = motors[0] + 2 * (-abs(x) - x) * turningRate + (1 + x) * turningRate;
            }
            else if (-y > abs(x))
            {
                motors[0] += (1 + y) * turningRate;
            }
            else if (x >= abs(y))
            {
                motors[0] += (1 - x) * turningRate;
            }
            else
            {
                motors[0] += (1 - y) * turningRate;
            }

            if (y >= abs(x))
            {
                motors[1] = motors[1] - 2 * (-abs(y) + x) * turningRate + (1 - y) * turningRate;
            }
            else if (x > abs(y))
            {
                motors[1] += (1 - x) * turningRate;
            }
            else if (-y >= abs(x))
            {
                motors[1] += (1 + y) * turningRate;
            }
            else
            {
                motors[1] += (1 + x) * turningRate;
            }

            if (x >= abs(y))
            {
                motors[2] = motors[2] - 2 * (y - abs(x)) * turningRate + (1 - x) * turningRate;
            }
            else if (y > abs(x))
            {
                motors[2] += (1 - y) * turningRate;
            }
            else if (-x >= abs(y))
            {
                motors[2] += (1 + x) * turningRate;
            }
            else
            {
                motors[2] += (1 + x) * turningRate;
            }

            if (-x >= abs(y))
            {
                motors[3] = motors[3] - 2 * (-abs(y) - x) * turningRate + (1 + x) * turningRate;
            }
            else if (-y > abs(x))
            {
                motors[3] += (1 + y) * turningRate;
            }
            else if (x >= abs(y))
            {
                motors[3] += (1 - y) * turningRate;
            }
            else
            {
                motors[3] += (1 - x) * turningRate;
            }
        }

        return motors;
    }


    public void drive(double[] motors)
    {
        //Sets the power of the motors based on the motor values
        driveMotors[0].setPower(motors[0]);
        driveMotors[1].setPower(motors[1]);
        driveMotors[2].setPower(motors[2]);
        driveMotors[3].setPower(motors[3]);
    }

    private double n(double x)
    {
        return -abs(x);
    }

    public void actuator(Gamepad input1)
    {
        miscMotors[0].setPower(-input1.right_stick_y);
        /*
        //TODO: Get functional encoder wire to allow this part of the code to work
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
        float val = (float) actuatorController.pidCalculate(target, miscMotors[0]
        .getCurrentPosition());
        miscMotors[0].setPower(-val);*/
    }

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

    public void marker(Gamepad operator, boolean marker, Servo servos[])
    {
        //Controls the holder for the marker, used for testing.
        if (gamepad2.y)
        {
            marker = !marker;
        }

        if (marker)
        {
            servos[0].setPosition(.17);
            servos[1].setPosition(.15);
        }
        else
        {
            servos[0].setPosition(.85);
            servos[1].setPosition(.9);
        }
    }

    public void telemetry(boolean inverseControls, boolean actuator, double motors[],
                          Servo servos[], AndroidOrientation orientation)
    {
        //Creates and Displays Telemetry data for various functions on the Robot
        orientation.startListening();
        telemetry.addLine("Booleans")
                .addData("InvertControls", inverseControls)
                .addData("Actuator", actuator);
        telemetry.addLine("Motors")
                .addData("Motor0", motors[0])
                .addData("Motor1", motors[1])
                .addData("Motor2", motors[2])
                .addData("Motor3", motors[3]);
        telemetry.addLine("Misc Functions")
                .addData("Actuator Control", operator.right_stick_y)
                .addData("Orientation", orientation.getPitch());
        telemetry.update();
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
        miscMotors = new DcMotor[]{hardwareMap.get(DcMotor.class, "actuator")};
        driver = this.gamepad1;
        operator = this.gamepad2;
        //Sets up a system to hold the actuator in a position.
        actuatorController = new PID(0.01, 0, 0.00000, 999999,
                                     99999, 999999, 9999999);
        //Defines the servos in an array.
        servos = new Servo[]{hardwareMap.get(Servo.class, "marker1"),
                             hardwareMap.get(Servo.class, "marker2")};
        orientation = new AndroidOrientation();
        servos[0].setDirection(REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            //Main code for the robot, the stuff that actually does stuff.
            drivemode(driver);
            double[] direction = orient(driver);
            double motors[] = setmovement(direction[0], direction[1]);
            double turn = driver.right_trigger-driver.left_trigger;
            turning(motors, direction[0], direction[1], turn);
            drive(motors);
            actuator(operator);
            marker(operator, marker, servos);
            telemetry(inverseControls, actuator, motors, servos, orientation);
        }
    }
}



