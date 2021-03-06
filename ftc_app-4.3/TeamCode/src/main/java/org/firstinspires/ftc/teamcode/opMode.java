/**
 *  This is the Tele-Op code for the 2018-2019 ftc robot for team 12566. This code was a joint effort
 * between team members Jeremy Patrick and Tyler Silva. This code was written over the course a few
 * weeks. It was written in the Java programming language. We used Github for version control and
 * collaboration. We wrote the design for the code in Notepad++.
 *  We still have plenty of room to improve the code, for both functionality and elegance. We can
 * expand commenting in order to improve the readability of the code. We can also add more
 * functionality to the telemetry section in order to make it more useful.
 */


/**
 * Program: OpMode
 *
 * Definition: This program is a dice rolling game that can run with multiple players and win conditions
 *
 * Author: Jeremy Patrick and Tyler Silva
 *
 * Date: 12/6/18
 *
 * History: 1/16/2019: File Creation
 */

//********************************    IMPORTS   ***********************************
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

@TeleOp
public class opMode extends LinearOpMode
{
//******************************    VARIABLES   **********************************
    private DcMotor driveMotors[] = {null, null, null, null};
    private DcMotor miscMotors[] = {null, null, null};

    private boolean inverseControls = false;
    private boolean intakeOut = false;
    private Gamepad driver = null;
    private Gamepad operator = null;
    private boolean actuator = false;
    private PID actuatorController;
    private double motors[] = {0, 0, 0, 0, 0};
    private Servo servos[] = {null, null};
    private AnalogGyro orientation;
    private boolean marker = true;

//===========================  BEGIN CODE ==============================

    /**
     * Sets the drive mode, determines which stick is used to drive.
      * @param input Gamepad input data
     */
    void drivemode(Gamepad input)
    {
        if (input.back)
        {
            inverseControls = !inverseControls;
        }
    }

    /**
     * @param x A number being inputted.
     * @return The absolute value of x.
     */
    private double abs(double x)
    {
        return Math.abs(x);
    }

    /**
     *
     * @param input
     * @return
     */
    private double[] orient(Gamepad input)
    {
        double angle = Math.atan2(input.right_stick_x, input.right_stick_y);
        angle = -angle+orientation.angle;
        double magnitude = Math.sqrt(Math.pow(input.right_stick_x, 2)+Math.pow(input.right_stick_y, 2));
        double x = magnitude*Math.cos(angle);
        double y = magnitude*Math.sin(angle);
        return new double[] {x, y};

    }

    /**
     * For more detail, see Math Docs.
     * @param x The x value of the joystick
     * @param y The y value of the joystick
     * @return The values of the motors, before turning
     */
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

    /**
     * Sets the power of the motors based on the motor values.
     * @param motors The motor values based on the above math.
     */
    public void drive(double[] motors)
    {
        driveMotors[0].setPower(motors[0]);
        driveMotors[1].setPower(motors[1]);
        driveMotors[2].setPower(motors[2]);
        driveMotors[3].setPower(motors[3]);
    }

    /**
     * @param x The number to be operated on.
     * @return The inverse of the absolute value of x.
     */
    private double n(double x)
    {
        return -abs(x);
    }

    /**
     * Controls the movement of the actuator.
     * @param input the input from the controller.
     */
    public void actuator(Gamepad input)
    {
        miscMotors[0].setPower(-input.right_stick_y);
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

    /**
     * Controls the intake mechanism.
     * @param input Handles the input from the controller.
     * @param miscMotors
     */
    public void intake(Gamepad input, DcMotor miscMotors[])
    {
        //Controls the speed and direction of the intake.
        if (input.left_trigger > 0)
        {
            //reverse
            miscMotors[1].setPower(-input.left_trigger);
            miscMotors[2].setPower(input.left_trigger);
        }
        else if (input.right_trigger > 0)
        {
            //forward
            miscMotors[1].setPower(input.right_trigger);
            miscMotors[2].setPower(-input.right_trigger);
        }
        else
        {
            miscMotors[1].setPower(0);
            miscMotors[2].setPower(0);
        }
    }

    /**
     * Controls the the holder for the marker.
     * @param operator The second gamepad
     * @param marker Whether or not the marker holder is open
     * @param servos Controls the servo for the marker holder
     */

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

    /**
     * Sets the telemetry
     * @param inverseControls Whether the controls are inverted
     * @param actuator Whether the actuator is up or down
     * @param motors Motor values
     * @param servos Servo values
     * @param orientation Gyro orientation
     */

    public void telemetry(boolean inverseControls, boolean actuator, double motors[],
                          Servo servos[], AnalogGyro orientation)
    {
        //Creates and Displays Telemetry data for various functions on the Robot

        telemetry.addLine("Booleans")
                .addData("InvertControls", inverseControls)
                .addData("Actuator", actuator);
        telemetry.addLine("Motors")
                .addData("Motor0", motors[0])
                .addData("Motor1", motors[1])
                .addData("Motor2", motors[2])
                .addData("Motor3", motors[3]);
                //.addData("Intake1", motors[]);
        telemetry.addLine("Misc Functions")
                .addData("Actuator Control", operator.right_stick_y)
                .addData("Orientation", orientation.angle);
        telemetry.update();
    }

    /**
     * Main op mode, runs each of the the functions and procedures and defines some stuff.
     */

    @Override
    public void runOpMode()
    {
        //Defines the drive motors
        driveMotors = new DcMotor[]{hardwareMap.get(DcMotor.class, "drive1"),
                                    hardwareMap.get(DcMotor.class, "drive2"),
                                    hardwareMap.get(DcMotor.class, "drive3"),
                                    hardwareMap.get(DcMotor.class, "drive4")};
        //defines the gamepads
        miscMotors = new DcMotor[]{hardwareMap.get(DcMotor.class, "actuator"),
                                   hardwareMap.get(DcMotor.class, "intake1"),
                                   hardwareMap.get(DcMotor.class, "intake2")};
        driver = this.gamepad1;
        operator = this.gamepad2;
        //Sets up a system to hold the actuator in a position.
        actuatorController = new PID(0.01, 0, 0.00000, 999999,
                                     99999, 999999, 9999999);
        //Defines the servos in an array.
        servos = new Servo[]{hardwareMap.get(Servo.class, "marker1"),
                             hardwareMap.get(Servo.class, "marker2")};
        orientation = new AnalogGyro(hardwareMap.get(AnalogInput.class, "gyro"));
        servos[0].setDirection(REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        orientation.start();
        while (opModeIsActive())
        {
            //Main code for the robot, the stuff that actually does stuff.
            orientation.update();
            drivemode(driver);
            double[] direction = orient(driver);
            double motors[] = setmovement(driver.right_stick_y, -driver.right_stick_x);
            double turn = driver.right_trigger-driver.left_trigger;
            turning(motors, direction[0], direction[1], turn);
            drive(motors);
            actuator(operator);
            marker(operator, marker, servos);
            intake(operator, miscMotors);
            telemetry(inverseControls, actuator, motors, servos, orientation);
        }
    }
}