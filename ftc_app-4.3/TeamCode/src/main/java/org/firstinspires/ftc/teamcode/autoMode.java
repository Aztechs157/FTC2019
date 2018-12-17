package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

/**
 * this is the first autonomous written for the 2018-2019 robot. this autonomous route is currently
 * untested, which will be fixed shortly. it lands, claims the depot, then goes into the crater.
 */
@Autonomous
public class autoMode extends LinearOpMode
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

    /**
     * function that was coppied from tele-op, not entirely necessary here. it switches the drive
     * mode.
     * @param input:the controller that is being used
     */
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

    /**
     * simply gets the absolute value of a number, is made so i don't have to write Math.abs every
     * time
     * @param x: the value to find the absolute value of
     * @return: the absolute value of x
     */
    private double abs(double x)
    {
        return Math.abs(x);
    }

    /**
     * takes in a controller input, and converts it to the 
     * @param x
     * @param y
     * @return
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
        //Creates Telemetry data for the motors.
        driveMotors[0].setPower(motors[0]);
        driveMotors[1].setPower(motors[1]);
        driveMotors[2].setPower(motors[2]);
        driveMotors[3].setPower(motors[3]);
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

    /**
     * Used to control the Intake.
     * This has not been tested as the intake doesn't work mechanically.
     * @param input
     * @param miscMotors
     */
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
        miscMotors = new DcMotor[]{hardwareMap.get(DcMotor.class, "actuator")};
        driver = this.gamepad1;
        operator = this.gamepad2;
        //Sets up a system to hold the actuator in a position.
        actuatorController = new PID(0.01, 0, 0.00000, 999999,
                                     99999, 999999, 9999999);
        //Defines the servos in an array.
        servos = new Servo[]{hardwareMap.get(Servo.class, "marker1"),
                             hardwareMap.get(Servo.class, "marker2")};
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        servos[0].setDirection(REVERSE);
        servos[0].setPosition(.85);
        servos[1].setPosition(.9);
        waitForStart();

        ElapsedTime time = new ElapsedTime();

        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < 8650)
        {
            miscMotors[0].setPower(1);
        }
        miscMotors[0].setPower(0);
        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < 100)
        {
            miscMotors[0].setPower(0);
            drive(setmovement(0, -1));
        }
        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < 1300)
        {
            miscMotors[0].setPower(-1);
            drive(setmovement(1, -0.1));
        }
        drive(new double[]{0, 0, 0, 0});
        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < 130)
        {
            drive(turning(setmovement(0, 0), 0, 0, 1));
            //miscMotors[0].setPower(-1);
        }
        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < 500)
        {
            drive(setmovement(0, 0));
            //miscMotors[0].setPower(-1);
            servos[0].setPosition(.17);
            servos[1].setPosition(.15);
        }
        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < 2250)
        {
            miscMotors[0].setPower(0);
            drive(setmovement(0, 1));
        }
        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < 100)
        {
            drive(turning(setmovement(0, 0), 0, 0, -1));
            miscMotors[0].setPower(0);
        }
        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < 1000)
        {
            miscMotors[0].setPower(0);
            drive(setmovement(0, 1));
        }


    }
}