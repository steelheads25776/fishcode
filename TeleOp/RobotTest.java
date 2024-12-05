package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class RobotTest extends LinearOpMode
{
    GoBildaPinpointDriver odometry;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    Robot bot;

    boolean driveButtonA, driveButtonX, driveButtonB, driveButtonY;
    boolean driveDpadL, driveDpadU, driveDpadD, driveDpadR;
    float stickLeftX, stickLeftY, stickRightX, stickRightY;
    float triggerLeft, triggerRight;

    double motorspeed = 1.0;
    double rotateSpeed = 0.75;
    double slowRotateSpeed = 0.35;
    double motorspeedhigh = 1.0;
    double motorspeednormal = 0.5;
    double motorspeedslower = 0.25;
    String rotateMode = "slow";

    public void initialize()
    {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");

        bot = new Robot(odometry);
        bot.setDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        bot.setMotors(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);
        bot.setSpeed(0.4);
        bot.reset();

        waitForStart();
    }

    public void drive()
    {

        stickLeftX = gamepad1.left_stick_x;
        stickLeftY = gamepad1.left_stick_y;
        stickRightX = gamepad1.right_stick_x;
        stickRightY = gamepad1.right_stick_y;

        triggerLeft = gamepad1.left_trigger;
        triggerRight = gamepad1.right_trigger;

        if(stickLeftX == 0.0 && stickLeftY == 0.0)
        {
            rotateMode = "slow";
            motorFrontLeft.setPower((stickRightX * slowRotateSpeed) * -1);
            motorFrontRight.setPower((stickRightX * slowRotateSpeed) );
            motorBackLeft.setPower((stickRightX * slowRotateSpeed) * -1);
            motorBackRight.setPower((stickRightX * slowRotateSpeed));
        }


        else
        {
            rotateMode = "fast";
            motorFrontLeft.setPower((stickLeftY - stickLeftX - (stickRightX * rotateSpeed)) * motorspeed);
            motorFrontRight.setPower((stickLeftY + stickLeftX + (stickRightX* rotateSpeed)) * motorspeed);
            motorBackLeft.setPower((stickLeftY + stickLeftX - (stickRightX* rotateSpeed)) * motorspeed);
            motorBackRight.setPower((stickLeftY - stickLeftX + (stickRightX * rotateSpeed)) * motorspeed);
        }

        if(triggerLeft >= 0.1)
        {
            motorspeed = motorspeedhigh;
        }

        else if(triggerRight >= 0.1)
        {
            motorspeed = motorspeedslower;
        }

        else
        {
            motorspeed = motorspeednormal;
        }
    }

    public void rotatePad()
    {
        driveDpadL = gamepad1.dpad_left;
        driveDpadR = gamepad1.dpad_right;
        driveDpadU = gamepad1.dpad_up;
        driveDpadD = gamepad1.dpad_down;

        if(driveDpadL)
        {
            bot.rotate(270);
            bot.sleep(1000);
        }

        if(driveDpadU)
        {
            bot.rotate(0);
            bot.sleep(1000);
        }

        if(driveDpadR)
        {
            bot.rotate(90);
            bot.sleep(1000);
        }

        if(driveDpadD)
        {
            bot.rotate(180);
            bot.sleep(1000);
        }
    }

    public void resetRobot()
    {
        driveDpadD = gamepad1.dpad_down;

        if(driveDpadD)
        {
            bot.reset();
            bot.sleep(1000);
        }
    }

    public void forward()
    {
        driveButtonY = gamepad1.y;

        if(driveButtonY)
        {
            bot.forward(100);
            bot.sleep(1000);
        }
    }
    public void backward()
    {
        driveButtonA = gamepad1.a;

        if(driveButtonA)
        {
            bot.backward(100);
            bot.sleep(1000);
        }
    }
    public void left()
    {
        driveButtonX = gamepad1.x;

        if(driveButtonX)
        {
            bot.left(100);
            bot.sleep(1000);
        }
    }
    public void right()
    {
        driveButtonB = gamepad1.b;

        if(driveButtonB)
        {
            bot.right(100);
            bot.sleep(1000);
        }
    }

    public void showTelemetry()
    {
        bot.update();
        telemetry.addData("current x", bot.getX());
        telemetry.addData("current y", bot.getY());
        telemetry.addData("current target", bot.getTargetPos());
        telemetry.addData("current degrees", bot.getDegrees());
        telemetry.addData("current radians", bot.getRad());
        telemetry.addData("distance to target", bot.targetDistance);
        telemetry.addData("spin direction", bot.spinDirection);
        telemetry.update();
    }


    public void runOpMode()
    {
        initialize();
        while(opModeIsActive())
        {
            forward();
            backward();
            drive();
            left();
            right();
            rotatePad();
            //resetRobot();
            showTelemetry();
        }
    }
}
