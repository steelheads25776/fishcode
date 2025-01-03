package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class RobotTest extends LinearOpMode
{
    GoBildaPinpointDriver odometry;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorSlideLeft, motorSlideRight;
    DcMotorEx motorClawArm;
    Servo servoClawGrabber, servoClawExtend;
    Robot bot;

    double orientationCurrent = 0.0; // in degrees - value will be 0 - 359.99
    double orientationTarget = -1.0;  // -1.0 means no rotation
    double distanceToTargetOrientation = 0.0;
    double positionOrientationTarget = 0.0;
    double rotationErrorTolerance = 1.0;
    double positionXTarget = -10000.0;
    double positionYTarget = -10000.0;
    double distanceToTargetPosition = 0.0;
    double[] rotateReturn = new double[2];
    double[] positionReturn = new double[2];
    double slideTarget = -1.0;
    double armTarget = -1.0;

    boolean driveButtonA, driveButtonX, driveButtonB, driveButtonY;
    boolean driveDpadL, driveDpadU, driveDpadD, driveDpadR;
    float stickLeftX, stickLeftY, stickRightX, stickRightY;
    float triggerLeft, triggerRight;

    double motorspeed = 1.0;
    double rotateSpeed = 0.75;
    double slowRotateSpeed = 0.55;
    double motorspeedhigh = 1.0;
    double motorspeednormal = 0.5;
    double motorspeedslower = 0.25;
    String rotateMode = "slow";

    double armExtendedPosition = 0.42;//a
    double armRetractedPosition = 0.0;//x -0.55

    double clawOpenPosition = 0.7;//left bumper
    double clawClosedPosition = 0.1;//right bumper

    public void initialize()
    {
        // setting hardware to variables
        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorSlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        motorSlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");

        motorSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorClawArm = hardwareMap.get(DcMotorEx.class, "Arm");

        motorClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorClawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorClawArm.setDirection(DcMotorSimple.Direction.FORWARD);

        servoClawGrabber =  hardwareMap.get(Servo.class, "Grabber");
        servoClawExtend = hardwareMap.get(Servo.class, "Extender");

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        odometry.setOffsets(-124, -150); // don't know how to get these offsets yet
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.resetPosAndIMU();

        bot = new Robot(odometry);
        //bot.setDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //bot.setMotors(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

        //bot = new Robot();
        bot.setMotors(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorSlideLeft, motorSlideRight, motorClawArm);
        bot.setServos(servoClawGrabber, servoClawExtend);
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
    public void driverDPad()
    {
        driveDpadL = gamepad1.dpad_left;
        driveDpadR = gamepad1.dpad_right;
        driveDpadU = gamepad1.dpad_up;
        driveDpadD = gamepad1.dpad_down;

        /*if(driveDpadL)
        {
            orientationTarget = 270;
        }
        else if(driveDpadU)
        {
            orientationTarget = 0;
        }
        else if(driveDpadR)
        {
            orientationTarget = 90;
        }
        else if(driveDpadD)
        {
            orientationTarget = 180;
        }*/
        if(driveDpadL)
        {
            servoClawGrabber.setPosition(clawOpenPosition);
            positionXTarget = 700;
            positionYTarget = 0;
            positionOrientationTarget = 0;
        }
        else if(driveDpadU)
        {
            //orientationTarget = 0;
            //positionXTarget = 0;
            //positionYTarget = 1000;
            //positionOrientationTarget = bot.getOrientationCurrent();
            //slideTarget = 0;
            armTarget = 2500;
            servoClawExtend.setPosition(armRetractedPosition);
            servoClawGrabber.setPosition(clawOpenPosition);
        }
        else if(driveDpadR)
        {
            servoClawGrabber.setPosition(clawClosedPosition);
            orientationTarget = 0;
            positionXTarget = 0;
            positionYTarget = 700;
            positionOrientationTarget = 0;
            slideTarget = 930;
        }
        else if(driveDpadD)
        {
            //orientationTarget = 0;
            //positionXTarget = 0;
            //positionYTarget = 0;
            //positionOrientationTarget = bot.getOrientationCurrent();
            //slideTarget = 0;
            armTarget = 1500;
        }
    }

    //public void rotatePad()
    //{
    //    driveDpadL = gamepad1.dpad_left;
    //    driveDpadR = gamepad1.dpad_right;
    //    driveDpadU = gamepad1.dpad_up;
    //    driveDpadD = gamepad1.dpad_down;

    //    if(driveDpadL)
    //    {
    //        bot.rotate(270);
    //        bot.sleep(1000);
    //    }

    //    if(driveDpadU)
    //    {
    //        bot.rotate(0);
    //        bot.sleep(1000);
    //    }

    //    if(driveDpadR)
    //    {
    //        bot.rotate(90);
    //        bot.sleep(1000);
    //    }

    //    if(driveDpadD)
    //    {
    //        bot.rotate(180);
    //        bot.sleep(1000);
    //    }
    //}

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
            bot.forward(1000);
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
        telemetry.addData("current target", orientationTarget);
        telemetry.addData("target slide", slideTarget);
        telemetry.addData("current slide", motorSlideLeft.getCurrentPosition());
        telemetry.addData("test", bot.test);

        telemetry.addData("current orientation", bot.getOrientationCurrent());
        //telemetry.addData("current radians", bot.getRad());
        telemetry.addData("distance to target rotation", distanceToTargetOrientation);
        telemetry.addData("distance to target position", distanceToTargetPosition);
        telemetry.addData("stopped position", bot.YStoppedPosition);
        telemetry.addData("armSlide target", slideTarget);
        telemetry.addData("slide position", motorSlideLeft.getCurrentPosition());
        telemetry.addData("clawArm target", armTarget);
        telemetry.addData("clawArm position", motorClawArm.getCurrentPosition());
        telemetry.addData("arm position", bot.test);
        telemetry.update();
    }

    public void runOpMode()
    {
        initialize();
        while(opModeIsActive())
        {
            drive();
            rotateReturn = bot.navRotate(orientationTarget, "none", rotationErrorTolerance);
            distanceToTargetOrientation = rotateReturn[0];
            orientationTarget = rotateReturn[1];

            if(orientationTarget < 0)
            {
                positionReturn = bot.navToPosition(positionXTarget, positionYTarget, positionOrientationTarget, true);
                distanceToTargetPosition = positionReturn[0];
                positionXTarget = positionReturn[1];
            }

            //slideTarget = bot.slideToPosition(slideTarget);
            armTarget = bot.armToPosition(armTarget, false);

            driverDPad();
            showTelemetry();

            forward();
            backward();
            left();
            right();

            //rotatePad();
            //resetRobot();
        }
    }
}
