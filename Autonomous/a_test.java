package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;

import java.nio.file.FileAlreadyExistsException;

@Autonomous
public class a_test extends OpMode
{
    GoBildaPinpointDriver odometry;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorSlideLeft, motorSlideRight;
    DcMotorEx motorClawArm;
    Servo servoClawGrabber, servoClawExtend;
    VoltageSensor sensorVoltage;
    Robot bot;
    TouchSensor buttonArmStop;


    double navXTarget = 0.0;
    double navYTarget = 0.0;
    double navOrientationTarget = -1.0;  // -1.0 means no rotation
    boolean navPrecise = true;
    boolean navOnlyRotate = false;
    boolean navEngaged = false;
    double positionXCurrent = 0.0;
    double positionYCurrent = 0.0;
    double distanceToPositionTarget = 0.0;
    double orientationCurrent = 0.0; // in degrees - value will be 0 - 359.99
    double distanceToOrientationTarget = 0.0;
    String rotationDirection = "none";
    double[] rotateReturn = new double[2];
    double orientationTarget = -1.0;  // -1.0 means no rotation
    double distanceToTargetOrientation = 0.0;
    double rotationErrorTolerance = 1.0;
    double[] navReturn = new double[3];
    double slideTarget = -1.0;
    double armTarget = -10000;
    String armDirection = "up";
    //double armCurrent = 0.0;

    boolean driveButtonA, driveButtonX, driveButtonB, driveButtonY;
    boolean driveDpadL, driveDpadU, driveDpadD, driveDpadR;
    float stickLeftX, stickLeftY, stickRightX, stickRightY;
    float triggerLeft, triggerRight;

    double armExtendedPosition = 0.42;//a
    double armRetractedPosition = 0.0;//x -0.55

    double clawOpenPosition = 0.7;//left bumper
    double clawClosedPosition = 0.1;//right bumper

    ElapsedTime timer = new ElapsedTime();

    String step = "None";

    public void showTelemetry()
    {
        bot.update();
        //telemetry.addData("degrees", bot.getDeg());
        telemetry.addLine("loop");
        telemetry.addData("x target", navXTarget);
        telemetry.addData("x position", bot.getX());
        telemetry.addData("y target", navYTarget);
        telemetry.addData("y position", bot.getY());
        telemetry.addData("orientation target", navOrientationTarget);
        telemetry.addData("orientation", bot.getOrientationCurrent());
        telemetry.addData("test", bot.test);
        telemetry.addData("arm target", armTarget);
        telemetry.addData("arm current", motorClawArm.getCurrentPosition());
        telemetry.addData("arm power", motorClawArm.getPower());
        telemetry.addData("slide power", motorSlideLeft.getPower());
        telemetry.addData("slide position", motorSlideLeft.getCurrentPosition());
        telemetry.addData("slide target", slideTarget);
        telemetry.addData("step", step);


        telemetry.addData("FL power", motorFrontLeft.getPower());
        telemetry.addData("FR power", motorFrontRight.getPower());
        telemetry.addData("BL power", motorBackLeft.getPower());
        telemetry.addData("BR power", motorBackRight.getPower());

        telemetry.update();
    }

    public void init()
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

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorSlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        motorSlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");

        motorSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

        sensorVoltage = hardwareMap.get(VoltageSensor.class, "Control Hub");

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        odometry.setOffsets(-90, 120); // don't know how to get these offsets yet
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

        //armTarget = 750;//2800
        //armTarget = -1;
        servoClawExtend.setPosition(armRetractedPosition);
        servoClawGrabber.setPosition(clawOpenPosition);

        buttonArmStop = hardwareMap.get(TouchSensor.class, "Button");
        bot.setButton(buttonArmStop);
        timer.reset();

        bot.voltageStart = sensorVoltage.getVoltage();
    }

    public void init_loop()
    {
        //armTarget = bot.armToPosition(armTarget, "up");
        if(timer.milliseconds() >= 2000)
        {
            //servoClawGrabber.setPosition(clawClosedPosition);
        }
    }

    public void start()
    {
        telemetry.addLine("start");
        telemetry.update();
        step = "start";
    }

    public void autonomous()
    {
        if(step.equalsIgnoreCase("start"))
        {
            navEngaged = true;
            //navXTarget = -500;
            //navYTarget = 100;
            navOrientationTarget = 45;
            navOnlyRotate = true;
            //navPrecise = false;
            step = "test 1";
        }
        else if (step.equalsIgnoreCase("test 1"))
        {
            if(!navEngaged)
            {
                timer.reset();
                step = "wait 1";
            }
        }
        else if (step.equalsIgnoreCase("wait 1"))
        {
            if(timer.milliseconds() > 0)
            {
                navEngaged = true;
                navXTarget = -1000;
                navYTarget = 400;
                navOrientationTarget = 0;
                navOnlyRotate = false;
                navPrecise = true;
                bot.distanceToTargetPrevious = 10000;
                step = "test 2";
            }
        }
        else if (step.equalsIgnoreCase("test 2"))
        {
            if(!navEngaged)
            {
                timer.reset();
                servoClawExtend.setPosition(armExtendedPosition);
                step = "wait 2";
            }
        }
        else if (step.equalsIgnoreCase("wait 2"))
        {
            if(timer.milliseconds() > 300)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
                if(timer.milliseconds() > 500)
                {
                    navEngaged = true;
                    navXTarget = -1300;
                    navYTarget = 345;
                    navOrientationTarget = 35;
                    navOnlyRotate = false;
                    navPrecise = true;
                    bot.distanceToTargetPrevious = 10000;
                    servoClawExtend.setPosition(armRetractedPosition);
                    //armTarget = 1000;//1500
                    //armDirection = "up";
                    //slideTarget = 2200;
                    step = "test 3";
                }
            }
        }
        else if (step.equalsIgnoreCase("test 3"))
        {
            if(!navEngaged)
            {
                step = "end";
            }
        }
    }

    public void loop()
    {
        showTelemetry();

        rotateReturn = bot.navRotate(orientationTarget, rotationDirection, rotationErrorTolerance);
        distanceToTargetOrientation = rotateReturn[0];
        orientationTarget = rotateReturn[1];

        //navReturn = bot.navToPosition(navXTarget, navYTarget, navPrecise, navEngaged);
        navReturn = bot.navToPositionAndRotate(navXTarget, navYTarget, navOrientationTarget, navOnlyRotate, navPrecise, navEngaged);
        if(navReturn[0] < 0) // turn off navToPosition function
        {
            navEngaged = false;
        }

        slideTarget = bot.slideToPosition(slideTarget);
        armTarget = bot.armToPosition(armTarget, armDirection);

        autonomous();

    }

}
