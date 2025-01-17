package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;

import java.nio.file.FileAlreadyExistsException;

@Autonomous
public class a_1hang3bucket extends OpMode
{
    GoBildaPinpointDriver odometry;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorSlideLeft, motorSlideRight;
    DcMotorEx motorClawArm;
    Servo servoClawGrabber, servoClawExtend;
    VoltageSensor sensorVoltage;
    Robot bot;

    double orientationCurrent = 0.0; // in degrees - value will be 0 - 359.99
    double orientationTarget = -1.0;  // -1.0 means no rotation
    double distanceToTargetOrientation = 0.0;
    double positionOrientationTarget = 0.0;
    String rotationDirection = "none";
    double rotationErrorTolerance = 1.0;
    double positionXTarget = -10000.0;
    double positionYTarget = -10000.0;
    boolean positionPrecise = true;
    double distanceToTargetPosition = 0.0;
    double[] rotateReturn = new double[2];
    double[] positionReturn = new double[2];
    double slideTarget = -1.0;
    double armTarget = -10000;
    //double armCurrent = 0.0;

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

    double armExtendedPosition = 0.42;//a
    double armRetractedPosition = 0.0;//x -0.55

    double clawOpenPosition = 0.7;//left bumper
    double clawClosedPosition = 0.1;//right bumper

    ElapsedTime timer = new ElapsedTime();

    String step = "None";

    public void showTelemetry()
    {
        //telemetry.addData("degrees", bot.getDeg());
        telemetry.addLine("loop");
        telemetry.addData("x target", positionXTarget);
        telemetry.addData("x pos", positionReturn[0]);
        telemetry.addData("y pos", positionReturn[1]);
        telemetry.addData("orientation target", orientationTarget);
        telemetry.addData("orientation distance to target", rotateReturn[0]);
        telemetry.addData("arm target", armTarget);
        telemetry.addData("arm current", motorClawArm.getCurrentPosition());
        telemetry.addData("slide power", motorSlideLeft.getPower());
        telemetry.addData("slide position", motorSlideLeft.getCurrentPosition());
        telemetry.addData("slide target", slideTarget);
        telemetry.addData("test", bot.test);


        telemetry.addData("FL power", motorFrontLeft.getPower());
        telemetry.addData("FR power", motorFrontRight.getPower());
        telemetry.addData("BL power", motorBackLeft.getPower());
        telemetry.addData("BR power", motorBackRight.getPower());
        //telemetry.addData("step1", steps[0]);
        //telemetry.addData("step2", steps[1]);
        //telemetry.addData("step3", steps[2]);
        //telemetry.addData("step4", steps[3]);
        //telemetry.addData("step5", steps[4]);

        bot.update();
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

        armTarget = 750;//2800
        //armTarget = -1;
        servoClawExtend.setPosition(armRetractedPosition);
        servoClawGrabber.setPosition(clawOpenPosition);
        timer.reset();

        bot.voltageStart = sensorVoltage.getVoltage();
    }

    public void init_loop()
    {
        armTarget = bot.armToPosition(armTarget, false);
        if(timer.milliseconds() >= 2000)
        {
            servoClawGrabber.setPosition(clawClosedPosition);
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
            //servoClawGrabber.setPosition(clawClosedPosition);
            orientationTarget = 0;
            positionXTarget = 0;
            positionYTarget = 475;
            positionOrientationTarget = 0;
            bot.distanceToTargetPrevious = 1000;
            positionPrecise = true;
            //armTarget = 360;
            //servoClawExtend.setPosition(armExtendedPosition);
            if(positionXTarget < -9999 && armTarget < -9999)
            {
                step = "move to hang s1 position";
            }
        }
        else if(step.equalsIgnoreCase("move to hang s1 position"))
        {
            //servoClawExtend.setPosition(armExtendedPosition);
            //slideTarget = 860;
            //armTarget = 420;

            if(slideTarget < 0)
            {
                timer.reset();
                step = "lock s1 to bar";
            }
        }
        else if(step.equalsIgnoreCase("lock s1 to bar"))
        {
            orientationTarget = 0;
            positionXTarget = 0;
            positionYTarget = 360;
            positionOrientationTarget = 0;
            bot.distanceToTargetPrevious = 1000;


            if(timer.milliseconds() > 500)
            {
                //servoClawGrabber.setPosition(clawOpenPosition);
                if(positionXTarget < -9999)
                {
                    //servoClawExtend.setPosition(armRetractedPosition);
                    step = "moveinaccurate";
                }
            }

        }
        else if(step.equalsIgnoreCase("moveinaccurate"))
        {
            orientationTarget = 0;
            positionYTarget = 245;
            positionXTarget = -1000;
            positionOrientationTarget = 0;
            bot.distanceToTargetPrevious = 1000;
            //slideTarget = 0;
            //armTarget = 0;
            positionPrecise = true;

            step = "movetosamples";
        }
        else if(step.equalsIgnoreCase("movetosamples") && positionXTarget <= -9999 && slideTarget <= -9999)
        {
            orientationTarget = 0;
            positionYTarget = 730;
            positionXTarget = -1400;
            positionOrientationTarget = 0;
            bot.distanceToTargetPrevious = 1000;
            //slideTarget = 0;
            //armTarget = 0;
            positionPrecise = true;

            step = "end"; //sample in front of robot
        }
    }

    public void loop()
    {
        showTelemetry();

        rotateReturn = bot.navRotate(orientationTarget, rotationDirection, rotationErrorTolerance);
        distanceToTargetOrientation = rotateReturn[0];
        orientationTarget = rotateReturn[1];

        if(orientationTarget < 0)
        {
            positionReturn = bot.navToPosition(positionXTarget, positionYTarget, positionOrientationTarget, positionPrecise);
            distanceToTargetPosition = positionReturn[0];
            positionXTarget = positionReturn[1];
        }

        slideTarget = bot.slideToPosition(slideTarget);
        armTarget = bot.armToPosition(armTarget, false);

        autonomous();

    }

}

