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
public class a_3hang extends OpMode
{
    GoBildaPinpointDriver odometry;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorSlideLeft, motorSlideRight;
    DcMotorEx motorClawArm;
    Servo servoClawGrabber, servoClawExtend;
    VoltageSensor sensorVoltage;
    Robot bot;
    TouchSensor buttonArmStop;

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
    String armDirection = "up";
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
        bot.update();
        //telemetry.addData("degrees", bot.getDeg());
        telemetry.addLine("loop");
        telemetry.addData("x target", positionXTarget);
        telemetry.addData("y target", positionYTarget);
        telemetry.addData("distance to target", positionReturn[0]);
        telemetry.addData("x pos return", positionReturn[1]);
        telemetry.addData("x pos", bot.getX());
        telemetry.addData("y pos", bot.getY());
        telemetry.addData("orientation target", orientationTarget);
        telemetry.addData("orientation distance to target", rotateReturn[0]);
        telemetry.addData("arm target", armTarget);
        telemetry.addData("arm current", motorClawArm.getCurrentPosition());
        telemetry.addData("arm power", motorClawArm.getPower());
        telemetry.addData("slide power", motorSlideLeft.getPower());
        telemetry.addData("slide position", motorSlideLeft.getCurrentPosition());
        telemetry.addData("slide target", slideTarget);
        telemetry.addData("step", step);
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
        odometry.setOffsets(-124, -500); // don't know how to get these offsets yet
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

        buttonArmStop = hardwareMap.get(TouchSensor.class, "Button");
        bot.setButton(buttonArmStop);
        timer.reset();

        bot.voltageStart = sensorVoltage.getVoltage();
    }

    public void init_loop()
    {
        armTarget = bot.armToPosition(armTarget, "up");
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
            servoClawGrabber.setPosition(clawClosedPosition);
            positionYTarget = 515;//475
            bot.distanceToTargetPrevious = 1000;
            step = "start started";
        }
        else if (step.equalsIgnoreCase("start started"))
        {
            if(positionYTarget < -9999)
            {
                step = "move to hang s1 position";
            }
        }
        else if(step.equalsIgnoreCase("move to hang s1 position"))
        {
            servoClawExtend.setPosition(armExtendedPosition);
            slideTarget = 975;// 965
            armTarget = 390;// 420
            armDirection = "down";
            step = "move to hang s1 position started";
        }
        else if(step.equalsIgnoreCase("move to hang s1 position started"))
        {
            if(slideTarget < 0)
            {
                timer.reset();
                step = "lock s1 to bar";
            }
        }
        else if(step.equalsIgnoreCase("lock s1 to bar"))
        {
            positionYTarget = 360; //360
            bot.distanceToTargetPrevious = 10000;
            timer.reset();
            step = "lock s1 to bar started";
        }
        else if(step.equalsIgnoreCase("lock s1 to bar started"))
        {
            armTarget = 425;//415
            armDirection = "up";
            if(timer.milliseconds() > 800)
            {
                servoClawGrabber.setPosition(clawOpenPosition);
                if(positionYTarget < -9999)
                {
                    step = "get square";
                }
            }
            else if(timer.milliseconds() > 300)
            {
                servoClawExtend.setPosition(armRetractedPosition);
            }
        }
        else if(step.equalsIgnoreCase("get square"))
        {
            orientationTarget = 0;
            slideTarget = 0;
            armTarget = 30;
            armDirection = "down";
            step = "get square started";
        }
        else if(step.equalsIgnoreCase("get square started"))
        {
            if (slideTarget < 0 && armTarget < -9999 && orientationTarget < 0)
            {
                step = "move to sample 1";
            }
        }
        else if(step.equalsIgnoreCase("move to sample 1"))
        {
            armTarget = 50;
            armDirection = "down";
            slideTarget = 0;
            positionXTarget = 1270;
            timer.reset();
            step = "reset arm s1";
        }
        else if(step.equalsIgnoreCase("reset arm s1"))
        {
            if(positionXTarget < -9999 && armTarget < -9999 && (slideTarget < 0 || timer.milliseconds() > 2000))
            {
                bot.resetSlideAndArm();
                step = "get square 2";
            }
        }
        else if(step.equalsIgnoreCase("get square 2"))
        {
            orientationTarget = 0;
            step = "get square started 2";
        }
        else if(step.equalsIgnoreCase("get square started 2"))
        {
            if (orientationTarget < 0)
            {
                step = "move forward a tad";
            }
        }
        else if(step.equalsIgnoreCase("move forward a tad"))
        {
            positionYTarget = 450;
            step = "extend sample 2";
        }
        else if(step.equalsIgnoreCase("extend sample 2"))
        {
            if(positionYTarget < -9999)
            {
                servoClawExtend.setPosition(armExtendedPosition - 0.1);
                timer.reset();
                step = "grab sample 2";
            }
        }
        else if(step.equalsIgnoreCase("grab sample 2"))
        {
            if(timer.milliseconds() > 400)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
                timer.reset();
                step = "arm to observation zone 1";
            }
        }
        else if(step.equalsIgnoreCase("arm to observation zone 1"))
        {
            if(timer.milliseconds() > 400)
            {
                servoClawExtend.setPosition(armRetractedPosition);
                slideTarget = 650;
                armTarget = 2000;//2000
                armDirection = "up";
                step = "drop sample 2";
            }
        }
        else if(step.equalsIgnoreCase("drop sample 2"))
        {
            if(armTarget < -9999 && slideTarget < 0)
            {
                servoClawGrabber.setPosition(clawOpenPosition);
                timer.reset();
                step = "lower slide and arm 1";
            }
        }
        else if(step.equalsIgnoreCase("lower slide and arm 1"))
        {
            if(timer.milliseconds() > 250)
            {
                armTarget = 50;
                armDirection = "down";
                slideTarget = 0;
                timer.reset();
                step = "reset encoders 1";
            }
        }
        else if(step.equalsIgnoreCase("reset encoders 1"))
        {
            if(armTarget < -9999 && (slideTarget < 0 || timer.milliseconds() > 2000))
            {
                bot.resetSlideAndArm();
                step = "rotate to sample 1";
            }
        }
        else if(step.equalsIgnoreCase("rotate to sample 1"))
        {
            orientationTarget = 335;
            step = "extend to sample 1";
        }
        else if(step.equalsIgnoreCase("extend to sample 1"))
        {
            if(orientationTarget < 0)
            {
                servoClawExtend.setPosition(armExtendedPosition);
                timer.reset();
                step = "grab sample 1";
            }
        }
        else if(step.equalsIgnoreCase("grab sample 1"))
        {
            if(timer.milliseconds() > 400)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
                timer.reset();
                step = "rotate to observation zone 2";
            }
        }
        else if(step.equalsIgnoreCase("rotate to observation zone 2"))
        {
            if(timer.milliseconds() > 400)
            {
                servoClawExtend.setPosition(armRetractedPosition);
                orientationTarget = 0;
                timer.reset();
                step = "arm to observation zone 2";
            }
        }
        else if(step.equalsIgnoreCase("arm to observation zone 2"))
        {
            if(orientationTarget < 0 && timer.milliseconds() > 300)
            {
                slideTarget = 650;//660
                armTarget = 2000;//2000
                armDirection = "up";
                step = "drop sample 1";
            }
        }
        else if(step.equalsIgnoreCase("drop sample 1"))
        {
            if(slideTarget < 0 && armTarget < -9999)
            {
                servoClawGrabber.setPosition(clawOpenPosition);
                timer.reset();
                step = "move back to grab s2";
            }
        }
        else if(step.equalsIgnoreCase("move back to grab s2"))
        {
            if(timer.milliseconds() > 200)
            {
                positionYTarget = 260;
                timer.reset();
                step = "slight pause and grab";
            }
        }
        else if(step.equalsIgnoreCase("slight pause and grab"))
        {
            if(timer.milliseconds() > 800 && positionYTarget < -9999)
            {
                positionYTarget = 240;
                timer.reset();
                step = "arm to hang position s2";
            }
            if(timer.milliseconds() > 400)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
            }
        }
        else if(step.equalsIgnoreCase("arm to hang position s2"))
        {
            if(timer.milliseconds() > 350 && positionYTarget < -9999)
            {
                positionXTarget = -120;
                bot.distanceToTargetPrevious = 10000;
                step = "square up for s2";
            }
            else if(timer.milliseconds() > 200)
            {
                slideTarget = 975;//975
                armTarget = 390;
                armDirection = "down";
            }
        }
        else if(step.equalsIgnoreCase("square up for s2"))
        {
            if(positionXTarget < -9999 && slideTarget < 0 && armTarget < -9999)
            {
                orientationTarget = 0;
                step = "move to hang position s2";
            }
        }
        else if(step.equalsIgnoreCase("move to hang position s2"))
        {
            if(orientationTarget < 0)
            {
                servoClawExtend.setPosition(armExtendedPosition);
                positionYTarget = 515;//475
                step = "lock s2 to bar";
            }
        }
        else if(step.equalsIgnoreCase("lock s2 to bar"))
        {
            if(positionYTarget < -9999)
            {
                positionYTarget = 360; //360
                bot.distanceToTargetPrevious = 10000;
                timer.reset();
                step = "lock s2 to bar started";
            }
        }
        else if(step.equalsIgnoreCase("lock s2 to bar started"))
        {
            armTarget = 425;
            armDirection = "up";
            if(timer.milliseconds() > 800)
            {
                servoClawGrabber.setPosition(clawOpenPosition);
                if(positionYTarget < -9999)
                {
                    timer.reset();
                    step = "go back to grab s3";
                }
            }
            else if(timer.milliseconds() > 600)
            {
                servoClawExtend.setPosition(armRetractedPosition);
            }
        }
        else if(step.equalsIgnoreCase("go back to grab s3"))
        {
            if(timer.milliseconds() > 200 && armTarget < -9999)
            {
                positionXTarget = 850;
                bot.distanceToTargetPrevious = 10000;
                step = "square up to grab s3";
            }
        }
        else if(step.equalsIgnoreCase("square up to grab s3"))
        {
            if(positionXTarget < -9999)
            {
                orientationTarget = 0;
                step = "rotate arm to grab s3";
            }
        }
        else if(step.equalsIgnoreCase("rotate arm to grab s3"))
        {
            if(orientationTarget < 0)
            {
                slideTarget = 730;
                armTarget = 1880;//1800
                armDirection = "up";
                step = "move back to grab s3";
            }
        }
        else if(step.equalsIgnoreCase("move back to grab s3"))
        {
            if(armTarget < -9999 && slideTarget < 0)
            {
                positionYTarget = 280;
                timer.reset();
                step = "wait and grab s3";
            }
        }
        else if(step.equalsIgnoreCase("wait and grab s3"))
        {
            if(timer.milliseconds() > 800 && positionYTarget < -9999)
            {
                positionYTarget = 240;
                bot.distanceToTargetPrevious = 10000;
                step = "move over to hang s3";
            }
            if(timer.milliseconds() > 390)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
            }
        }
        else if(step.equalsIgnoreCase("move over to hang s3"))
        {
            if(positionYTarget < -9999)
            {
                slideTarget = 965;//975
                armTarget = 370;
                armDirection = "down";
                positionXTarget = -220;
                bot.distanceToTargetPrevious = 10000;
                step = "get squared to hang s3";
            }
        }
        else if(step.equalsIgnoreCase("get squared to hang s3"))
        {
            if(positionXTarget < -9999)
            {
                orientationTarget = 0;
                step = "move forward to hang s3";
            }
        }
        else if(step.equalsIgnoreCase("move forward to hang s3"))
        {
            if(orientationTarget < 0)
            {
                servoClawExtend.setPosition(armExtendedPosition);
                positionYTarget = 515;//475
                step = "lock s3 to bar";
            }
        }
        else if(step.equalsIgnoreCase("lock s3 to bar"))
        {
            if(positionYTarget < -9999)
            {
                positionYTarget = 100; //360
                bot.distanceToTargetPrevious = 10000;
                timer.reset();
                step = "lock s3 to bar started";
            }
        }
        else if(step.equalsIgnoreCase("lock s3 to bar started"))
        {
            armTarget = 510;//490
            armDirection = "up";
            if(timer.milliseconds() > 800)
            {
                servoClawGrabber.setPosition(clawOpenPosition);
                if(positionYTarget < -9999)
                {
                    timer.reset();
                    step = "move far to the right";
                }
            }
            else if(timer.milliseconds() > 700)
            {
                servoClawExtend.setPosition(armRetractedPosition);
            }
        }
        else if(step.equalsIgnoreCase("move far to the right"))
        {
            positionXTarget = 1000;
        }
    }

    public void loop()
    {
        showTelemetry();

        rotateReturn = bot.navRotate(orientationTarget, rotationDirection, rotationErrorTolerance);
        distanceToTargetOrientation = rotateReturn[0];
        orientationTarget = rotateReturn[1];

        /*
        if(orientationTarget < 0)
        {
            positionReturn = bot.navToPosition(positionXTarget, positionYTarget, positionOrientationTarget, positionPrecise);
            distanceToTargetPosition = positionReturn[0];
            positionXTarget = positionReturn[1];
        }

         */

        positionXTarget = bot.navToXPosition(positionXTarget, positionOrientationTarget);
        positionYTarget = bot.navToYPosition(positionYTarget, positionOrientationTarget);

        slideTarget = bot.slideToPosition(slideTarget);
        armTarget = bot.armToPosition(armTarget, armDirection);

        autonomous();

    }

}

