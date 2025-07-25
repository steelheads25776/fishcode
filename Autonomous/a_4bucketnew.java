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
public class a_4bucketnew extends OpMode
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

    double navXTarget = 0.0;
    double navYTarget = 0.0;
    double navOrientationTarget = -1.0;  // -1.0 means no rotation
    boolean navPrecise = true;
    boolean navOnlyRotate = false;
    boolean navEngaged = false;
    double[] navReturn = new double[3];

    ElapsedTime timer = new ElapsedTime();

    String step = "None";

    public void showTelemetry()
    {
        bot.update();
        //telemetry.addData("degrees", bot.getDeg());
        telemetry.addLine("loop");

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
            //positionYTarget = 515;//495
            navXTarget = -200;
            navYTarget = 200;
            navOrientationTarget = 0;
            navEngaged = true;
            navPrecise = false;
            bot.distanceToTargetPrevious = 10000;
            step = "score sample 0";
        }

        else if(step.equalsIgnoreCase("score sample 0"))
        {
            if(!navEngaged)
            {
                navEngaged = true;
                navXTarget = -1160;
                navYTarget = 245;
                navOrientationTarget = 45;
                navOnlyRotate = false;
                navPrecise = true;
                //bot.distanceToTargetPrevious = 10000;
                servoClawExtend.setPosition(armExtendedPosition);
                armTarget = 1000;
                armDirection = "up";
                slideTarget = 2200;
                //armTarget = 1000;//1500
                //armDirection = "up";
                //slideTarget = 2200;
                step = "reach for bucket 0 arm";
                //step == "end";
            }
        }
        else if(step.equalsIgnoreCase("reach for bucket 0 arm"))
        {
            if(slideTarget < 0 && !navEngaged)
            {
                //servoClawExtend.setPosition(armExtendedPosition);
                armTarget = 1350;
                armDirection = "up";
                //step = "reach for bucket 1 started";
                step = "drop sample 0";
            }
        }
        else if(step.equalsIgnoreCase("drop sample 0"))
        {
            if(armTarget < -9999)
            {
                timer.reset();
                servoClawGrabber.setPosition(clawOpenPosition);
                step = "lower from bucket 0";
            }
        }
        else if(step.equalsIgnoreCase("lower from bucket 0"))
        {
            if(timer.milliseconds() > 200)
            {
                servoClawExtend.setPosition(armRetractedPosition);
                armTarget = 0;
                armDirection = "down";
                slideTarget = 0;
                navEngaged = true;
                navOrientationTarget = 0;
                navYTarget = 340;
                navXTarget = -1040;
                bot.distanceToTargetPrevious = 10000;
                timer.reset();
                step = "reset slide and arm 1";
            }
        }
        else if(step.equalsIgnoreCase("reset slide and arm 1"))
        {
            if((slideTarget >=0 || armTarget > -10000) && buttonArmStop.isPressed())
            {
                bot.resetSlideAndArm();
            }
            if (slideTarget < 0 && armTarget < -9999 && orientationTarget < 0)
            {
                step = "move to sample 1";
            }
        }
        else if(step.equalsIgnoreCase("move to sample 1"))
        {

            navYTarget = 380;
            navXTarget = -1040; // -1040
            navOrientationTarget = 0;
            navEngaged = true;
            bot.distanceToTargetPrevious = 10000;

            step = "reach for sample 1";
        }
        else if(step.equalsIgnoreCase("reach for sample 1"))
        {
            if (!navEngaged)
            {
                armTarget = 0;
                armDirection = "down";
                servoClawExtend.setPosition(armExtendedPosition);
                timer.reset();
                step = "grab and move to bucket 1";
            }
        }
        else if(step.equalsIgnoreCase("grab and move to bucket 1"))
        {
            if(timer.milliseconds() > 300)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
                if(timer.milliseconds() > 500)
                {
                    navEngaged = true;
                    navXTarget = -1160;
                    navYTarget = 245;
                    navOrientationTarget = 45;
                    navOnlyRotate = false;
                    navPrecise = true;
                    bot.distanceToTargetPrevious = 10000;
                    servoClawExtend.setPosition(armExtendedPosition);
                    armTarget = 1000;
                    armDirection = "up";
                    slideTarget = 2200;
                    //armTarget = 1000;//1500
                    //armDirection = "up";
                    //slideTarget = 2200;
                    step = "reach for bucket 1 arm";
                }
            }
        }
        else if(step.equalsIgnoreCase("reach for bucket 1 arm"))
        {
            if(slideTarget < 0 && !navEngaged)
            {
                //servoClawExtend.setPosition(armExtendedPosition);
                armTarget = 1350;
                armDirection = "up";
                //step = "reach for bucket 1 started";
                step = "drop sample 1";
            }
        }
        else if(step.equalsIgnoreCase("drop sample 1"))
        {
            if(armTarget < -9999)
            {
                timer.reset();
                servoClawGrabber.setPosition(clawOpenPosition);
                step = "lower from bucket 1";
            }
        }
        else if(step.equalsIgnoreCase("lower from bucket 1"))
        {
            if (timer.milliseconds() > 300)
            {
                servoClawExtend.setPosition(armRetractedPosition);
                armTarget = 0;
                armDirection = "down";
                slideTarget = 0;
                navEngaged = true;
                navOrientationTarget = 0;
                navYTarget = 380;
                navXTarget = -1300;
                bot.distanceToTargetPrevious = 10000;
                timer.reset();
                step = "reset encoders 1";
            }
        }
        else if(step.equalsIgnoreCase("reset encoders 1"))
        {
            if(armTarget < -9999 && (slideTarget < 0 || timer.milliseconds() > 2000) && !navEngaged)
            {
                bot.resetSlideAndArm();
                timer.reset();
                step = "reach for sample 2";
            }
        }

        else if(step.equalsIgnoreCase("reach for sample 2"))
        {

            if(timer.milliseconds() > 300)
            {
                servoClawExtend.setPosition(armExtendedPosition);
                timer.reset();
                step = "grab and move to bucket 2";
            }
        }
        else if(step.equalsIgnoreCase("grab and move to bucket 2"))
        {
            if(timer.milliseconds() > 300)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
                if(timer.milliseconds() > 500)
                {
                    navEngaged = true;
                    navXTarget = -1160;
                    navYTarget = 245;
                    navOrientationTarget = 45;
                    navOnlyRotate = false;
                    navPrecise = true;
                    bot.distanceToTargetPrevious = 10000;
                    servoClawExtend.setPosition(armExtendedPosition);
                    armTarget = 1000;
                    armDirection = "up";
                    slideTarget = 2200;
                    //armTarget = 1000;//1500
                    //armDirection = "up";
                    //slideTarget = 2200;
                    step = "reach for bucket 2 arm";
                    //step == "end";
                }
            }
        }
        else if(step.equalsIgnoreCase("reach for bucket 2 arm"))
        {
            if(slideTarget < 0 && !navEngaged)
            {
                //servoClawExtend.setPosition(armExtendedPosition);
                armTarget = 1350;
                armDirection = "up";
                //step = "reach for bucket 1 started";
                step = "drop sample 2";
            }
        }
        else if(step.equalsIgnoreCase("drop sample 2"))
        {
            if(armTarget < -9999)
            {
                timer.reset();
                servoClawGrabber.setPosition(clawOpenPosition);
                step = "lower from bucket 2";
            }
        }

        else if(step.equalsIgnoreCase("lower from bucket 2"))
        {
            if (timer.milliseconds() > 300)
            {
                servoClawExtend.setPosition(armRetractedPosition);
                armTarget = 0;
                armDirection = "down";
                slideTarget = 0;
                navEngaged = true;
                navOrientationTarget = 345;//335
                navYTarget = 380;
                navXTarget = -1370;
                bot.distanceToTargetPrevious = 10000;
                timer.reset();
                step = "reset encoders 2";
            }
        }
        else if(step.equalsIgnoreCase("reset encoders 2"))
        {
            if(armTarget < -9999 && (slideTarget < 0 || timer.milliseconds() > 2000) && !navEngaged)
            {
                bot.resetSlideAndArm();
                timer.reset();
                //step = "reach for sample 2";
                step = "reach for sample 3";
            }
        }

        else if(step.equalsIgnoreCase("reach for sample 3"))
        {
            if(timer.milliseconds() > 300)
            {
                servoClawExtend.setPosition(armExtendedPosition - 0.09);
                timer .reset();
                step = "grab and move to bucket 3";
            }
        }
        else if(step.equalsIgnoreCase("grab and move to bucket 3"))
        {
            if(timer.milliseconds() > 300)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
                if(timer.milliseconds() > 600)
                {
                    navEngaged = true;
                    navXTarget = -1160;
                    navYTarget = 245;
                    navOrientationTarget = 45;
                    navOnlyRotate = false;
                    navPrecise = true;
                    bot.distanceToTargetPrevious = 10000;
                    servoClawExtend.setPosition(armExtendedPosition);
                    armTarget = 1000;
                    armDirection = "up";
                    slideTarget = 2200;
                    //armTarget = 1000;//1500
                    //armDirection = "up";
                    //slideTarget = 2200;
                    step = "reach for bucket 3 arm";
                    //step == "end";
                }
            }
        }
        else if(step.equalsIgnoreCase("reach for bucket 3 arm"))
        {
            if(slideTarget < 0 && !navEngaged)
            {
                //servoClawExtend.setPosition(armExtendedPosition);
                armTarget = 1350;
                armDirection = "up";
                //step = "reach for bucket 1 started";
                step = "drop sample 3";
            }
        }
        else if(step.equalsIgnoreCase("drop sample 3"))
        {
            if(armTarget < -9999)
            {
                timer.reset();
                servoClawGrabber.setPosition(clawOpenPosition);
                step = "lower to park";
            }
        }
        else if(step.equalsIgnoreCase("lower to park"))
        {
            if(timer.milliseconds() > 300)
            {
                servoClawExtend.setPosition(armRetractedPosition);
                slideTarget = 0;
                armTarget = 650;
                armDirection = "down";
                step = "move to park position";
            }
        }
        else if(step.equalsIgnoreCase("move to park position"))
        {
            if(armTarget < -9999 && slideTarget < 0)
            {
                navEngaged = true;
                navXTarget = -680;
                navYTarget = 1400;
                navOrientationTarget = 90;
                navOnlyRotate = false;
                navPrecise = true;

                step = "extend arm to park";
            }
        }

        else if(step.equalsIgnoreCase("extend arm to park"))
        {
            if (!navEngaged)
            {
                servoClawExtend.setPosition(armExtendedPosition);
                servoClawGrabber.setPosition(clawClosedPosition);
                timer.reset();
                step = "move over a tad";
            }
        }
        else if(step.equalsIgnoreCase("move over a tad"))
        {
            if(timer.milliseconds() > 500)
            {
                navEngaged = true;
                navXTarget = -550;
                navYTarget = 1400;
                navOrientationTarget = 90;
                navOnlyRotate = false;
                navPrecise = true;
                armTarget = 500;
                armDirection = "down";
                step = "end";
            }
        }
    }

    public void loop()
    {
        showTelemetry();

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
