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

@Autonomous
public class a_4hangnew extends OpMode
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
            navXTarget = 50;
            navYTarget = 515;
            navOrientationTarget = 0;
            navEngaged = true;
            navPrecise = true;
            bot.distanceToTargetPrevious = 10000;
            //step = "start started";
            step = "start started";
        }
        else if (step.equalsIgnoreCase("start started"))
        {
            if(!navEngaged)
            {
                step = "move to hang s1 position";
            }
        }
        else if(step.equalsIgnoreCase("move to hang s1 position"))
        {
            servoClawExtend.setPosition(armExtendedPosition);
            slideTarget = 975;//975
            armTarget = 390;//420 400
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
            //positionYTarget = 360; //370
            navXTarget = 50;
            navYTarget = 360;
            navOrientationTarget = 0;
            navEngaged = true;
            bot.distanceToTargetPrevious = 10000;

            timer.reset();
            step = "lock s1 to bar started";
        }
        else if(step.equalsIgnoreCase("lock s1 to bar started"))
        {
            armTarget = 425;//415
            armDirection = "up";
            if(timer.milliseconds() > 600)
            {
                servoClawGrabber.setPosition(clawOpenPosition);
                if(!navEngaged)
                {
                    step = "move to sample 1";
                }
            }
            else if(timer.milliseconds() > 300)
            {
                servoClawExtend.setPosition(armRetractedPosition);
            }
        }
        else if(step.equalsIgnoreCase("lower arm from specimen 1"))
        {
            //orientationTarget = 0;
            slideTarget = 0;
            armDirection = "down";
            armTarget = 0;


            step = "move to sample 1";
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
            slideTarget = 0;
            armDirection = "down";
            armTarget = 0;
            navYTarget = 380;
            navXTarget = 1300;
            navOrientationTarget = 335;
            navEngaged = true;
            bot.distanceToTargetPrevious = 10000;

            step = "reach for sample 1";
        }
        else if(step.equalsIgnoreCase("reach for sample 1"))
        {
            if (!navEngaged)
            {
                //armTarget = 0;
                //armDirection = "down";
                servoClawExtend.setPosition(armExtendedPosition);
                timer.reset();
                step = "grab and rotate to drop 1";
            }
        }
        else if(step.equalsIgnoreCase("grab and rotate to drop 1"))
        {
            if(timer.milliseconds() > 300)//200
            {
                servoClawGrabber.setPosition(clawClosedPosition);
                if(timer.milliseconds() > 600)//500
                {
                    navEngaged = true;
                    navOrientationTarget = 195;
                    servoClawExtend.setPosition((armExtendedPosition) - 0.25);
                    navOnlyRotate = true;
                    navPrecise = true;
                    timer.reset();
                    step = "drop sample 1";
                }
            }
        }
        else if(step.equalsIgnoreCase("drop sample 1"))
        {
            if(!navEngaged)
            {
                servoClawGrabber.setPosition(clawOpenPosition);
                servoClawExtend.setPosition(armRetractedPosition);
                timer.reset();
                step = "move to sample 2";
            }
        }
        else if(step.equalsIgnoreCase("move to sample 2"))
        {
            if(timer.milliseconds() > 200)
            {
                navEngaged = true;
                navOrientationTarget = 0;
                //navYTarget = 380;
                //navXTarget = 1300;
                navOnlyRotate = true;
                navPrecise = true;
                bot.distanceToTargetPrevious = 10000;
                timer.reset();
                step = "reach for sample 2";
            }
        }
        else if(step.equalsIgnoreCase("reach for sample 2"))
        {
            if (!navEngaged)
            {
                servoClawExtend.setPosition((armExtendedPosition) - 0.05);
                timer.reset();
                step = "grab and rotate to drop 2";
            }
        }
        else if(step.equalsIgnoreCase("grab and rotate to drop 2"))
        {
            if(timer.milliseconds() > 300)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
                if(timer.milliseconds() > 600)
                {
                    navEngaged = true;
                    navOrientationTarget = 195;
                    servoClawExtend.setPosition((armExtendedPosition) - 0.28);
                    navOnlyRotate = true;
                    navPrecise = true;
                    step = "drop sample 2";
                }
            }
        }
        else if(step.equalsIgnoreCase("drop sample 2"))
        {
            if(!navEngaged)
            {
                servoClawGrabber.setPosition(clawOpenPosition);
                servoClawExtend.setPosition(armRetractedPosition);
                timer.reset();
                step = "rotate to grab specimen 2";
            }
        }
        else if(step.equalsIgnoreCase("rotate to grab specimen 2"))
        {
            if(timer.milliseconds() > 300)
            {
                navOnlyRotate = true;
                navEngaged = true;
                navOrientationTarget = 220;
                navPrecise = true;
                timer.reset();
                step = "reach and grab specimen 2";
            }
        }
        else if(step.equalsIgnoreCase("reach and grab specimen 2"))
        {
            if(!navEngaged)
            {
                servoClawExtend.setPosition(armExtendedPosition);

                timer.reset();
                step = "rotate to hang s2";

            }
        }
        else if(step.equalsIgnoreCase("rotate to hang s2"))
        {
            if(timer.milliseconds() > 300)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
                if(timer.milliseconds() > 600)
                {
                    servoClawExtend.setPosition(armRetractedPosition);
                    //positionYTarget = 515;//495
                    navOrientationTarget = 0;
                    navOnlyRotate = true;
                    navEngaged = true;
                    navPrecise = true;
                    bot.distanceToTargetPrevious = 10000;
                    //step = "start started";
                    step = "move to hang s2";
                }
            }
        }
        else if(step.equalsIgnoreCase("move to hang s2"))
        {
            if(!navEngaged)
            {
                //positionYTarget = 515;//495
                armTarget = 750;
                armDirection = "up";
                navXTarget = -50;
                navYTarget = 515;
                navOrientationTarget = 0;
                navOnlyRotate = false;
                navEngaged = true;
                navPrecise = true;
                bot.distanceToTargetPrevious = 10000;
                //step = "start started";
                step = "start started 2";
            }
        }
        else if (step.equalsIgnoreCase("start started 2"))
        {
            if(!navEngaged)
            {
                step = "move to hang s2 position";
            }
        }
        else if(step.equalsIgnoreCase("move to hang s2 position"))
        {
            servoClawExtend.setPosition(armExtendedPosition);
            slideTarget = 975;//975
            armTarget = 390;//420 400
            armDirection = "down";
            step = "move to hang s2 position started";
        }
        else if(step.equalsIgnoreCase("move to hang s2 position started"))
        {
            if(slideTarget < 0)
            {
                timer.reset();
                step = "lock s2 to bar";
            }
        }
        else if(step.equalsIgnoreCase("lock s2 to bar"))
        {
            //positionYTarget = 360; //370
            navXTarget = -50;
            navYTarget = 350;//360
            navOrientationTarget = 0;
            navEngaged = true;
            bot.distanceToTargetPrevious = 10000;

            timer.reset();
            step = "lock s2 to bar started";
        }
        else if(step.equalsIgnoreCase("lock s2 to bar started"))
        {
            armTarget = 425;//415
            armDirection = "up";
            if(timer.milliseconds() > 600)
            {
                servoClawGrabber.setPosition(clawOpenPosition);
                if(!navEngaged)
                {
                    step = "move to specimen 3";
                }
            }
            else if(timer.milliseconds() > 300)
            {
                servoClawExtend.setPosition(armRetractedPosition);
            }
        }
        else if(step.equalsIgnoreCase("lower arm from specimen 2"))
        {
            //orientationTarget = 0;
            slideTarget = 0;
            armDirection = "down";
            armTarget = 0;
            step = "reset slide and arm 2";
        }
        else if(step.equalsIgnoreCase("reset slide and arm 2"))
        {
            if((slideTarget >=0 || armTarget > -10000) && buttonArmStop.isPressed())
            {
                bot.resetSlideAndArm();
            }
            if (slideTarget < 0 && armTarget < -9999 && orientationTarget < 0)
            {
                step = "move to specimen 3";
            }
        }
        else if(step.equalsIgnoreCase("move to specimen 3"))
        {
            slideTarget = 0;
            armDirection = "down";
            armTarget = 0;
            navYTarget = 400;
            navXTarget = 500; // -1040
            navOrientationTarget = 125;
            navEngaged = true;
            bot.distanceToTargetPrevious = 10000;
            //servoClawExtend.setPosition(armExtendedPosition);
            timer.reset();

            step = "reach and grab specimen 3";
        }
        else if(step.equalsIgnoreCase("reach and grab specimen 3"))
        {
            if(!navEngaged)
            {
                servoClawExtend.setPosition(armExtendedPosition);

                timer.reset();
                step = "rotate to hang s3";

            }
        }
        else if(step.equalsIgnoreCase("rotate to hang s3"))
        {
            if(timer.milliseconds() > 300)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
                if(timer.milliseconds() > 600)
                {
                    servoClawExtend.setPosition(armRetractedPosition);
                    //positionYTarget = 515;//495
                    navOrientationTarget = 0;
                    navOnlyRotate = true;
                    navEngaged = true;
                    navPrecise = true;
                    bot.distanceToTargetPrevious = 10000;
                    //step = "start started";
                    step = "move to hang s3";
                }
            }
        }
        else if(step.equalsIgnoreCase("move to hang s3"))
        {
            if(!navEngaged)
            {
                //positionYTarget = 515;//495
                armTarget = 750;
                armDirection = "up";
                navXTarget = -150;
                navYTarget = 515;
                navOrientationTarget = 0;
                navOnlyRotate = false;
                navEngaged = true;
                navPrecise = true;
                bot.distanceToTargetPrevious = 10000;
                //step = "start started";
                step = "start started 3";
            }
        }
        else if (step.equalsIgnoreCase("start started 3"))
        {
            if(!navEngaged)
            {
                step = "move to hang s3 position";
            }
        }
        else if(step.equalsIgnoreCase("move to hang s3 position"))
        {
            servoClawExtend.setPosition(armExtendedPosition);
            slideTarget = 975;//975
            armTarget = 390;//420 400
            armDirection = "down";
            step = "move to hang s3 position started";
        }
        else if(step.equalsIgnoreCase("move to hang s3 position started"))
        {
            if(slideTarget < 0)
            {
                timer.reset();
                step = "lock s3 to bar";
            }
        }
        else if(step.equalsIgnoreCase("lock s3 to bar"))
        {
            //positionYTarget = 360; //370
            navXTarget = -150;
            navYTarget = 345;//360
            navOrientationTarget = 0;
            navEngaged = true;
            bot.distanceToTargetPrevious = 10000;

            timer.reset();
            step = "lock s3 to bar started";
        }
        else if(step.equalsIgnoreCase("lock s3 to bar started"))
        {
            armTarget = 425;//415
            armDirection = "up";
            if(timer.milliseconds() > 600)
            {
                servoClawGrabber.setPosition(clawOpenPosition);
                if(!navEngaged)
                {
                    step = "move to specimen 4";
                }
            }
            else if(timer.milliseconds() > 300)
            {
                servoClawExtend.setPosition(armRetractedPosition);
            }
        }
        else if(step.equalsIgnoreCase("lower arm from specimen 3"))
        {
            //orientationTarget = 0;
            slideTarget = 0;
            armDirection = "down";
            armTarget = 0;
            step = "reset slide and arm 3";
        }
        else if(step.equalsIgnoreCase("reset slide and arm 3"))
        {
            if((slideTarget >=0 || armTarget > -10000) && buttonArmStop.isPressed())
            {
                bot.resetSlideAndArm();
            }
            if (slideTarget < 0 && armTarget < -9999 && orientationTarget < 0)
            {
                step = "move to specimen 4";
            }
        }
        else if(step.equalsIgnoreCase("move to specimen 4"))
        {

            slideTarget = 0;
            armDirection = "down";
            armTarget = 0;
            navYTarget = 400;
            navXTarget = 500; // -1040
            navOrientationTarget = 125;
            navEngaged = true;
            bot.distanceToTargetPrevious = 10000;
            //servoClawExtend.setPosition(armExtendedPosition);
            timer.reset();

            step = "reach and grab specimen 4";
        }
        else if(step.equalsIgnoreCase("reach and grab specimen 4"))
        {
            if(!navEngaged)
            {
                servoClawExtend.setPosition(armExtendedPosition);

                timer.reset();
                step = "rotate to hang s4";
                //step = "move to hang s4";
            }
        }
        else if(step.equalsIgnoreCase("rotate to hang s4"))
        {
            if(timer.milliseconds() > 300)
            {
                servoClawGrabber.setPosition(clawClosedPosition);
                if(timer.milliseconds() > 600)
                {
                    servoClawExtend.setPosition(armRetractedPosition);
                    //positionYTarget = 515;//495
                    navOrientationTarget = 0;
                    navOnlyRotate = true;
                    navEngaged = true;
                    navPrecise = true;
                    bot.distanceToTargetPrevious = 10000;
                    //step = "start started";
                    step = "move to hang s4";
                }
            }
        }
        else if(step.equalsIgnoreCase("move to hang s4"))
        {
            if(!navEngaged)
            {
                //positionYTarget = 515;//495
                armTarget = 750;
                armDirection = "up";
                navXTarget = -250;
                navYTarget = 515;
                navOrientationTarget = 0;
                navOnlyRotate = false;
                navEngaged = true;
                navPrecise = true;
                bot.distanceToTargetPrevious = 10000;
                //step = "start started";
                step = "start started 4";
            }
        }
        else if (step.equalsIgnoreCase("start started 4"))
        {
            if(!navEngaged)
            {
                step = "move to hang s4 position";
            }
        }
        else if(step.equalsIgnoreCase("move to hang s4 position"))
        {
            servoClawExtend.setPosition(armExtendedPosition);
            slideTarget = 975;//975
            armTarget = 390;//420 400
            armDirection = "down";
            step = "move to hang s4 position started";
        }
        else if(step.equalsIgnoreCase("move to hang s4 position started"))
        {
            if(slideTarget < 0)
            {
                timer.reset();
                step = "lock s4 to bar";
            }
        }
        else if(step.equalsIgnoreCase("lock s4 to bar"))
        {
            //positionYTarget = 360; //370
            navXTarget = -250;
            navYTarget = 345;//360
            navOrientationTarget = 0;
            navEngaged = true;
            bot.distanceToTargetPrevious = 10000;

            timer.reset();
            step = "lock s4 to bar started";
        }
        else if(step.equalsIgnoreCase("lock s4 to bar started"))
        {
            armTarget = 425;//415
            armDirection = "up";
            if(timer.milliseconds() > 600)
            {
                servoClawGrabber.setPosition(clawOpenPosition);
                if(!navEngaged)
                {
                    step = "lower arm from specimen 4";
                }
            }
            else if(timer.milliseconds() > 300)
            {
                servoClawExtend.setPosition(armRetractedPosition);
            }
        }
        else if(step.equalsIgnoreCase("lower arm from specimen 4"))
        {
            //orientationTarget = 0;
            slideTarget = 0;
            armDirection = "down";
            armTarget = 0;
            navXTarget = 1200;
            navYTarget = 50;
            navOrientationTarget = 0;
            navEngaged = true;
            navOnlyRotate = false;
            navPrecise = false;
            step = "end";
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
