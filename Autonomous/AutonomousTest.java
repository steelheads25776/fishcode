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
public class AutonomousTest extends OpMode
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

    boolean[] steps = new boolean[30];
    boolean[] stepsStarted = new boolean[steps.length];

    ElapsedTime timer = new ElapsedTime();

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
        for(int i = 0; i < steps.length; i++)
        {
            steps[i] = false;
            stepsStarted[i] = false;
        }

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

        armTarget = 2800;
        //armTarget = -1;
        servoClawExtend.setPosition(armRetractedPosition);
        servoClawGrabber.setPosition(clawOpenPosition);

        bot.voltageStart = sensorVoltage.getVoltage();
    }

    public void init_loop()
    {
        armTarget = bot.armToPosition(armTarget, false);
    }

    public void start()
    {
        telemetry.addLine("start");
        telemetry.update();
        /*
        positionXTarget = 0;
        positionYTarget = 1000;
        while(positionXTarget > -10000);

        orientationTarget = 180;
        while(orientationTarget > -1);

        armTarget = 2500;
        while(armTarget > -1);

        armTarget = 0;
        while(armTarget > -1);

        orientationTarget = 0;
        while(orientationTarget > -1);

        positionXTarget = 0;
        positionYTarget = 0;

         */
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

        //steps[0] = true;
        //steps[1] = true;
        //steps[0] = true;
        //steps[0] = true;


        if(steps[0] == false && stepsStarted[0] == false)
        {
            // SPECIMEN 1 - close grabber, set slide and arm positions, and move to bar
            servoClawGrabber.setPosition(clawClosedPosition);
            orientationTarget = 0;
            positionXTarget = 0;
            positionYTarget = 730;
            positionOrientationTarget = 0;
            positionPrecise = true;
            slideTarget = 930;
            stepsStarted[0] = true;
        }
        else if(stepsStarted[0] == true && steps[0] == false && positionXTarget < -9999 && slideTarget <= -1)
        {
            steps[0] = true;
        }

        if(steps[0] == true && stepsStarted[1] == false)
        {
            // SPECIMEN 1 - move arm to lock specimen to bar
            armTarget = 1600;
            stepsStarted[1] = true;
            timer.reset();
        }
        else if(stepsStarted[1] == true && steps[1] == false && timer.milliseconds() >= 500)
        {
            steps[1] = true;
        }

        if(steps[1] == true && stepsStarted[2] == false)
        {
            // SPECIMEN 1 - release grabber and move toward sample 2 via waypoint
            servoClawGrabber.setPosition(clawOpenPosition);
            bot.sleep(200);
            positionXTarget = 100;
            positionYTarget = 400;
            positionOrientationTarget = 0;
            positionPrecise = false;
            bot.distanceToTargetPrevious = 1000;
            armTarget = 0;
            slideTarget = 0;
            stepsStarted[2] = true;
        }
        else if(stepsStarted[2] && steps[2] == false && positionXTarget < -9999)
        {
            steps[2] = true;
        }

        if(steps[2] == true && stepsStarted[3] == false)
        {
            // SAMPLE 1 - move to lined up with sample 2
            positionXTarget = 1300;
            positionYTarget = 460;
            positionPrecise = true;
            stepsStarted[3] = true;
        }
        else if(stepsStarted[3] == true && steps[3] == false && positionXTarget < -9999 && armTarget < -9999)
        {
            steps[3] = true;
        }

        if(steps[3] == true && stepsStarted[4] == false)
        {
            // SAMPLE 1 - rotate to lined up with sample 1 and extend arm to grab position
            orientationTarget = 330;
            rotationErrorTolerance = 1.5;
            servoClawExtend.setPosition(armRetractedPosition + 0.25);
            stepsStarted[4] = true;
        }
        else if(stepsStarted[4] == true && steps[4] == false && orientationTarget < 0)
        {
            steps[4] = true;
        }

        if(steps[4] == true && stepsStarted[5] == false)
        {
            // SAMPLE 1 - grab sample 1, rotate to player area, and retract arm
            servoClawGrabber.setPosition(clawClosedPosition);
            bot.sleep(250);
            servoClawExtend.setPosition(armRetractedPosition + 0.1);
            orientationTarget = 180;
            rotationDirection = "counterclockwise";
            rotationErrorTolerance = 4.0;
            stepsStarted[5] = true;
        }
        else if(stepsStarted[5] == true && steps[5] == false && orientationTarget < 0)
        {
            steps[5] = true;
        }

        if(steps[5] == true && stepsStarted[6] == false)
        {
            // SAMPLE 1 - release sample 1 and rotate sample 2 grab position
            servoClawGrabber.setPosition(clawOpenPosition);
            bot.sleep(100);
            servoClawExtend.setPosition(armRetractedPosition + 0.18);
            orientationTarget = 0;
            rotationDirection = "clockwise";
            rotationErrorTolerance = 1.5;
            //positionXTarget = 1300;
            //positionYTarget = 450;
            //positionPrecise = true;
            stepsStarted[6] = true;
        }
        else if(stepsStarted[6] == true && steps[6] == false && orientationTarget < 0) // && positionXTarget < -9999)
        {
            steps[6] = true;
        }

        if(steps[6] == true && stepsStarted[7] == false)
        {
            // SAMPLE 2 - grab sample 2 and rotate to player area
            servoClawGrabber.setPosition(clawClosedPosition);
            bot.sleep(250);
            servoClawExtend.setPosition(armRetractedPosition + 0.1);
            orientationTarget = 180;
            rotationDirection = "counterclockwise";
            rotationErrorTolerance = 4.0;
            stepsStarted[7] = true;
        }
        else if(stepsStarted[7] == true && steps[7] == false && orientationTarget < 0)
        {
            steps[7] = true;
            steps[8] = true;
            steps[9] = true;
        }

        if(steps[7] == true && stepsStarted[8] == false)
        {
            // SAMPLE 2 - release sample 2, and rotate sample 3 grab position
            servoClawGrabber.setPosition(clawOpenPosition);
            bot.sleep(100);
            servoClawExtend.setPosition(armRetractedPosition + 0.4);
            orientationTarget = 30;
            rotationDirection = "clockwise";
            rotationErrorTolerance = 1.5;
            stepsStarted[8] = true;
        }

        else if(stepsStarted[8] == true && steps[8] == false && orientationTarget < 0)
        {
            steps[8] = true;
        }

        if(steps[8] == true && stepsStarted[9] == false)
        {
            // SAMPLE 3 - grab sample 3 and rotate to player area
            servoClawGrabber.setPosition(clawClosedPosition);
            bot.sleep(250);
            servoClawExtend.setPosition(armRetractedPosition + 0.1);
            orientationTarget = 180;
            rotationDirection = "counterclockwise";
            rotationErrorTolerance = 4.0;
            stepsStarted[9] = true;
        }
        else if(stepsStarted[9] == true && steps[9] == false && orientationTarget < 0)
        {
            steps[9] = true;
        }

        if(steps[9] == true && stepsStarted[10] == false)
        {
            // SAMPLE 3 - release sample 3, lift slide to sample grab height, and extend arm to grab position
            servoClawGrabber.setPosition(clawOpenPosition);
            servoClawExtend.setPosition(armRetractedPosition + 0.32);
            slideTarget = 1000;
            orientationTarget = 180;
            rotationErrorTolerance = 1.5;
            stepsStarted[10] = true;
        }
        else if(stepsStarted[10] && steps[10] == false && slideTarget < 0)
        {
            steps[10] = true;
        }

        if(steps[10] == true && stepsStarted[11] == false)
        {
            // SPECIMEN 2 - grab specimen 2
            servoClawGrabber.setPosition(clawClosedPosition);
            bot.sleep(200);
            slideTarget = 1200;
            armTarget = 2800;
            stepsStarted[11] = true;
        }
        else if(stepsStarted[11] == true && steps[11] == false && slideTarget < 0)
        {
            steps[11] = true;
        }

        if(steps[11] == true && stepsStarted[12] == false)
        {
            // SPECIMEN 2 - move to bar
            orientationTarget = 0;
            rotationDirection = "clockwise";
            servoClawExtend.setPosition(armRetractedPosition);
            positionXTarget = -100;
            positionYTarget = 720;
            positionOrientationTarget = 0;
            positionPrecise = true;
            slideTarget = 930;
            stepsStarted[12] = true;
        }
        else if(stepsStarted[12] == true && steps[12] == false && orientationTarget < 0 && positionXTarget < -9999)
        {
            steps[12] = true;
        }        

        if(steps[12] == true && stepsStarted[13] == false)
        {
            // SPECIMEN 2 - move arm to lock specimen to bar
            orientationTarget = 0;
            armTarget = 1600;
            stepsStarted[13] = true;
            timer.reset();
        }
        else if(stepsStarted[13] == true && steps[13] == false && timer.milliseconds() >= 500)
        {
            steps[13] = true;
        }

        if(steps[13] == true && stepsStarted[14] == false)
        {
            // SPECIMEN 2 - release grabber and move toward specimen 3 via waypoint
            servoClawGrabber.setPosition(clawOpenPosition);
            positionXTarget = 100;
            positionYTarget = 400;
            positionOrientationTarget = 0;
            positionPrecise = false;
            bot.distanceToTargetPrevious = 1000;
            armTarget = 0;
            slideTarget = 800;
            stepsStarted[14] = true;
        }
        else if(stepsStarted[14] == true && steps[14] == false && positionXTarget < -9999)
        {
            steps[14] = true;
        }

        if(steps[14] == true && stepsStarted[15] == false)
        {
            // SPECIMEN 2 - release grabber and move toward specimen 3 via waypoint
            servoClawGrabber.setPosition(clawOpenPosition);
            bot.sleep(100);
            positionYTarget = 450;
            positionXTarget = 600;
            positionPrecise = false;
            bot.distanceToTargetPrevious = 1000;
            stepsStarted[15] = true;
        }
        else if(stepsStarted[15] == true && steps[15] == false && positionXTarget < -9999)
        {
            steps[15] = true;
        }

        if(steps[15] == true && stepsStarted[16] == false)
        {
            // SPECIMEN 3 - move to grab position
            positionYTarget = 450;
            positionXTarget = 850;
            slideTarget = 1000;
            positionPrecise = true;
            stepsStarted[16] = true;
        }
        else if(stepsStarted[16] == true && steps[16] == false && positionXTarget < -9999)
        {
            steps[16] = true;
        }

        if(steps[16] == true && stepsStarted[17] == false)
        {
            // SPECIMEN 3 - rotate and extend arm
            servoClawExtend.setPosition(armRetractedPosition + 0.32);
            orientationTarget = 180;
            rotationDirection = "counterclockwise";
            stepsStarted[17] = true;
        }
        else if(stepsStarted[17] == true && steps[17] == false && orientationTarget < 0)
        {
            steps[17] = true;
        }

        if(steps[17] == true && stepsStarted[18] == false)
        {
            // SPECIMEN 3 - grab specimen 3
            servoClawGrabber.setPosition(clawClosedPosition);
            bot.sleep(200);
            slideTarget = 1200;
            armTarget = 2800;
            stepsStarted[18] = true;
        }
        else if(stepsStarted[18] == true && steps[18] == false && slideTarget < 0)
        {
            steps[18] = true;
        }

        if(steps[18] == true && stepsStarted[19] == false)
        {
            // SPECIMEN 3 - move to bar
            orientationTarget = 0;
            rotationDirection = "clockwise";
            servoClawExtend.setPosition(armRetractedPosition);
            positionXTarget = -200;
            positionYTarget = 730;
            positionOrientationTarget = 0;
            positionPrecise = true;
            slideTarget = 930;
            stepsStarted[19] = true;
        }
        else if(stepsStarted[19] == true && steps[19] == false && orientationTarget < 0 && positionXTarget < -9999)
        {
            steps[19] = true;
        }

        if(steps[19] == true && stepsStarted[20] == false)
        {
            // SPECIMEN 2 - move arm to lock specimen to bar
            armTarget = 1600;
            stepsStarted[20] = true;
            timer.reset();
        }
        else if(stepsStarted[20] == true && steps[20] == false && timer.milliseconds() >= 500)
        {
            steps[20] = true;
        }

        if(steps[20] == true && stepsStarted[21] == false)
        {
            // SPECIMEN 23 - release grabber and move toward specimen 3 via waypoint
            servoClawGrabber.setPosition(clawOpenPosition);
            bot.sleep(200);
            positionXTarget = 100;
            positionYTarget = 400;
            positionOrientationTarget = 0;
            positionPrecise = false;
            bot.distanceToTargetPrevious = 1000;
            armTarget = 0;
            slideTarget = 800;
            stepsStarted[21] = true;
        }
        else if(stepsStarted[21] == true && steps[21] == false && positionXTarget < -9999)
        {
            steps[21] = true;
        }

        if(steps[21] == true && stepsStarted[22] == false)
        {
            // PARK
            positionXTarget = 850;
            positionYTarget = 0;
            slideTarget = 0;
            armTarget = 0;
            positionPrecise = true;
            stepsStarted[22] = true;
        }
        else if(stepsStarted[22] == true && steps[22] == false && positionXTarget < -9999)
        {
            steps[22] = true;
        }

    }

}
