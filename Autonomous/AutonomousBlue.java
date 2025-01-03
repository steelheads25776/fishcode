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
public class AutonomousBlue extends OpMode
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

    boolean[] steps = new boolean[20];
    boolean[] stepsStarted = new boolean[steps.length];

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

        telemetry.addData("D2T Previous", bot.distanceToTargetPrevious);
        telemetry.addData("D Travel Previous", bot.distanceTraveledFromPrevious);
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

        if(steps[0] == false && stepsStarted[0] == false)
        {
            // SPECIMEN 1 - close grabber, set slide and arm positions, and move to bar
            servoClawGrabber.setPosition(clawClosedPosition);
            orientationTarget = 0;
            positionXTarget = 0;
            positionYTarget = 720;
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
            armTarget = 1550;
            stepsStarted[1] = true;
        }
        else if(stepsStarted[1] == true && steps[1] == false && armTarget < -9999)
        {
            steps[1] = true;
        }

        if(steps[1] == true && stepsStarted[2] == false)
        {
            // SPECIMEN 1 - release grabber and move toward specimen 2 via waypoint
            servoClawGrabber.setPosition(clawOpenPosition);
            positionXTarget = 100;
            positionYTarget = 500;
            positionOrientationTarget = 0;
            positionPrecise = false;
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
            // SPECIMEN 3 - move to specimen 3 grab position
            positionXTarget = 1010;
            positionYTarget = 500;
            positionPrecise = true;
            stepsStarted[3] = true;
        }
        else if(stepsStarted[3] == true && steps[3] == false && positionXTarget < -9999 && armTarget < -9999)
        {
            steps[3] = true;
        }

        if(steps[3] == true && stepsStarted[4] == false)
        {
            // SPECIMEN 3 - grab specimen
            servoClawGrabber.setPosition(clawClosedPosition);
            stepsStarted[4] = true;
        }
        else if(stepsStarted[4] == true && steps[4] == false)
        {
            bot.sleep(200);
            steps[4] = true;
        }

        if(steps[4] == true && stepsStarted[5] == false)
        {
            // SPECIMEN 3 - move arm back to drop specimen in player area
            armTarget = 7620;
            slideTarget = 720;
            positionYTarget = 400;
            positionXTarget = 1010;
            stepsStarted[5] = true;
        }
        else if(stepsStarted[5] == true && steps[5] == false && positionXTarget < -9999 && armTarget < -9999)
        {
            steps[5] = true;
        }

        if(steps[5] == true && stepsStarted[6] == false)
        {
            // SPECIMEN 2 - release specimen 3 and move toward hanging specimen 2
            servoClawGrabber.setPosition(clawOpenPosition);
            bot.sleep(100);
            positionYTarget = 450;
            positionXTarget = 600;
            positionPrecise = false;
            stepsStarted[6] = true;
        }
        else if(stepsStarted[6] == true && steps[6] == false && positionXTarget < -9999)
        {
            steps[6] = true;
        }

        if(steps[6] == true && stepsStarted[7] == false)
        {
            // SPECIMEN 2 - move to grab position for hanging specimen 2
            positionYTarget = 300;
            positionXTarget = 700;
            positionPrecise = true;
            stepsStarted[7] = true;
        }
        else if(stepsStarted[7] == true && steps[7] == false && positionXTarget < -9999)
        {
            steps[7] = true;
        }

        if(steps[7] == true && stepsStarted[8] == false)
        {
            // SPECIMEN 2 - grab hanging specimen 2 and move arm to bar placement position
            servoClawGrabber.setPosition(clawClosedPosition);
            bot.sleep(200);
            armTarget = 800;
            stepsStarted[8] = true;
        }
        else if(stepsStarted[8] == true && steps[8] == false && motorClawArm.getCurrentPosition() < 4000 && motorClawArm.getCurrentPosition() > 3500)
        {
            // SPECIMEN 2 - move slide to bar placement position and move toward bar via waypoint
            slideTarget = 1500;
            positionXTarget = -100;
            positionYTarget = 550;
            positionPrecise = true;
        }
        else if(stepsStarted[8] == true && steps[8] == false && armTarget < -9999 && positionXTarget < -9999)
        {
            //bot.sleep(200);
            steps[8] = true;
        }

        if(steps[8] == true && stepsStarted[9] == false)
        {
            // SPECIMEN 2 - move to bar placement position and move to bar
            positionXTarget = -100;
            positionYTarget = 620;
            positionPrecise = true;
            armTarget = 2700;
            stepsStarted[9] = true;
        }
        else if(stepsStarted[9] == true && steps[9] == false && armTarget < -9999)
        {
            steps[9] = true;
        }

        if(steps[9] == true && stepsStarted[10] == false)
        {
            servoClawGrabber.setPosition(clawOpenPosition);
            bot.sleep(200);
            positionXTarget = 100;
            positionYTarget = 500;
            positionOrientationTarget = 0;
            positionPrecise = false;
            armTarget = 0;
            slideTarget = 0;
            stepsStarted[10] = true;
        }
        else if(stepsStarted[10] && steps[10] == false && positionXTarget < -9999)
        {
            steps[10] = true;
        }

        if(steps[10] == true && stepsStarted[11] == false)
        {
            positionXTarget = 1270;
            positionYTarget = 500;
            positionPrecise = true;
            stepsStarted[11] = true;
        }
        else if(stepsStarted[11] == true && steps[11] == false && positionXTarget < -9999 && armTarget < -9999)
        {
            steps[11] = true;
        }

        if(steps[11] == true && stepsStarted[12] == false)
        {
            servoClawGrabber.setPosition(clawClosedPosition);
            stepsStarted[12] = true;
        }
        else if(stepsStarted[12] == true && steps[12] == false)
        {
            bot.sleep(200);
            steps[12] = true;
        }

        if(steps[12] == true && stepsStarted[13] == false)
        {
            armTarget = 7620;
            slideTarget = 720;
            positionYTarget = 400;
            positionXTarget = 1010;
            stepsStarted[13] = true;
        }
        else if(stepsStarted[13] == true && steps[13] == false && positionXTarget < -9999 && armTarget < -9999)
        {
            steps[13] = true;
        }

        if(steps[13] == true && stepsStarted[14] == false)
        {
            servoClawGrabber.setPosition(clawOpenPosition);
            bot.sleep(100);
            positionYTarget = 450;
            positionXTarget = 600;
            positionPrecise = false;
            stepsStarted[14] = true;
        }
        else if(stepsStarted[14] == true && steps[14] == false && positionXTarget < -9999)
        {
            steps[14] = true;
        }

        if(steps[14] == true && stepsStarted[15] == false)
        {
            positionYTarget = 300;
            positionXTarget = 700;
            positionPrecise = true;
            stepsStarted[15] = true;
        }
        else if(stepsStarted[15] == true && steps[15] == false && positionXTarget < -9999)
        {
            steps[15] = true;
        }

        if(steps[15] == true && stepsStarted[16] == false)
        {
            // SPECIMEN 2 - grab hanging specimen 2 and move arm to bar placement position
            servoClawGrabber.setPosition(clawClosedPosition);
            bot.sleep(200);
            armTarget = 800;
            stepsStarted[16] = true;
        }
        else if(stepsStarted[16] == true && steps[16] == false && motorClawArm.getCurrentPosition() < 4000 && motorClawArm.getCurrentPosition() > 3500)
        {
            // SPECIMEN 2 - move slide to bar placement position and move toward bar via waypoint
            slideTarget = 1500;
            positionXTarget = -200;
            positionYTarget = 500;
            positionPrecise = true;
        }
        else if(stepsStarted[16] == true && steps[16] == false && armTarget < -9999 && positionXTarget < -9999)
        {
            //bot.sleep(200);
            steps[16] = true;
        }

        if(steps[16] == true && stepsStarted[17] == false)
        {
            // SPECIMEN 2 - move to bar placement position and move to bar
            positionXTarget = -200;
            positionYTarget = 620;
            positionPrecise = true;
            armTarget = 2700;
            stepsStarted[9] = true;
        }
        else if(stepsStarted[17] == true && steps[17] == false && armTarget < -9999)
        {
            steps[7] = true;
        }

//        if(steps[7] == true && stepsStarted[8] == false)
//        {
//            positionXTarget = -100;
//            positionYTarget = 700;
//            stepsStarted[8] = true;
//        }
//        else if(stepsStarted[8] == true && steps[8] == false && positionXTarget < -9999)
//        {
//            //bot.sleep(200);
//            steps[8] = true;
//        }


    }

}
