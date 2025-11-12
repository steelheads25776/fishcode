package org.firstinspires.ftc.teamcode.TeleOp;

import android.provider.FontRequest;
import android.transition.Slide;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.RTPAxon;
import org.firstinspires.ftc.teamcode.Robot;
import org.opencv.*;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.*;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class leteleop extends LinearOpMode
{
    GoBildaPinpointDriver odometry;
    Robot bot;
    OpenCvWebcam testCam = null;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorShooterRight, motorShooterLeft;
    IMU imu;
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    private RTPAxon testAxon;


    CRServo axon;
    CRServo servoLoaderStartLeft, servoLoaderStartRight, servoLoaderAssist;
    Servo servoLifter;
    AnalogInput axonEncoder;
    DcMotor intakeMotor;

    double driveStickLeftX;
    double driveStickLeftY;
    double driveStickRightX;

    boolean driveB;
    boolean driveA;//intake
    boolean driveY;
    boolean driveX;//reset orientation
    boolean driveBumperRight;
    boolean driveBumperLeft;
    double currentRotation;
    double driveTriggerLeft;//high speed on drive
    double driveTriggerRight;//slow speed on drive
    boolean driveDpadL;
    boolean driveDpadU;
    boolean driveDpadD;
    boolean driveDpadR;
    double axonMaxSpeed = 0.20;
    String rotateMode = "slow";

    double motorspeedhigh = 1.0;
    double motorspeed = 1.0;
    double motorspeednormal = 0.75;
    double motorspeedslower = 0.5;
    double slowRotateSpeed = 0.55;
    double robotRotateSpeedMultiplier = 2.0;
    double axonTargetPosition = 41;
    String axonDirection = "cw";
    String launchType = "";
    String launchStep = "None";

    boolean auxA, auxB, auxY, auxX;
    boolean auxDpadL, auxDpadR, auxDpadU, auxDpadD;
    boolean auxDpadLPressed, auxDpadRPressed, auxDpadUPressed, auxDpadDPressed;
    boolean auxBumperLeft, auxBumperRight;
    double auxStickLeftX, auxStickLeftY, auxStickRightX, auxStickRightY;
    double auxTriggerRight, auxTriggerLeft;
    boolean auxXPressed;
    boolean auxDpadPressed;
    boolean auxAPressed;
    boolean auxBPressed;
    boolean auxBumperLeftPressed;
    boolean auxBumperRightPressed;
    boolean auxYPressed;
    boolean launchEngaged = false;
    int loaderState = 0;
    int shooterState = 0;
    int intakeState = 0;
    int lifterServoState = 0;
    int magState = 0;
    boolean bumperRightPressed;
    final int initialTarget = 41;
    ElapsedTime testTimer;
    ElapsedTime stepTimer;
    boolean magazineEngaged = false;
    double maxCorrections = 3;
    int test = 1000;

    double lifterServoDown = 0.78;
    double lifterServoUp = 0.50;

    public void drive()
    {

        driveStickLeftX = gamepad1.left_stick_x * -1;
        driveStickLeftY = gamepad1.left_stick_y;
        driveStickRightX = gamepad1.right_stick_x * -1;
        driveTriggerLeft = gamepad1.left_trigger;
        driveTriggerRight = gamepad1.right_trigger;
        driveX = gamepad1.x;

        currentRotation = bot.getTeleOpIMURotation();//odometry.getHeading(AngleUnit.RADIANS);

        if(driveX)
        {
            bot.reset();
        }

        if (driveStickLeftX == 0.0 && driveStickLeftY == 0.0)
        {
            double rotationSpeed = slowRotateSpeed;
            if (driveTriggerLeft > 0.1)
            {
                rotationSpeed = motorspeedhigh;
            }
            else if (driveTriggerRight > 0.1)
            {
                rotationSpeed = motorspeedslower;
            }

            //rotateMode = "slow";
            motorFrontLeft.setPower((driveStickRightX * rotationSpeed));
            motorFrontRight.setPower((driveStickRightX * rotationSpeed) * -1);
            motorBackLeft.setPower((driveStickRightX * rotationSpeed));
            motorBackRight.setPower((driveStickRightX * rotationSpeed) * -1);
        }
        else
        {
            driveStickRightX *= robotRotateSpeedMultiplier;
            double rotationX = driveStickLeftX * Math.cos(-currentRotation) - driveStickLeftY * Math.sin(-currentRotation);
            double rotationY = driveStickLeftX * Math.sin(-currentRotation) + driveStickLeftY * Math.cos(-currentRotation);

            rotationX = rotationX * 1.1;

            double denominator = Math.max(Math.abs(rotationY) + Math.abs(rotationX) + Math.abs(driveStickRightX), 1);
            motorFrontLeft.setPower(((rotationY + rotationX + driveStickRightX) / denominator) * motorspeed);
            motorFrontRight.setPower(((rotationY - rotationX - driveStickRightX) / denominator) * motorspeed);
            motorBackLeft.setPower(((rotationY - rotationX + driveStickRightX) / denominator) * motorspeed);
            motorBackRight.setPower(((rotationY + rotationX - driveStickRightX) / denominator) * motorspeed);
        }

        if (driveTriggerLeft >= 0.1)
        {
            motorspeed = motorspeedhigh;

        }
        else if (driveTriggerRight >= 0.1)
        {
            motorspeed = motorspeedslower;

        }
        else
        {
            motorspeed = motorspeednormal;
        }
    }
    public void intake()
    {
        //leftStickY = gamepad1.left_stick_y * -1; // control intake motor value relative to left stick y
        auxA = gamepad2.a;
        auxB = gamepad2.b;
        //auxY = gamepad2.y;
        if(auxA)
        {
            auxAPressed = true;
        }
        if(auxB)
        {
            intakeMotor.setPower(-0.45);//-0.37
            auxBPressed = true;
        }
        if((!auxA && (auxAPressed || auxBPressed) && intakeState == 0) && !auxB)
        {
            intakeMotor.setPower(1.0);
            auxAPressed = false;
            auxBPressed = false;
            intakeState = 1;
        }
        else if((!auxA && (auxAPressed || auxBPressed) && intakeState == 1) && !auxB)
        {
            intakeMotor.setPower(0.0);
            auxAPressed = false;
            auxBPressed = false;
            intakeState = 0;
        }
        //intakeMotor.setPower(leftStickY);
    }

    public void magazine()
    {
        auxBumperLeft = gamepad2.left_bumper; //CCW rotation
        auxBumperRight = gamepad2.right_bumper; //CW rotation

        if(auxBumperLeft)
        {
            auxBumperLeftPressed = true;
        }
        if(auxBumperRight)
        {
            auxBumperRightPressed = true;
        }

        if(!auxBumperLeft && auxBumperLeftPressed)
        {
            //testAxon.changeTargetRotation(40);
            axonTargetPosition -= 120;
            if(axonTargetPosition < 0)
            {
                axonTargetPosition += 360;
            }
            else if(axonTargetPosition > 360)
            {
                axonTargetPosition -= 360;
            }
            axonDirection = "ccw";
            magazineEngaged = true;
            auxBumperLeftPressed = false;
        }

        if(!auxBumperRight && auxBumperRightPressed)
        {
            //testAxon.setTargetRotation(160);
            axonTargetPosition += 120;
            if(axonTargetPosition < 0)
            {
                axonTargetPosition += 360;
            }
            else if(axonTargetPosition > 360)
            {
                axonTargetPosition -= 360;
            }
            axonDirection = "cw";
            magazineEngaged = true;
            auxBumperRightPressed = false;
        }

    }

    public void launch()
    {
        auxDpadU = gamepad2.dpad_up;  // launch current
        auxDpadD = gamepad2.dpad_down;  // launch all

        if(!launchEngaged)
        {
            if (auxDpadU)
            {
                auxDpadUPressed = true;
            }
            if (auxDpadD)
            {
                auxDpadDPressed = true;
            }

            if (!auxDpadU && auxDpadUPressed)
            {
                launchEngaged = true;
                launchType = "current";
                auxDpadUPressed = false;
            }

            if (!auxDpadD && auxDpadDPressed)
            {
                launchEngaged = true;
                launchType = "all";
                auxDpadDPressed = false;
            }
        }

        if(launchEngaged)
        {
            if(launchStep.equalsIgnoreCase("none"))
            {
                magazineEngaged = true;
                launchStep = "a1 - initial position";
            }
            else if (launchStep.equalsIgnoreCase("a1 - initial position") && !magazineEngaged)
            {
                servoLoaderAssist.setPower(-1.0);
                servoLoaderStartRight.setPower(-1.0);
                servoLoaderStartLeft.setPower(1.0);
                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "a1 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a1 - lifter engaged") && stepTimer.milliseconds() > 500)
            {
                servoLifter.setPosition(lifterServoDown);
                motorShooterLeft.setPower(0.7);
                motorShooterRight.setPower(0.7);

                stepTimer.reset();
                launchStep = "a1 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a1 - drop lifter") && stepTimer.milliseconds() > 800)
            {
                axonTargetPosition += 120;
                if(axonTargetPosition < 0)
                {
                    axonTargetPosition += 360;
                }
                else if(axonTargetPosition > 360)
                {
                    axonTargetPosition -= 360;
                }
                axonDirection = "cw";
                magazineEngaged = true;
                launchStep = "a1 - get next artifact";
            }
            else if (launchStep.equalsIgnoreCase("a1 - get next artifact") && !magazineEngaged)
            {
                servoLoaderAssist.setPower(0.0);
                servoLoaderStartRight.setPower(0.0);
                servoLoaderStartLeft.setPower(0.0);
                motorShooterLeft.setPower(0.0);
                motorShooterRight.setPower(0.0);

                launchEngaged = false;
                launchStep = "none";
            }
        }
    }
    public void loading()
    {
        auxX = gamepad2.x;
        auxY = gamepad2.y;
        auxDpadL = gamepad2.dpad_left;
        auxDpadR = gamepad2.dpad_right;

        if(auxX)
        {
            auxXPressed = true;
        }

        if(auxXPressed && !auxX && loaderState == 0)
        {
            servoLoaderAssist.setPower(-1.0);
            servoLoaderStartRight.setPower(-1.0);
            servoLoaderStartLeft.setPower(1.0);
            auxXPressed = false;
            loaderState = 1;
        }
        else if(auxXPressed && !auxX && loaderState == 1)
        {
            servoLoaderAssist.setPower(0.0);
            servoLoaderStartRight.setPower(0.0);
            servoLoaderStartLeft.setPower(0.0);
            auxXPressed = false;
            loaderState = 0;
        }

        if(auxDpadL)
        {
            servoLifter.setPosition(0.50);

        }
        if(auxDpadR)
        {
            testTimer.reset();
            servoLifter.setPosition(0.78);
            //if(testTimer.milliseconds() > 500)
            //{
            //    servoLifter.setPosition(0.5);
            //}
        }
        /*

        if(auxY)
        {
            auxYPressed = true;
        }

        else if(!auxY && auxYPressed && lifterServoState == 0)
        {
            servoLifter.setPosition(0.20);
            auxYPressed = false;
            lifterServoState = 1;
        }

        else if(!auxY && auxYPressed && lifterServoState == 1)
        {
            servoLifter.setPosition(0.0);
            auxYPressed = false;
            lifterServoState = 0;
        }

         */
    }

    public void shooter()
    {
        auxDpadD = gamepad2.dpad_down;//75%
        //auxDpadU = gamepad1.dpad_up;//stop motors

        if(auxDpadD)
        {
            auxDpadPressed = true;
        }

        if(!auxDpadD && auxDpadPressed && shooterState == 0)
        {
            motorShooterLeft.setPower(0.75);
            motorShooterRight.setPower(0.75);
            //motorShooterLeft.setVelocity(1999);
            //motorShooterRight.setVelocity(1999);
            auxDpadPressed = false;
            shooterState = 1;
        }
        if(!auxDpadD && auxDpadPressed && shooterState == 1)
        {
            motorShooterLeft.setPower(0.0);
            motorShooterRight.setPower(0.0);
            //motorShooterLeft.setVelocity(1999);
            //motorShooterRight.setVelocity(1999);
            auxDpadPressed = false;
            shooterState = 0;

        }
    }
    public void showTelemetry()
    {
        telemetry.addData("launch step", launchStep);
        telemetry.addData("odometry Y", bot.getY());
        telemetry.addData("odometry x", bot.getX());
        telemetry.addData("odometry", odometry.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Left stick y", driveStickLeftY);
        telemetry.addData("axon encoder", ((axonEncoder.getVoltage() / 3.3) * 360));
        telemetry.addData("axon target", axonTargetPosition);
        telemetry.addData("rotation direction", axonDirection);
        telemetry.addData("axon direction", axon.getDirection());
        //telemetry.addData("test target", testAxon.getTargetRotation());
        //telemetry.addData("test angle", testAxon.log());
        //telemetry.addData("difference", ((axonEncoder.getVoltage() / 3.3) * 360) - testAxon.getCurrentAngle());
        telemetry.addData("engaged", magazineEngaged);
        telemetry.addData("intake power", intakeMotor.getPower());
        telemetry.addData("magazine power", axon.getPower());
        telemetry.addData("motor velocity left", motorShooterLeft.getVelocity());
        telemetry.addData("motor velocity right", motorShooterRight.getVelocity());

        telemetry.update();
    }
    public void initialize2()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);



        servoLoaderStartLeft = hardwareMap.get(CRServo.class, "StartLeft");
        servoLoaderStartRight = hardwareMap.get(CRServo.class, "StartRight");
        servoLoaderAssist = hardwareMap.get(CRServo.class, "LoaderAssist");
        servoLifter = hardwareMap.get(Servo.class, "lifter");

        axon = hardwareMap.get(CRServo.class, "axon");
        axonEncoder = hardwareMap.get(AnalogInput.class, "axonEncoder");
        testAxon = new RTPAxon(axon, axonEncoder);
        testAxon.setMaxPower(0.1);
        testAxon.setPidCoeffs(0.02, 0.0005, 0.0025);

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        //axon = hardwareMap.get(CRServo.class, "Magazine");

        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        motorShooterLeft = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        motorShooterRight = hardwareMap.get(DcMotorEx.class, "RightShooter");

        motorShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorShooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testTimer = new ElapsedTime();
        stepTimer = new ElapsedTime();

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        odometry.setOffsets(-85, -150); // don't know how to get these offsets yet
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.resetPosAndIMU();

        bot = new Robot(odometry);
        bot.setMotors(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);
        bot.setSpeed(0.4);
        bot.setOffsetIMU(imu);
        bot.getIMUOffset();
        bot.reset();


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Internal Cam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());//not sure what this does or how it works...
        testCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //testCam.setPipeline(new searchColor());
        testCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                testCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                //do nothing (yet?)
            }
        });
        //axonTargetPosition = -80;
        //testAxon.setRtp(true);

        servoLifter.setPosition(lifterServoDown);

        waitForStart();
        bot.getIMUOffset();

    }
    private void axonToPosition(double target, String direction)
    {
        double powerRotateCWMax = -0.11;
        double powerRotateCWSlow = -0.05;
        double powerRotateCCWMax = -0.14;
        double powerRotateCCWSlow = -0.07;

        double powerRotateMax = -0.1;
        double powerRotateSlow = -0.05;

        double currentPos = (axonEncoder.getVoltage() / 3.3) * 360;
        double distanceToTarget = currentPos - target;



        if(Math.abs(distanceToTarget) >= 30)
        {
            if(direction.equalsIgnoreCase("cw"))
            {
                axon.setPower(powerRotateCWMax);
                axon.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            else
            {
                axon.setPower(powerRotateCCWMax);
                axon.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }
        else if(distanceToTarget < -5)
        {
            axon.setPower(powerRotateCWSlow);
            axon.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (distanceToTarget > 5)
        {
            axon.setPower(powerRotateCCWSlow);
            axon.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if(distanceToTarget < -2)
        {
            axon.setPower(powerRotateCWSlow);
            axon.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (distanceToTarget > 2)
        {
            axon.setPower(powerRotateCCWSlow);
            axon.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            axon.setPower(0.0);
            sleep(100);
            currentPos = (axonEncoder.getVoltage() / 3.3) * 360;
            if(Math.abs(currentPos - target) < 2)
            {
                magazineEngaged = false;
            }
        }

    }

    @Override
    public void runOpMode()
    {
        initialize2();
        magazineEngaged = true;
        axonDirection = "cw";
        while(opModeIsActive())
        {
            if(!launchEngaged)
            {
                drive();
                magazine();
                intake();
                //shooter();
                //loading();
            }
            launch();
            showTelemetry();

            if(magazineEngaged)
            {
                axonToPosition(axonTargetPosition, axonDirection);
            }


            //testAxon.update();
            odometry.update();
        }
    }
}
