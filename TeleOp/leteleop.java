package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.easyopencv.*;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class leteleop extends LinearOpMode
{
    double shootingPower = 0.62;//0.75
    double powerRotateCWMax = -0.11;
    double powerRotateCWSlow = -0.05;
    double powerRotateCCWMax = -0.14;
    double powerRotateCCWSlow = -0.07;
    GoBildaPinpointDriver odometry;
    Robot bot;
    OpenCvWebcam testCam = null;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorShooterRight, motorShooterLeft;
    //IMU imu;
    //IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));



    CRServo axon;
    CRServo servoLoaderStartLeft, servoLoaderStartRight, servoLoaderAssist;
    Servo servoLifter;
    AnalogInput axonEncoder;
    CRServo internalLight;
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
    double currentOrientation;
    double currentOrientationRad;
    double axonPreviousPosition = -1000;
    int axonFrozen = 0;
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
    double axonTargetPosition = 43;
    String axonDirection = "cw";
    String launchType = "";
    String launchStep = "None";

    boolean auxA, auxB, auxY, auxX;
    boolean auxDpadL, auxDpadR, auxDpadU, auxDpadD;
    boolean auxDpadLPressed, auxDpadRPressed, auxDpadUPressed, auxDpadDPressed;
    boolean auxBumperLeft, auxBumperRight;
    double auxStickLeftX, auxStickLeftY, auxStickRightX, auxStickRightY;
    double auxTriggerRight, auxTriggerLeft;
    boolean auxTriggerRightPressed, auxTriggerLeftPressed;
    boolean auxXPressed;
    boolean auxYPressed;
    boolean auxDpadPressed;
    boolean auxAPressed;
    boolean auxBPressed;
    boolean auxBumperLeftPressed;
    boolean auxBumperRightPressed;
    boolean launchEngaged = false;
    int loaderState = 0;
    int shooterState = 0;
    int intakeState = 0;
    int lifterServoState = 0;
    int magState = 0;
    boolean bumperRightPressed;
    final int initialTarget = 43;
    ElapsedTime testTimer;
    ElapsedTime stepTimer;
    ElapsedTime axonTimer;
    boolean magazineEngaged = false;
    double maxCorrections = 3;
    int test = 1000;

    double lifterServoDown = 0.78;
    double lifterServoUp = 0.50;

    double rotationX = 0;
    double rotationY = 0;

    int velocityShooting1 = 1700;
    int velocityShooting2 = 1800;
    int velocityShooting3 = 1900;
    public void drive()
    {

        // Game controller left stick provide negative values for up and left and positive for down and right
        // Multiply by  left stick up value by -1.0 to make is so up and right are positive and down and left are negative
        driveStickLeftX = gamepad1.left_stick_x;
        driveStickLeftY = gamepad1.left_stick_y * -1.0;

        driveStickRightX = gamepad1.right_stick_x;

        driveTriggerLeft = gamepad1.left_trigger;
        driveTriggerRight = gamepad1.right_trigger;
        driveX = gamepad1.x;

        currentOrientationRad = odometry.getHeading(AngleUnit.RADIANS);

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
            rotationX = driveStickLeftX * Math.cos(currentOrientationRad) - driveStickLeftY * Math.sin(currentOrientationRad);
            rotationY = driveStickLeftX * Math.sin(currentOrientationRad) + driveStickLeftY * Math.cos(currentOrientationRad);

            //rotationX = rotationX * 1.1;

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
            auxBPressed = false;
        }
        else if(auxB)
        {
            // reverse intake while B pressed
            intakeMotor.setPower(-0.45);//-0.37
            //internalLight.setPower(.5);;
            auxBPressed = true;
            auxAPressed = false;
        }
        else if(auxBPressed)
        {
            auxBPressed = false;
        }
        else if(intakeState == 1)
        {
            if(!auxAPressed)
            {
                intakeMotor.setPower(1.0);
            }
            else
            {
                intakeMotor.setPower(0.0);
                auxAPressed = false;
                intakeState = 0;
            }
        }
        else if(intakeState == 0)
        {
            if(!auxAPressed)
            {
                intakeMotor.setPower(0.0);
            }
            else
            {
                intakeMotor.setPower(1.0);
                auxAPressed = false;
                intakeState = 1;
            }
        }
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

    public void fixShooter()
    {
        auxY = gamepad2.y;
        if(auxY)
        {
            auxYPressed = true;
        }
        if(auxY && auxYPressed)
        {
            launchEngaged = false;
            launchStep = "none";

            servoLoaderAssist.setPower(1.0);
            servoLoaderStartRight.setPower(1.0);
            servoLoaderStartLeft.setPower(-1.0);
            motorShooterRight.setVelocity(velocityShooting1);
            motorShooterLeft.setVelocity(velocityShooting1);
        }
        else if(!auxY && auxYPressed)
        {

            servoLoaderAssist.setPower(0.0);
            servoLoaderStartRight.setPower(0.0);
            servoLoaderStartLeft.setPower(0.0);
            motorShooterRight.setVelocity(0);
            motorShooterLeft.setVelocity(0);
            auxYPressed = false;
        }
    }

    public void fixMagazine()
    {
        auxDpadL = gamepad2.dpad_left;//disengage
        auxDpadU = gamepad2.dpad_up;//up power value
        auxDpadD = gamepad2.dpad_down;//lower power value

        if(auxDpadL)
        {
            auxDpadLPressed = true;
        }
        if(auxDpadU)
        {
            auxDpadUPressed = true;
        }
        if(auxDpadD)
        {
            auxDpadDPressed = true;
        }


        if(!auxDpadL && auxDpadLPressed)
        {
            magazineEngaged = false;
            auxDpadLPressed = false;
        }
        if(!auxDpadU && auxDpadUPressed)
        {
            powerRotateCWMax -= 0.01;
            powerRotateCWSlow -= 0.01;
            powerRotateCCWMax -= 0.01;
            powerRotateCCWSlow -= 0.01;

            auxDpadUPressed = false;
        }
        if(!auxDpadD && auxDpadDPressed)
        {
            powerRotateCWMax += 0.01;
            powerRotateCWSlow += 0.01;
            powerRotateCCWMax += 0.01;
            powerRotateCCWSlow += 0.01;

            auxDpadDPressed = false;
        }
    }

    public void launch()
    {
        auxTriggerLeft = gamepad2.left_trigger;  // launch current
        auxTriggerRight = gamepad2.right_trigger;  // launch all

        if(!launchEngaged)
        {
            if (auxTriggerRight >= 0.3)
            {
                auxTriggerRightPressed = true;
            }
            if (auxTriggerLeft >= 0.3)
            {
                auxTriggerLeftPressed = true;
            }

            if (auxTriggerLeft < 0.3 && auxTriggerLeftPressed)
            {
                launchEngaged = true;
                launchType = "current";
                auxTriggerLeftPressed = false;
            }

            if (auxTriggerRight < 0.3 && auxTriggerRightPressed)
            {
                launchEngaged = true;
                launchType = "all";
                auxTriggerRightPressed = false;
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
                intakeMotor.setPower(0.0);

                servoLoaderAssist.setPower(1.0);
                servoLoaderStartRight.setPower(1.0);
                servoLoaderStartLeft.setPower(-1.0);
                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "a1 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a1 - lifter engaged") && stepTimer.milliseconds() > 300)
            {
                servoLifter.setPosition(lifterServoDown);
                motorShooterLeft.setVelocity(velocityShooting1);
                motorShooterRight.setVelocity(velocityShooting1);

                stepTimer.reset();
                launchStep = "a1 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a1 - drop lifter") && stepTimer.milliseconds() > 350)
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
                launchStep = "a2 - get next artifact";
                stepTimer.reset();
            }
            else if (launchStep.equalsIgnoreCase("a2 - get next artifact") && !magazineEngaged)
            {
                if(launchType.equalsIgnoreCase("current"))
                {
                    if(stepTimer.milliseconds() > 500)
                    {
                        servoLoaderAssist.setPower(0.0);
                        servoLoaderStartRight.setPower(0.0);
                        servoLoaderStartLeft.setPower(0.0);
                        motorShooterLeft.setVelocity(0);
                        motorShooterRight.setVelocity(0);

                        launchEngaged = false;
                        launchStep = "none";
                    }
                }
                else if(launchType.equalsIgnoreCase("all"))
                {
                    launchStep = "a2 - initial position";
                }
            }
            else if (launchStep.equalsIgnoreCase("a2 - initial position") && !magazineEngaged)
            {
                /*
                servoLoaderAssist.setPower(-1.0);
                servoLoaderStartRight.setPower(-1.0);
                servoLoaderStartLeft.setPower(1.0);
                 */
                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "a2 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a2 - lifter engaged") && stepTimer.milliseconds() > 300)
            {
                servoLifter.setPosition(lifterServoDown);
                motorShooterLeft.setVelocity(velocityShooting2);
                motorShooterRight.setVelocity(velocityShooting2);

                stepTimer.reset();
                launchStep = "a2 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a2 - drop lifter") && stepTimer.milliseconds() > 350)
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
                launchStep = "a3 - get next artifact";
            }
            else if (launchStep.equalsIgnoreCase("a3 - get next artifact") && !magazineEngaged)
            {
                launchStep = "a3 - initial position";
            }
            else if (launchStep.equalsIgnoreCase("a3 - initial position") && !magazineEngaged)
            {
                /*
                servoLoaderAssist.setPower(-1.0);
                servoLoaderStartRight.setPower(-1.0);
                servoLoaderStartLeft.setPower(1.0);
                 */
                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "a3 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a3 - lifter engaged") && stepTimer.milliseconds() > 350)
            {
                servoLifter.setPosition(lifterServoDown);
                motorShooterLeft.setVelocity(velocityShooting3);
                motorShooterRight.setVelocity(velocityShooting3);

                stepTimer.reset();
                launchStep = "a3 - turn off launcher";
            }
            else if (launchStep.equalsIgnoreCase("a3 - turn off launcher") && stepTimer.milliseconds() > 1000)
            {
                servoLoaderAssist.setPower(0.0);
                servoLoaderStartRight.setPower(0.0);
                servoLoaderStartLeft.setPower(0.0);
                motorShooterLeft.setVelocity(0);
                motorShooterRight.setVelocity(0);

                launchEngaged = false;
                launchStep = "none";
            }
        }
    }
    public void showTelemetry()
    {
        telemetry.addData("Front Left Power", motorFrontLeft.getPower());
        telemetry.addData("CCW max", powerRotateCCWMax);
        telemetry.addData("CCW slow", powerRotateCCWSlow);

        telemetry.addData("Rotation X", rotationX);
        telemetry.addData("Rotation Y", rotationY);

        telemetry.addData("launch step", launchStep);
        telemetry.addData("odometry Y", bot.getY());
        telemetry.addData("odometry x", bot.getX());
        telemetry.addData("odometry", bot.getOrientationCurrent());
        telemetry.addData("drive Left stick y", driveStickLeftY);
        telemetry.addData("drive Left stick x", driveStickLeftX);
        telemetry.addData("drive Right stick x", driveStickRightX);

        telemetry.addData("aux stick right Y", auxStickRightY);
        telemetry.addData("aux stick right X", auxStickRightX);
        telemetry.addData("aux stick left X", auxStickLeftX);
        telemetry.addData("aux stick left Y", auxStickLeftY);


        telemetry.addData("axon encoder", ((axonEncoder.getVoltage() / 3.3) * 360));
        //telemetry.addData("axon target", axonTargetPosition);
        //telemetry.addData("rotation direction", axonDirection);
        //telemetry.addData("axon direction", axon.getDirection());
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
        //imu = hardwareMap.get(IMU.class, "imu");
        //imu.initialize(parameters);

        servoLoaderStartLeft = hardwareMap.get(CRServo.class, "StartLeft");
        servoLoaderStartRight = hardwareMap.get(CRServo.class, "StartRight");
        servoLoaderAssist = hardwareMap.get(CRServo.class, "LoaderAssist");
        servoLifter = hardwareMap.get(Servo.class, "lifter");
        internalLight = hardwareMap.get(CRServo.class, "InternalLight");

        axon = hardwareMap.get(CRServo.class, "axon");
        axonEncoder = hardwareMap.get(AnalogInput.class, "axonEncoder");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        //axon = hardwareMap.get(CRServo.class, "Magazine");

        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorShooterLeft = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        motorShooterRight = hardwareMap.get(DcMotorEx.class, "RightShooter");

        motorShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorShooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testTimer = new ElapsedTime();
        stepTimer = new ElapsedTime();
        axonTimer = new ElapsedTime();

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        odometry.setOffsets(172, -74, DistanceUnit.MM); // don't know how to get these offsets yet
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odometry.setYawScalar(-1.0);
        odometry.resetPosAndIMU();

        bot = new Robot(odometry);
        bot.setMotors(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);
        bot.setSpeed(0.4);
        //bot.setOffsetIMU(imu);
        //bot.getIMUOffset();
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
        internalLight.setPower(-1.0);//0.5

        servoLifter.setPosition(lifterServoDown);

        waitForStart();
        //bot.getIMUOffset();

    }
    private void axonToPosition(double target, String direction)
    {
        powerRotateCWMax = -0.13;
        powerRotateCWSlow = -0.05;
        powerRotateCCWMax = -0.16;
        powerRotateCCWSlow = -0.07;

        double currentPos = (axonEncoder.getVoltage() / 3.3) * 360;
        if(Math.abs(axonPreviousPosition - currentPos) < .5)
        {
            axonFrozen++;
        }
        else
        {
            axonFrozen = 0;
        }
        if (axonFrozen >= 3)
        {
            powerRotateCWMax = powerRotateCWMax - (.01 * (axonFrozen - 2));
            powerRotateCWSlow = powerRotateCWSlow - (.01 * (axonFrozen - 2));
            powerRotateCCWMax = powerRotateCCWMax - (.01 * (axonFrozen - 2));
            powerRotateCCWSlow = powerRotateCCWSlow - (.01 * (axonFrozen - 2));
        }
        axonPreviousPosition = currentPos;
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
        else if(distanceToTarget < -8)
        {
            axon.setPower(powerRotateCWSlow);
            axon.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (distanceToTarget > 8)
        {
            axon.setPower(powerRotateCCWSlow);
            axon.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if(distanceToTarget < -3)
        {
            axon.setPower(powerRotateCWSlow);
            axon.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (distanceToTarget > 3)
        {
            axon.setPower(powerRotateCCWSlow);
            axon.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            axon.setPower(0.0);
            sleep(100);
            currentPos = (axonEncoder.getVoltage() / 3.3) * 360;
            if(Math.abs(currentPos - target) < 3)
            {
                magazineEngaged = false;
                axonPreviousPosition = -1000;
            }
        }
    }

    @Override
    public void runOpMode()
    {
        initialize2();
        magazineEngaged = true;
        axonDirection = "cw";

        int axonState = 0;
        while(opModeIsActive())
        {
            if(axonState < 20)
            {
                servoLoaderAssist.setPower(1.0);
                servoLoaderStartRight.setPower(1.0);
                servoLoaderStartLeft.setPower(-1.0);
                axonState += 1;
            }
            else if(axonState == 20)
            {
                servoLoaderAssist.setPower(0.0);
                servoLoaderStartRight.setPower(0.0);
                servoLoaderStartLeft.setPower(0.0);
                axonState = 200;
            }

            fixShooter();
            //fixMagazine();
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
