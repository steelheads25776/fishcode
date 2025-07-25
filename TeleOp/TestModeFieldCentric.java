package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.Convert;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp
public class TestModeFieldCentric extends LinearOpMode
{
    GoBildaPinpointDriver odometry;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorSlideLeft, motorSlideRight;
    DcMotorEx motorClawArm;
    DcMotorEx motorTilter;
    VoltageSensor sensorVoltage;
    Servo servoClawGrabber, servoClawExtend;
    Robot bot;
    TouchSensor buttonArmStop;
    IMU imu;
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));



    double motorspeed = 1.0;
    double rotateSpeed = 1.00;
    double slowRotateSpeed = 0.55;
    double motorspeedhigh = 1.0;
    double motorspeednormal = 0.65;
    double motorspeedslower = 0.25;

    float driveStickLeftX;
    float driveStickLeftY;
    float driveStickRightX;
    float driveStickRightY;

    float driveTriggerLeft;
    float driveTriggerRight;

    boolean driveButtonA;
    boolean driveButtonX;
    boolean driveButtonB;
    boolean driveButtonY;
    boolean driveDpadL, driveDpadU, driveDpadD, driveDpadR;

    boolean armButtonA;
    boolean armLeftBumper;
    boolean armRightBumper;
    boolean armButtonX;
    boolean armButtonB;
    boolean armButtonY;

    double armRightStickY;
    double armRightStickX;
    double armLeftStickY;
    float armTriggerLeft;
    float armTriggerRight;

    boolean armDpadUp, armDpadDown, armDpadLeft, armDpadRight;

    double armExtendedPosition = 0.42;//a
    double armRetractedPosition = 0.0;//x -0.55

    double servoPosition = 0.0;

    double clawOpenMaxPosition = 0.7;
    double clawOpenPosition = 0.4;//left bumper 0.5
    double clawClosedPosition = 0.0;//right bumper

    boolean clawExtended = false;

    double armRotateSpeed = 0.72; // 0.72
    double slidePower = 1.0;

    double orientationCurrent = 0.0; // in degrees - value will be 0 - 359.99
    double orientationTarget = -1.0;  // -1.0 means no rotation
    double distanceToTargetOrientation = 0.0;
    String rotationDirection = "none";
    double rotationErrorTolerance = 1.0;
    double positionOrientationTarget = 0.0;
    double positionXTarget = -10000.0;
    double positionYTarget = -10000.0;
    //boolean positionPrecise = true;
    double distanceToTargetPosition = 0.0;
    double[] rotateReturn = new double[2];
    double[] positionReturn = new double[2];
    double slideTarget = -1;
    String slideDirection = "up";
    double armTarget = -10000;
    String armDirection = "down";

    String step = "none";
    boolean hangStarted = false;
    boolean noSlow = false;

    double brakePower = -0.10;
    int brakeActivePosition = 100;
    int slideSlowZone = 100;

    int slidePosition = 0;
    int slideHangStart = 0;
    int armHangStart = 0;

    String rotateMode = "slow";

    ElapsedTime timer = new ElapsedTime();

    int armMaximumPosition = 1000;

    double armUpwardsPower = 1.0;
    double armDownwardsPower = 0.5;
    ElapsedTime timerClaw = new ElapsedTime();
    ElapsedTime timerArm = new ElapsedTime();

    boolean isBumperCurrentlyPressed = false;
    boolean armButtonBCurrentlyPressed = false;

    double currentRotation;

    double robotRotateSpeedMultiplier = 2.0;

    public void robotHang()
    {
        if(step.equalsIgnoreCase("start hang"))
        {
            motorTilter.setPower(1.0);
            motorFrontLeft.setPower(-0.3);
            motorFrontRight.setPower(-0.3);
            //bot.sleep(1000);
            sleep(1000);
            motorTilter.setPower(0.0);

            motorFrontLeft.setPower(0.0);
            motorFrontRight.setPower(0.0);
            motorBackLeft.setPower(0.0);
            motorBackRight.setPower(0.0);
            slideTarget = -1000;
            timer.reset();
            step = "pause before 1st lift";
        }
        else if(step.equalsIgnoreCase("pause before 1st lift"))
        {
            if(timer.milliseconds() > 500)
            {
                motorTilter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                step = "lift to 1st bar";
                timer.reset();
                slideHangStart = motorSlideLeft.getCurrentPosition();
                armHangStart = motorClawArm.getCurrentPosition();
                slideTarget = slideHangStart - 735;
                bot.slidePositionHold = slideTarget;

                bot.slidePositionPrevious = slideHangStart;
                bot.brakePower = 0.4;
                bot.slidePowerHang = 0.55;
                slideDirection = "down";
            }
        }
        else if(step.equalsIgnoreCase("lift to 1st bar"))
        {
            if (slideTarget < -999)
            {
                timer.reset();
                motorTilter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                step = "pause before move to 2nd bar";
            }
        }
        else if (step.equalsIgnoreCase("pause before move to 2nd bar"))
        {
            if(timer.milliseconds() > 1000)
            {
                step = "upper hooks to 2nd bar";
                slideTarget = slideHangStart + 400;
                bot.slidePositionHold = slideTarget;
                bot.brakePower = 0.25;
                bot.slidePowerHang = -0.2;
                slideDirection = "up";
                timer.reset();
            }
        }
        else if (step.equalsIgnoreCase("upper hooks to 2nd bar"))
        {
            if(timer.milliseconds() < 500)
            {
                motorTilter.setPower(-0.9);
            }
            else
            {
                motorTilter.setPower(0.0);
            }
            if(slideTarget < -999 && timer.milliseconds() > 500)
            {
                step = "bring up tilter";
                timer.reset();
            }
        }
        else if (step.equalsIgnoreCase("bring up tilter"))
        {
            step = "lift to 2nd bar";
            motorTilter.setPower(0.0);

            slideHangStart = motorSlideLeft.getCurrentPosition();
            slideTarget = slideHangStart - 2400;
            bot.slidePositionHold = slideTarget;
            bot.brakePower = 0.4;
            bot.slidePowerHang = 0.65;
            slideDirection = "down";

        }
        else if (step.equalsIgnoreCase("lift to 2nd bar"))
        {
            if (slideTarget < -999)
            {
                //bot.brakePower = 0.0;
                step = "celebrate";
                timer.reset();
            }
        }
        else if (step.equalsIgnoreCase("celebrate"))
        {
            if(timer.milliseconds() < 500)
            {

            }
        }
    }

    public void driverButtons()
    {
        driveDpadL = gamepad1.dpad_left;
        driveDpadR = gamepad1.dpad_right;
        driveDpadU = gamepad1.dpad_up;
        driveDpadD = gamepad1.dpad_down;
        driveButtonA = gamepad1.a;
        driveButtonX = gamepad1.x;
        driveButtonB = gamepad1.b;
        driveButtonY = gamepad1.y;

        if(driveDpadU)
        {
            armTarget = 750;
        }


        else if (driveButtonY && step == "none")
        {
            slideTarget = -1000;
            step = "start hang";
            hangStarted = true;
        }
        else if (driveButtonB)
        {
            hangStarted = false;
            noSlow = false;
            //bot.reset();
        }
        else if (driveButtonX)
        {
            bot.resetSlideAndArm();
        }
        else if (driveButtonA)
        {
            bot.reset();//imu.resetYaw();
        }


    }

    public void drive()
    {

        driveStickLeftX = gamepad1.left_stick_x * -1;
        driveStickLeftY = gamepad1.left_stick_y;
        driveStickRightX = gamepad1.right_stick_x * -1;
        driveStickRightY = gamepad1.right_stick_y;

        driveTriggerLeft = gamepad1.left_trigger;
        driveTriggerRight = gamepad1.right_trigger;

        currentRotation = bot.getTeleOpIMURotation();//imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if(armDpadDown || armDpadLeft || armDpadRight || armDpadUp || armRightStickX > 0.1 || armRightStickX < -0.1)
        {
            //don't move when arm dpad is pressed
        }
        else if(orientationTarget >= 0)
        {
            //don't move robot is rotating
        }
        else
        {

            if(driveStickLeftX == 0.0 && driveStickLeftY == 0.0)
            {
                double rotationSpeed = slowRotateSpeed;
                if(driveTriggerLeft > 0.1)
                {
                    rotationSpeed = motorspeedhigh;
                }
                else if(driveTriggerRight > 0.1)
                {
                    rotationSpeed = motorspeedslower;
                }

                //rotateMode = "slow";
                motorFrontLeft.setPower((driveStickRightX * rotationSpeed));
                motorFrontRight.setPower((driveStickRightX * rotationSpeed) * -1 );
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
        }

        if(driveTriggerLeft >= 0.1)
        {
            motorspeed = motorspeedhigh;

        }
        else if(driveTriggerRight >= 0.1)
        {
            motorspeed = motorspeedslower;

        }

        else
        {
            motorspeed = motorspeednormal;
        }
    }

    public void armExtend()
    {
        armButtonA = gamepad2.a;//extend
        armButtonX = gamepad2.x;//retract

        armTriggerLeft = gamepad2.left_trigger;//retract
        armTriggerRight = gamepad2.right_trigger;//extend

        if(armTriggerRight > 0.1)
        {
            servoPosition = servoPosition + (armTriggerRight * 0.03);//servoPosition + 0.02

            if(servoPosition > armExtendedPosition)
            {
                servoPosition = armExtendedPosition;
            }

            servoClawExtend.setPosition(servoPosition);
        }

        if(armTriggerLeft > 0.1)
        {
            servoPosition = servoPosition - (armTriggerLeft * 0.03);//servoPosition - 0.02

            if(servoPosition < armRetractedPosition)
            {
                servoPosition = armRetractedPosition;
            }
            servoClawExtend.setPosition(servoPosition);
        }


        if(armButtonX)
        {
            if(clawExtended)
            {
                servoClawExtend.setPosition(armRetractedPosition);
                servoPosition = armRetractedPosition;
                clawExtended = false;
            }
        }

        if(armButtonA)
        {
            if(!clawExtended)
            {
                servoClawExtend.setPosition(armExtendedPosition);
                servoPosition = armExtendedPosition;
                clawExtended = true;
            }
        }
    }

    public void armRotate()
    {
        armRightStickY = gamepad2.right_stick_y;
        /*
        if(motorSlideLeft.getCurrentPosition() < 650)// if slide is less than 650 ticks, don't allow arm to go past 1000 ticks
        {
            armMaximumPosition = 1000;
        }

        else if(motorSlideLeft.getCurrentPosition() > 650)// if slide is greater than 650 ticks, don't allow arm to go past 2000 ticks
        {
            armMaximumPosition = 2000;
        }



        if(motorClawArm.getCurrentPosition() >= 1000) //change speed values based on arm position
        {
            armUpwardsPower = 0.50;
            armDownwardsPower = 1.00;
        }
        else if(motorClawArm.getCurrentPosition() < 1000) //change speed values based on arm position
        {
            armUpwardsPower = 1.00;
            armDownwardsPower = 0.50;
        }
        */
        if(armRightStickY == 0 && armTarget > -10000)
        {
            // do nothing - arm is moving
        }
        else
        {   //slide 650 arm 2000
            //slide 0 arm 1000
            armTarget = -10000;
            if(motorClawArm.getCurrentPosition() >= armMaximumPosition)//disable upwards movement when at maximum position
            {
                if(armRightStickY > 0)//down
                {
                    motorClawArm.setPower(((armRightStickY * armRotateSpeed) * -1) * armDownwardsPower);
                }
                else
                {
                    motorClawArm.setPower(0.0);
                }
            }

            else if(buttonArmStop.isPressed())//disable downwards movement if button is pressed
            {
                if(armRightStickY < 0)//up
                {
                    motorClawArm.setPower(((armRightStickY * armRotateSpeed) * -1) * armUpwardsPower);
                }
                else
                {
                    motorClawArm.setPower(0.0);
                }
            }

            else//give full control if neither at max position or pressing button
            {
                if(armRightStickY > 0)//down
                {
                    motorClawArm.setPower(((armRightStickY * armRotateSpeed) * -1) * armDownwardsPower);
                }
                else//up
                {
                    motorClawArm.setPower(((armRightStickY * armRotateSpeed) * -1) * armUpwardsPower);
                }
            }
        }
    }

    public void armButtons()
    {
        armDpadUp = gamepad2.dpad_up; // drive forward
        armDpadDown = gamepad2.dpad_down; // drive backward

        armDpadLeft = gamepad2.dpad_left; // drive left
        armDpadRight = gamepad2.dpad_right; // drive right

        armRightStickX = gamepad2.right_stick_x; // rotate robot

        driveButtonY = gamepad2.y;// slow slides

        armRightBumper = gamepad2.right_bumper;//close claw
        armLeftBumper = gamepad2.left_bumper;//open claw

        armButtonB = gamepad2.b;

        if(driveButtonY)//slow slide when y button is true
        {
            slidePower = 0.15;
        }
        else if(!driveButtonY)// run at max speed when y button is false
        {
            slidePower = 1.0;
        }

        if(armRightStickX > 0.1 || armRightStickX < -0.1) //rotate robot, and disable driver control
        {
            motorFrontLeft.setPower((armRightStickX * motorspeedslower) * -1);
            motorFrontRight.setPower((armRightStickX * motorspeedslower) );
            motorBackLeft.setPower((armRightStickX * motorspeedslower) * -1);
            motorBackRight.setPower((armRightStickX * motorspeedslower));
        }


        if(armLeftBumper)//open claw
        {
            if(!isBumperCurrentlyPressed)
            {
                isBumperCurrentlyPressed = true;
                timerClaw.reset();
            }
            servoClawGrabber.setPosition(clawOpenPosition);
        }
        else if(!armLeftBumper)
        {
            isBumperCurrentlyPressed = false;
        }

        if(isBumperCurrentlyPressed && timerClaw.milliseconds() > 200)
        {//if left bumper has been pressed for more than 200 milliseconds
            servoClawGrabber.setPosition(clawOpenMaxPosition);
        }

        if(armRightBumper)//close claw
        {
            servoClawGrabber.setPosition(clawClosedPosition);
        }

        if(armButtonB)//raise arm to grab from middle
        {
            timerArm.reset();
            if(!armButtonBCurrentlyPressed)
            {
                armButtonBCurrentlyPressed = true;
            }
        }
        else if(!armButtonB)
        {
            armButtonBCurrentlyPressed = false;
        }

        if(armButtonBCurrentlyPressed)
        {
            if(timerArm.milliseconds() < 200)
            {
                motorClawArm.setPower(0.6);
            }
            else
            {
                motorClawArm.setPower(0.0);
            }
        }

        if(armDpadLeft)//strafe robot left, and disable driver control
        {
            motorFrontLeft.setPower((rotateSpeed * 0.5) * 1);
            motorFrontRight.setPower((rotateSpeed * 0.5) * -1);
            motorBackLeft.setPower((rotateSpeed * 0.5) * -1);
            motorBackRight.setPower((rotateSpeed * 0.5) * 1);
        }

        if(armDpadRight)//strafe robot right, and disable driver control
        {
            motorFrontLeft.setPower((rotateSpeed * 0.5) * -1);
            motorFrontRight.setPower((rotateSpeed * 0.5) * 1);
            motorBackLeft.setPower((rotateSpeed * 0.5) * 1);
            motorBackRight.setPower((rotateSpeed * 0.5) * -1);
        }

        if(armDpadUp)//drive robot forward, and disable driver control
        {
            motorFrontLeft.setPower(-0.2);
            motorFrontRight.setPower(-0.2);
            motorBackLeft.setPower(-0.2);
            motorBackRight.setPower(-0.2);
        }

        if(armDpadDown)//drive robot backward, and disable driver control
        {
            motorFrontLeft.setPower(0.2);
            motorFrontRight.setPower(0.2);
            motorBackLeft.setPower(0.2);
            motorBackRight.setPower(0.2);
        }

    }

    public void slides()
    {
        armLeftStickY = gamepad2.left_stick_y;

        if(armLeftStickY == 0 && slideTarget > -1)
        {
            // do nothing - slide is moving
        }
        else
        {
            slideTarget = -2;
            if (motorSlideLeft.getCurrentPosition() < slideSlowZone)
            {
                if (armLeftStickY != 0.0)
                {
                    if (armLeftStickY > 0)//down
                    {
                        motorSlideLeft.setPower(armLeftStickY * (slidePower * 0.50)); //0.85 // 0.70
                        motorSlideRight.setPower(armLeftStickY * (slidePower * 0.50)); //0.85 // 0.70
                    }
                    else//up
                    {
                        motorSlideLeft.setPower(armLeftStickY * slidePower);
                        motorSlideRight.setPower(armLeftStickY * slidePower);
                    }

                    slidePosition = motorSlideLeft.getCurrentPosition();
                }
                else
                {
                    if (motorSlideLeft.getCurrentPosition() >= brakeActivePosition)
                    {
                        motorSlideLeft.setPower(brakePower);
                        motorSlideRight.setPower(brakePower);

//                        if (motorSlideLeft.getCurrentPosition() < (slidePosition - 5))//up
//                        {
//                            brakePower = brakePower - 0.01;
//                            motorSlideLeft.setPower(brakePower);
//                            motorSlideRight.setPower(brakePower);
//                        }
//                        else if (motorSlideLeft.getCurrentPosition() > (slidePosition + 5))//down
//                        {
//                            brakePower = brakePower + 0.01;
//                            motorSlideLeft.setPower(brakePower);
//                            motorSlideRight.setPower(brakePower);
//                        }

                    }

                }

            }
            else
            {
                if (armLeftStickY != 0.0)
                {
                    motorSlideLeft.setPower(armLeftStickY * slidePower);
                    motorSlideRight.setPower(armLeftStickY * slidePower);
                }
                else
                {
                    if (motorSlideLeft.getCurrentPosition() >= brakeActivePosition)
                    {
                        motorSlideLeft.setPower(brakePower);
                        motorSlideRight.setPower(brakePower);
                    }
                }
            }
        }


    }

    public void showTelemetry()
    {
        armLeftStickY = gamepad2.left_stick_y;
        armRightStickY = gamepad2.right_stick_y;
        odometry.update();

        //telemetry.addData("clawArm:", clawArm.getCurrentPosition());

        //telemetry.addData("SlideLeft:", slideLeft.getCurrentPosition());
        //telemetry.addData("LeftPower:", slideLeft.getPower());
        //telemetry.addData("SlideLeft target:", slideLeft.getTargetPosition());

        //telemetry.addData("SlideRight:", slideRight.getCurrentPosition());
        //telemetry.addData("RightPower", slideRight.getPower());
        //telemetry.addData("SlideRight target:", slideRight.getTargetPosition());

        //telemetry.addData("LeftStickY:", armLeftStickY);

        //telemetry.addData("rotation mode", rotateMode);
        telemetry.addData("arm power", motorClawArm.getPower());

        telemetry.addData("FL power", motorFrontLeft.getPower());
        telemetry.addData("FR power", motorFrontRight.getPower());
        telemetry.addData("BL power", motorBackLeft.getPower());
        telemetry.addData("BR power", motorBackRight.getPower());

        //telemetry.addData("extender position", clawExtend.getPosition());

        telemetry.addData("odometry Position X", odometry.getPosX());
        telemetry.addData("odometry Position Y", odometry.getPosY());
        //telemetry.addData("odometry Radians", odometry.getHeading());
        //telemetry.addData("odometry Degrees", (odometry.getHeading() * (180 / Math.PI)));


        telemetry.addData("odometry velocity X", odometry.getVelX());
        telemetry.addData("odometry velocity Y", odometry.getVelY());
        //telemetry.addData("odometry velocity Radians", odometry.getHeadingVelocity());
        telemetry.addData("odometry velocity Degrees", (odometry.getHeadingVelocity() * (180 / Math.PI)));
        telemetry.addData("arm target", armTarget);
        telemetry.addData("arm current", motorClawArm.getCurrentPosition());
        telemetry.addData("slide target", slideTarget);
        telemetry.addData("slide start", slideHangStart);
        telemetry.addData("slide position", motorSlideLeft.getCurrentPosition());
        telemetry.addData("arm left stick y", armLeftStickY);
        telemetry.addData("arm right stick y", armRightStickY);
        telemetry.addData("Step", step);

        //telemetry.addData("Target Orientation", orientationTarget);slideHangStart
        //telemetry.addData("Distance to Orientation", distanceToTargetOrientation);
        telemetry.addData("Rotation Direction", rotationDirection);
        telemetry.addData("button", buttonArmStop.isPressed());
        telemetry.addData("Test", bot.test);

        telemetry.update();
    }

    public void initialize()
    {
        // setting hardware to variables

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");
        buttonArmStop = hardwareMap.get(TouchSensor.class, "Button");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //motorClawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorSlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        motorSlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");

        motorSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorClawArm = hardwareMap.get(DcMotorEx.class, "Arm");

        motorClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorClawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorClawArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorClawArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorTilter = hardwareMap.get(DcMotorEx.class, "Tilter");
        motorTilter.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTilter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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
        bot.setButton(buttonArmStop);
        bot.setSpeed(0.4);
        bot.setOffsetIMU(imu);
        bot.getIMUOffset();
        bot.reset();

        //servoClawExtend.setPosition(armRetractedPosition);
        motorTilter.setPower(-1.0);
        sleep(100);
        motorTilter.setPower(0.0);

        waitForStart();
        bot.getIMUOffset();
    }

    @Override
    public void runOpMode()
    {
        initialize();
        while(opModeIsActive())
        {
            if (hangStarted == false)
            {

                drive();
                armExtend();
                armRotate();
                slides();

                driverButtons();
                armButtons();
                showTelemetry();
                /*
                rotateReturn = bot.navRotate(orientationTarget, rotationDirection, rotationErrorTolerance);
                distanceToTargetOrientation = rotateReturn[0];
                orientationTarget = rotateReturn[1];


                if(orientationTarget < 0)
                {
                    positionReturn = bot.navToPosition(positionXTarget, positionYTarget, positionOrientationTarget, true);
                    distanceToTargetPosition = positionReturn[0];
                    positionXTarget = positionReturn[1];
                }

                slideTarget = bot.slideToPosition(slideTarget);
                armTarget = bot.armToPosition(armTarget);

                 */
            }

            if (hangStarted)
            {
                robotHang();
                slideTarget = bot.slideToPositionHang(slideTarget, slideDirection);
                armTarget = bot.armToPositionHang(armTarget, armDirection);

                //armButtons();
                driverButtons();
                //slides();
                showTelemetry();
            }
        }
    }
}
