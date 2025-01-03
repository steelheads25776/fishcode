package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp
public class TestMode extends LinearOpMode
{
    GoBildaPinpointDriver odometry;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorSlideLeft, motorSlideRight;
    DcMotorEx motorClawArm;
    VoltageSensor sensorVoltage;
    Servo servoClawGrabber, servoClawExtend;
    Robot bot;


    double motorspeed = 1.0;
    double rotateSpeed = 1.00;
    double slowRotateSpeed = 0.55;
    double motorspeedhigh = 1.0;
    double motorspeednormal = 0.65;
    double motorspeedslower = 0.25;
    float stickLeftX;
    float stickLeftY;
    float stickRightX;
    float stickRightY;

    float triggerLeft;
    float triggerRight;

    boolean driveButtonA;
    boolean driveButtonX;
    boolean driveButtonB;
    boolean driveButtonY;


    boolean armButtonA;
    boolean armLeftBumper;
    boolean armRightBumper;
    boolean armButtonX;
    boolean armButtonB;
    boolean armButtonY;
    boolean driveDpadL, driveDpadU, driveDpadD, driveDpadR;
    double armRightStickY;
    double armRightStickX;
    double armLeftStickY;
    float armTriggerLeft;
    float armTriggerRight;

    boolean armDpadUp, armDpadDown, armDpadLeft, armDpadRight;

    double armExtendedPosition = 0.42;//a
    double armRetractedPosition = 0.0;//x -0.55

    double servoPosition = 0.0;

    double clawOpenPosition = 0.7;//left bumper
    double clawClosedPosition = 0.1;//right bumper

    boolean clawExtended = false;

    double armRotateSpeed = 1.0; //previously 0.7
    double slidePower = 1.0;

    int clawArmRestingPosition = 114;
    int clawArmStartPosition = -2445;

    int armFerbDangerZone = 0;
    int slideFerbDangerZone = 0;

    int slideFerbGoodZone = 50;

    int slideBarPosition = 0; //?
    int slideBucketPosition = 0; //?

    int armBarPosition = 0; //?
    int armBucketPosition = 0; //?

    int barSquaredPosition = 0; //?

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
    double slideTarget = -1.0;
    double armTarget = -10000;

    String step = "none";

    boolean hangStarted = false;
    boolean noSlow = false;

    double brakePower = -0.10;
    int brakeActivePosition = 100;
    int slideSlowZone = 100;

    int slidePosition = 0;

    String rotateMode = "slow";

    ElapsedTime timerR = new ElapsedTime();
    ElapsedTime timerL = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    boolean hitR = false;
    boolean hitL = false;

    public void armToGamePosition()
    {
        /*if(position.equalsIgnoreCase("zero"))
        {
            armTarget = 0;
            slideTarget = 0;
        }
        else if(position.equalsIgnoreCase("bar"))
        {
            armTarget = 2500;
            slideTarget = 930;
        }
        else if(position.equalsIgnoreCase("bucket"))
        {
            armTarget = 3000;
            slideTarget = 2000;
        */
        if(step.equalsIgnoreCase("hang0"))
        {
            timer.reset();
            double hangPower = 0.50;
            motorFrontLeft.setPower(hangPower);
            motorFrontRight.setPower(hangPower);
            motorBackLeft.setPower(hangPower);
            motorBackRight.setPower(hangPower);
            step = "hang1";
        }
        else if(step.equalsIgnoreCase("hang1") && timer.milliseconds() > 100)
        {
            motorFrontLeft.setPower(0.0);
            motorFrontRight.setPower(0.0);
            motorBackLeft.setPower(0.0);
            motorBackRight.setPower(0.0);
            //orientationTarget = 0;
            //positionYTarget = -20;
            //positionXTarget = 0;
            slideTarget = 350;
            //armTarget = 1400;
            step = "hang1started";
        }
        else if (step.equalsIgnoreCase("hang1started") && slideTarget <= -1)
        {
            step = "hang2";
        }

        else if(step.equalsIgnoreCase("hang2"))
        {
            slideTarget = 1180;
            armTarget = 1400;
            step = "hang2started";
        }
        else if(step.equalsIgnoreCase("hang2started") && armTarget <= -9999)
        {
            step = "hang3";
        }

        else if(step.equalsIgnoreCase("hang3"))
        {
            timer.reset();
            double hangPower = -0.40;
            motorFrontLeft.setPower(hangPower);
            motorFrontRight.setPower(hangPower);
            motorBackLeft.setPower(hangPower);
            motorBackRight.setPower(hangPower);

            step = "hang3started";
        }
        else if(step.equalsIgnoreCase("hang3started") && timer.milliseconds() > 700)
        {
            step = "hang4";
        }

        else if (step.equalsIgnoreCase("hang4"))
        {
            motorFrontLeft.setPower(0.0);
            motorFrontRight.setPower(0.0);
            motorBackLeft.setPower(0.0);
            motorBackRight.setPower(0.0);
            noSlow = true;
            armTarget = -965;
            slideTarget = 1850;

            step = "hang4started";
        }
        else if(step.equalsIgnoreCase("step4started") && armTarget < -9999 && slideTarget < -9999)
        {
            step = "hang5";
        }
    }

    public void squareToBar()
    {
        driveButtonA = gamepad1.a;
        if(driveButtonA)
        {

        }
    }
    public void squareToBucket()
    {
        driveButtonX = gamepad1.x;
    }


    public void barPlacePosition()
    {
        armButtonB = gamepad2.b;
        if(armButtonB)
        {

        }
    }
    public void bucketPlacePosition()
    {
        armButtonY = gamepad2.y;
        if(armButtonY)
        {

        }
    }

    public void extra()
    {
        squareToBar();
        squareToBucket();
        barPlacePosition();
        bucketPlacePosition();
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

        if(driveDpadL)
        {
            //armToGamePosition("bar");
        }
        else if(driveDpadU)
        {
            //armToGamePosition("bucket");
            orientationTarget = 180;
            rotationDirection = "counterclockwise";
        }
        else if(driveDpadR)
        {
            //armToGamePosition("bar");
        }
        else if(driveDpadD)
        {
            //armToGamePosition("zero");
            orientationTarget = 0;
            rotationDirection = "clockwise";
        }
        else if (driveButtonY && step == "none")
        {
            motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorClawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //bot.reset();
            step = "hang0";
            hangStarted = true;
            /*
            if(step.equalsIgnoreCase("hang0started"))
            {
                step = "hang1";
            }
            //armToGamePosition("hang1");
            if(step.equalsIgnoreCase("hang1started") && slideTarget <= -1)
            {
                step = "hang2";
                //armToGamePosition("hang2");
            }
            else if(step.equalsIgnoreCase("hang2started") && armTarget <= -9999)
            {
                //step = "hang3";
            }

             */
        }
        else if (driveButtonB)
        {
            hangStarted = false;
            noSlow = false;
            //bot.reset();
        }

    }

    public void drive()
    {

        stickLeftX = gamepad1.left_stick_x;
        stickLeftY = gamepad1.left_stick_y;
        stickRightX = gamepad1.right_stick_x;
        stickRightY = gamepad1.right_stick_y;

        triggerLeft = gamepad1.left_trigger;
        triggerRight = gamepad1.right_trigger;

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

            if(stickLeftX == 0.0 && stickLeftY == 0.0)
            {
                double rotationSpeed = slowRotateSpeed;
                if(triggerLeft > 0.1)
                {
                    rotationSpeed = motorspeedhigh;
                }
                else if(triggerRight > 0.1)
                {
                    rotationSpeed = motorspeedslower;
                }

                rotateMode = "slow";
                motorFrontLeft.setPower((stickRightX * rotationSpeed) * -1);
                motorFrontRight.setPower((stickRightX * rotationSpeed) );
                motorBackLeft.setPower((stickRightX * rotationSpeed) * -1);
                motorBackRight.setPower((stickRightX * rotationSpeed));
            }
            else
            {
                rotateMode = "fast";
                double denominator = Math.max((Math.abs(stickLeftX) + Math.abs(stickLeftY) + Math.abs(stickRightX * rotateSpeed)), 1.0);
                motorFrontLeft.setPower(((stickLeftY - stickLeftX - (stickRightX * rotateSpeed)) / denominator) * motorspeed);
                motorFrontRight.setPower(((stickLeftY + stickLeftX + (stickRightX* rotateSpeed)) / denominator) * motorspeed);
                motorBackLeft.setPower(((stickLeftY + stickLeftX - (stickRightX* rotateSpeed)) / denominator) * motorspeed);
                motorBackRight.setPower(((stickLeftY - stickLeftX + (stickRightX * rotateSpeed)) / denominator) * motorspeed);
            }
        }

        if(triggerLeft >= 0.1)
        {
            motorspeed = motorspeedhigh;

        }

        else if(triggerRight >= 0.1)
        {
            motorspeed = motorspeedslower;

        }

        else
        {
            motorspeed = motorspeednormal;
        }
    }

    public void claw()
    {
        armRightBumper = gamepad2.right_bumper;//close claw
        armLeftBumper = gamepad2.left_bumper;//open claw



        //open claw
        if(armLeftBumper)
        {
            servoClawGrabber.setPosition(clawOpenPosition);
        }

        //close claw
        if(armRightBumper)
        {
            servoClawGrabber.setPosition(clawClosedPosition);
        }

    }

    public void extend()
    {
        armButtonA = gamepad2.a;//extend
        armButtonX = gamepad2.x;//retract
        //Temporary for now, will turn into one button later.


        armDpadUp = gamepad2.dpad_up; // forward
        armDpadDown = gamepad2.dpad_down; // backward

        armDpadLeft = gamepad2.dpad_left; // rotate robot left
        armDpadRight = gamepad2.dpad_right; // rotate robot right

        armTriggerLeft = gamepad2.left_trigger;//retract
        armTriggerRight = gamepad2.right_trigger;//extend

        armRightStickX = gamepad2.right_stick_x;

        if(armRightStickX > 0.1 || armRightStickX < -0.1)
        {
            motorFrontLeft.setPower((armRightStickX * motorspeedslower) * -1);
            motorFrontRight.setPower((armRightStickX * motorspeedslower) );
            motorBackLeft.setPower((armRightStickX * motorspeedslower) * -1);
            motorBackRight.setPower((armRightStickX * motorspeedslower));
        }



        if(armDpadLeft)
        {
            motorFrontLeft.setPower((rotateSpeed * 0.5) * 1);
            motorFrontRight.setPower((rotateSpeed * 0.5) * -1);
            motorBackLeft.setPower((rotateSpeed * 0.5) * -1);
            motorBackRight.setPower((rotateSpeed * 0.5) * 1);
        }

        if(armDpadRight)
        {
            motorFrontLeft.setPower((rotateSpeed * 0.5) * -1);
            motorFrontRight.setPower((rotateSpeed * 0.5) * 1);
            motorBackLeft.setPower((rotateSpeed * 0.5) * 1);
            motorBackRight.setPower((rotateSpeed * 0.5) * -1);
        }

        if(armDpadUp)
        {
            motorFrontLeft.setPower(-0.2);
            motorFrontRight.setPower(-0.2);
            motorBackLeft.setPower(-0.2);
            motorBackRight.setPower(-0.2);
        }

        if(armDpadDown)
        {
            motorFrontLeft.setPower(0.2);
            motorFrontRight.setPower(0.2);
            motorBackLeft.setPower(0.2);
            motorBackRight.setPower(0.2);
        }

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

    public void rotate()
    {
        armRightStickY = gamepad2.right_stick_y;
        if(armRightStickY == 0 && armTarget > -10000)
        {
            // do nothing - arm is moving
        }
        else
        {
            armTarget = -10000;
            motorClawArm.setPower((armRightStickY * armRotateSpeed) * -1);
        }
        /*
        if(clawArm.getCurrentPosition() > clawArmRestingPosition && slideLeft.getCurrentPosition() <= 15)
        {

            while(clawArm.getCurrentPosition() > clawArmRestingPosition)
            {
                clawArm.setPower(0.75);
            }
            clawArm.setPower(0.0);

            clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawArm.setTargetPosition(clawArmRestingPosition - 1);
            clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }

        if(clawArm.getCurrentPosition() > armFerbDangerZone && slideLeft.getCurrentPosition() > slideFerbDangerZone)
        {
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setTargetPosition(slideFerbGoodZone);
            slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        else
        {
            clawArm.setPower(armRightStickY * armRotateSpeed);
        }
         */

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
                        motorSlideLeft.setPower(armLeftStickY * (slidePower * 0.35));
                        motorSlideRight.setPower(armLeftStickY * (slidePower * 0.35));
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
        telemetry.addData("arm current position", motorClawArm.getCurrentPosition());
        telemetry.addData("slide position", motorSlideLeft.getCurrentPosition());
        telemetry.addData("arm left stick y", armLeftStickY);
        telemetry.addData("arm right stick y", armRightStickY);
        telemetry.addData("Step", step);

        telemetry.addData("Target Orientation", orientationTarget);
        telemetry.addData("Distance to Orientation", distanceToTargetOrientation);
        telemetry.addData("Rotation Direction", rotationDirection);
        telemetry.addData("Test", bot.test);

        telemetry.update();
    }

    public void initialize()
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

        servoClawExtend.setPosition(armRetractedPosition);

        waitForStart();
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
                claw();
                extend();
                rotate();
                slides();
            }
            driverButtons();
            //extra();
            showTelemetry();

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
            armTarget = bot.armToPosition(armTarget, noSlow);
            if (hangStarted)
            {
                armToGamePosition();
            }
        }
    }
}
