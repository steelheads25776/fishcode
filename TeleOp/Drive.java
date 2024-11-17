package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp
public class Drive extends LinearOpMode
{

    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    private DcMotorEx slideLeft, slideRight;
    private Servo clawGrabber, clawExtend;
    private DcMotorEx clawArm;
    double motorspeed = 1.0;
    double rotateSpeed = 0.75;
    double slowRotateSpeed = 0.35;
    double motorspeedhigh = 1.0;
    double motorspeedslow = 0.5;
    double motorspeedslower = 0.25;
    float stickLeftX;
    float stickLeftY;
    float stickRightX;
    float stickRightY;

    float triggerLeft;
    float triggerRight;

    boolean armButtonA;
    boolean armLeftBumper;
    boolean armRightBumper;
    boolean armButtonX;
    double armRightStickY;
    double armLeftStickY;

    boolean armDpadUp, armDpadDown, armDpadLeft, armDpadRight;

    double armExtendedPosition = 0.42;//a
    double armRetractedPosition = 0.0;//x -0.55

    double servoPosition = 0.0;

    double clawOpenPosition = 0.8;//left bumper
    double clawClosedPosition = 0.4;//right bumper

    boolean clawExtended = false;

    double armRotateSpeed = 1.0; //previously 0.7
    double slidePower = 1.0;

    int clawArmRestingPosition = 114;
    int clawArmStartPosition = -2445;

    int armFerbDangerZone = 0;
    int slideFerbDangerZone = 0;

    int slideFerbGoodZone = 50;

    double brakePower = -0.10;
    int brakeActivePosition = 100;
    int slideSlowZone = 100;

    String rotateMode = "slow";

    public void drive()
    {

        stickLeftX = gamepad1.left_stick_x;
        stickLeftY = gamepad1.left_stick_y;
        stickRightX = gamepad1.right_stick_x;
        stickRightY = gamepad1.right_stick_y;

        triggerLeft = gamepad1.left_trigger;
        triggerRight = gamepad1.right_trigger;

        if(stickLeftX == 0.0 && stickLeftY == 0.0)
        {
            rotateMode = "slow";
            motorFrontLeft.setPower((stickRightX * slowRotateSpeed) * -1);
            motorFrontRight.setPower((stickRightX * slowRotateSpeed) );
            motorBackLeft.setPower((stickRightX * slowRotateSpeed) * -1);
            motorBackRight.setPower((stickRightX * slowRotateSpeed));
        }


        else
        {
            rotateMode = "fast";
            motorFrontLeft.setPower((stickLeftY - stickLeftX - (stickRightX * rotateSpeed)) * motorspeed);
            motorFrontRight.setPower((stickLeftY + stickLeftX + (stickRightX* rotateSpeed)) * motorspeed);
            motorBackLeft.setPower((stickLeftY + stickLeftX - (stickRightX* rotateSpeed)) * motorspeed);
            motorBackRight.setPower((stickLeftY - stickLeftX + (stickRightX * rotateSpeed)) * motorspeed);
        }

        if(triggerLeft >= 0.2)
        {
            motorspeed = motorspeedslow;
        }
        if(triggerRight >= 0.2)
        {
            motorspeed = motorspeedslower;
        }
        else
        {
            motorspeed = motorspeedhigh;
        }
    }

    public void claw()
    {
        armRightBumper = gamepad2.right_bumper;//close claw
        armLeftBumper = gamepad2.left_bumper;//open claw



        //open claw
        if(armLeftBumper)
        {
            clawGrabber.setPosition(clawOpenPosition);
        }

        //close claw
        if(armRightBumper)
        {
            clawGrabber.setPosition(clawClosedPosition);
        }

    }

    public void extend()
    {
        armButtonA = gamepad2.a;//extend
        armButtonX = gamepad2.x;//retract
        //Temporary for now, will turn into one button later.

        armDpadUp = gamepad2.dpad_up; //claw forward
        armDpadDown = gamepad2.dpad_down; // claw backward

        armDpadLeft = gamepad2.dpad_left; // rotate robot left
        armDpadRight = gamepad2.dpad_right; // rotate robot right

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
            if(servoPosition <= armExtendedPosition)
            {
                servoPosition = servoPosition + 0.01;

                clawExtend.setPosition(servoPosition);

            }

            else
            {
                clawExtend.setPosition(armExtendedPosition);
                motorFrontLeft.setPower(-0.2);
                motorFrontRight.setPower(-0.2);
                motorBackLeft.setPower(-0.2);
                motorBackRight.setPower(-0.2);
            }
        }

        if(armDpadDown)
        {
            if(servoPosition >= armRetractedPosition)
            {
                servoPosition = servoPosition - 0.01;
                clawExtend.setPosition(servoPosition);

            }

            else
            {
                clawExtend.setPosition(armRetractedPosition);
                motorFrontLeft.setPower(0.2);
                motorFrontRight.setPower(0.2);
                motorBackLeft.setPower(0.2);
                motorBackRight.setPower(0.2);
            }

        }

        if(armButtonX)
        {
            if(clawExtended)
            {
                clawExtend.setPosition(armRetractedPosition);
                servoPosition = armRetractedPosition;
                clawExtended = false;
            }
        }

        if(armButtonA)
        {
            if(!clawExtended)
            {
                clawExtend.setPosition(armExtendedPosition);
                servoPosition = armExtendedPosition;
                clawExtended = true;
            }
        }
    }

    public void rotate()
    {

        armRightStickY = gamepad2.right_stick_y;
        clawArm.setPower((armRightStickY * armRotateSpeed));

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

        if(slideLeft.getCurrentPosition() < slideSlowZone)
        {
            if(armLeftStickY != 0.0)
            {
                if(armLeftStickY > 0)
                {
                    slideLeft.setPower(armLeftStickY * (slidePower * 0.3));
                    slideRight.setPower(armLeftStickY * (slidePower * 0.3));
                }
                else
                {
                    slideLeft.setPower(armLeftStickY * slidePower);
                    slideRight.setPower(armLeftStickY * slidePower);
                }
            }
            else
            {
                if (slideLeft.getCurrentPosition() >= brakeActivePosition)
                {
                    slideLeft.setPower(brakePower);
                    slideRight.setPower(brakePower);
                }
            }

        }

        else
        {
            if(armLeftStickY != 0.0)
            {
                slideLeft.setPower(armLeftStickY * slidePower);
                slideRight.setPower(armLeftStickY * slidePower);
            }

            else
            {
                if (slideLeft.getCurrentPosition() >= brakeActivePosition)
                {
                    slideLeft.setPower(brakePower);
                    slideRight.setPower(brakePower);
                }
            }
        }



    }

    public void showTelemetry()
    {
        armLeftStickY = gamepad2.left_stick_y;

        telemetry.addData("clawArm:", clawArm.getCurrentPosition());

        telemetry.addData("SlideLeft:", slideLeft.getCurrentPosition());
        telemetry.addData("LeftPower:", slideLeft.getPower());
        telemetry.addData("SlideLeft target:", slideLeft.getTargetPosition());

        telemetry.addData("SlideRight:", slideRight.getCurrentPosition());
        telemetry.addData("RightPower", slideRight.getPower());
        telemetry.addData("SlideRight target:", slideRight.getTargetPosition());

        telemetry.addData("LeftStickY:", armLeftStickY);

        telemetry.addData("rotation mode", rotateMode);

        telemetry.addData("FL power", motorFrontLeft.getPower());
        telemetry.addData("FR power", motorFrontRight.getPower());
        telemetry.addData("BL power", motorBackLeft.getPower());
        telemetry.addData("BR power", motorBackRight.getPower());

        telemetry.update();
    }

    public void initialize()
    {
        // setting motor hardware to variables
        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        clawGrabber =  hardwareMap.get(Servo.class, "Grabber");
        clawArm = hardwareMap.get(DcMotorEx.class, "Arm");
        clawExtend = hardwareMap.get(Servo.class, "Extender");

        clawExtend.setPosition(armRetractedPosition);

        slideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");

        //slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        // set motor direction
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // set zero power behavior
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawArm.setDirection(DcMotorSimple.Direction.REVERSE);

        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(clawArm.getCurrentPosition() > clawArmStartPosition)
        {
            clawArm.setPower(-0.50);
        }
        clawArm.setPower(0.0);


        waitForStart();
    }

    @Override
    public void runOpMode()
    {
        initialize();
        while(opModeIsActive())
        {
            drive();
            claw();
            extend();
            rotate();
            slides();
            showTelemetry();
        }
    }
}
