package org.firstinspires.ftc.teamcode.TeleOp;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class axonCRTest extends LinearOpMode
{
    //AnalogInput encoder;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx SlideLeft, SlideRight;
    IMU imu;
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    CRServo axon;
    Servo claw;

    double leftStickX;
    double leftStickY;
    double rightStickY;
    double rightStickX;

    boolean buttonX;
    boolean bumperRight;
    boolean bumperLeft;
    boolean buttonY;

    double currentRotation;

    double speed = 0.8;

    double slideSpeedFast = 1.0;
    double slideSpeedNormal = 0.75;
    double slideSpeedSlow = 0.5;
    double slideSpeed = 0.75;
    double slowedSpeed = 0.0;
    boolean slideRetracting = false;
    boolean canExtendForward = true;
    int slideTarget = -100;
    boolean slideEngaged = false;

    boolean buttonA;
    boolean buttonB;
    //public boolean slideTargetReached = false;
    double deadX = 0.2;
    double deadY = 0.2;
    double stickOrientation = 0.0;
    double stickOrientationRaw = 0.0;
    double slidePower = 0.0;

    public void getStickOrientation()
    {
        rightStickX = gamepad1.right_stick_x;
        rightStickY = gamepad1.right_stick_y;

        stickOrientation = (Math.atan(rightStickY/rightStickX) * 180/Math.PI);
        stickOrientationRaw = (Math.atan(rightStickY/rightStickX) * 180/Math.PI);

        if(rightStickX < (deadX*-1))//left side
        {
            stickOrientation = (Math.atan(rightStickY/rightStickX) * 180/Math.PI) + 270;
        }
        else if(rightStickX >= deadX)//right side
        {
            stickOrientation = (Math.atan(rightStickY/rightStickX) * 180/Math.PI) + 90;
        }
    }
    public void drive()
    {

        leftStickX = gamepad1.left_stick_x * -1;
        leftStickY = gamepad1.left_stick_y;
        rightStickX = gamepad1.right_stick_x * -1;
        buttonX = gamepad1.x;

        if(buttonX)
        {
            imu.resetYaw();
        }

        currentRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotationX = leftStickX * Math.cos(-currentRotation) - leftStickY * Math.sin(-currentRotation);
        double rotationY = leftStickX * Math.sin(-currentRotation) + leftStickY * Math.cos(-currentRotation);

        rotationX = rotationX * 1.1;

        double denominator = Math.max(Math.abs(rotationY) + Math.abs(rotationX) + Math.abs(rightStickX), 1);
        motorFrontLeft.setPower(((rotationY + rotationX + rightStickX) / denominator) * speed);
        motorFrontRight.setPower(((rotationY - rotationX - rightStickX) / denominator) * speed);
        motorBackLeft.setPower(((rotationY - rotationX + rightStickX) / denominator) * speed);
        motorBackRight.setPower(((rotationY + rotationX - rightStickX) / denominator) * speed);
    }

    public void slidesGoToPos(double Target)
    {
        double slidePos = SlideLeft.getCurrentPosition();
        double slidePowerLow = 0.35;
        double slidePowerHigh = 0.5;
        double distanceToSlow = 100;
        double distanceToTarget;
        String direction;
        distanceToTarget = Target - slidePos;
        if(Target < slidePos)
        {
            direction = "backward";
        }
        else
        {
            direction = "forward";
        }
        if(slideEngaged)
        {
            if(direction.equalsIgnoreCase("backward"))
            {
                if(distanceToTarget < (distanceToSlow * -1))
                {
                    slidePower = slidePowerHigh * -1;
                }
                else if(distanceToTarget > 0)
                {
                    slideEngaged = false;
                    //slidePower = -0.2;
                }
                else
                {
                    slidePower = ((slidePowerHigh - slidePowerLow) * (distanceToTarget / distanceToSlow)) + slidePowerLow * - 1;
                }
            }
            else if(direction.equalsIgnoreCase("forward"))
            {
                if(distanceToTarget > distanceToSlow)
                {
                    slidePower = slidePowerHigh;
                }
                else if(distanceToTarget < 0)
                {
                    slideEngaged = false;
                    //slidePower = -0.2;
                }
                else
                {
                    slidePower = ((slidePowerHigh - slidePowerLow) * (distanceToTarget / distanceToSlow)) + slidePowerLow;
                }
            }
            SlideLeft.setPower(slidePower);
            SlideRight.setPower(slidePower);
        }
    }

    public void servoTestCR()
    {
        buttonA = gamepad1.a;
        buttonB = gamepad1.b;

        if(buttonA)
        {
            axon.setPower(1.0);
        }
        else if(buttonB)
        {
            axon.setPower(-1.0);
        }
        else
        {
            axon.setPower(0.0);
        }
    }

    public void claw()
    {
        bumperLeft = gamepad1.left_bumper;
        bumperRight = gamepad1.right_bumper;

        if(bumperLeft)// open or close idontknow
        {
            claw.setPosition(0.5);
        }
        else if(bumperRight)
        {
            claw.setPosition(0.0);
        }
    }


    public void testDirection()
    {
        buttonA = gamepad1.a;
        buttonB = gamepad1.b;
        leftStickY = gamepad1.left_stick_y * -1;
        buttonY = gamepad1.y;
        buttonX = gamepad1.x;
        //rightStickY = gamepad1.right_stick_y;
        //rightBumper = gamepad1.right_bumper;
        //leftBumper = gamepad1.left_bumper;

        double distanceToSlow = 200;
        double slideMax = 500;
        
        if(buttonA)//slow
        {
            slideSpeed = slideSpeedSlow;
        }
        else if(buttonB)//fast
        {
            slideSpeed = slideSpeedFast;
        }
        else if(!buttonA && !buttonB)//normal
        {
            slideSpeed = slideSpeedNormal;
        }
        
        if(buttonY)
        {
            slideEngaged = true;
            slideTarget = 350;
        }
        else if(buttonX)
        {
            slideEngaged = true;
            slideTarget = 100;
        }

        if(SlideLeft.getCurrentPosition() <= distanceToSlow)//slow speed to stop crashing
        {
            if(SlideLeft.getPower() > 0.5)
            {
                slideSpeed = 0.5;
            }
            slowedSpeed = ((slideSpeed - 0.2) * (SlideLeft.getCurrentPosition() / distanceToSlow)) + 0.2;
            slideSpeed = slowedSpeed;
        }
        else if(SlideLeft.getCurrentPosition() > distanceToSlow)//back to normal speed
        {
            slideSpeedSlow = 0.5;
            slideSpeedNormal = 0.75;
            slideSpeedFast = 1.0;
        }

        if(SlideLeft.getCurrentPosition() >= slideMax)
        {
            if(leftStickY > 0) //down movement
            {
                SlideLeft.setPower((leftStickY * slideSpeed) * -1);
                SlideRight.setPower((leftStickY * slideSpeed) * -1);
            }
            else
            {
                SlideLeft.setPower(0.0);
                SlideRight.setPower(0.0);
            }
        }
        if(SlideLeft.getCurrentPosition() < slideMax)
        {
            SlideLeft.setPower(leftStickY * slideSpeed);
            SlideRight.setPower(leftStickY * slideSpeed);
        }
    }

    public void showTelemetry()
    {
        telemetry.addData("slide engaged", slideEngaged);
        telemetry.addData("stick orientation", stickOrientation);
        telemetry.addData("stick orientation raw", stickOrientationRaw);
        telemetry.addData("right stick x", rightStickX);
        telemetry.addData("right stick y", rightStickY);
        telemetry.addData("left slide position", SlideLeft.getCurrentPosition());
        telemetry.addData("right slide position", SlideRight.getCurrentPosition());
        telemetry.addData("left stick y", leftStickY);
        telemetry.addData("slideTarget", slideTarget);
        telemetry.addData("slideleft power", SlideLeft.getPower());
        telemetry.addData("slideright power", SlideRight.getPower());
        telemetry.addData("connection info", axon.getConnectionInfo());
        telemetry.addData("power", axon.getPower());
        //telemetry.addData("encoder", encoder.getVoltage());
        telemetry.addData("direction", axon.getDirection());
        telemetry.update();
    }
    public void initialize2()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        axon = hardwareMap.get(CRServo.class, "axon");
        //encoder = hardwareMap.get(AnalogInput.class, "encoder");
        claw = hardwareMap.get(Servo.class, "claw");
        //axon = hardwareMap.get(Servo.class, "axon");

        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");

        SlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        SlideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        SlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
    }

    @Override
    public void runOpMode()
    {
        initialize2();
        while(opModeIsActive())
        {
            showTelemetry();
            testDirection();
            //servoTestCR();
            getStickOrientation();
            claw();
            //slides();
            slidesGoToPos(slideTarget);
            //drive();
        }
    }
}
