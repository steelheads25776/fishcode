package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TestFieldDrive extends LinearOpMode
{
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorTilter;
    IMU imu;
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    double leftStickX;
    double leftStickY;
    double rightStickX;

    boolean buttonX;

    double currentRotation;

    double speed = 0.8;
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
    public void initialize2()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        motorTilter = hardwareMap.get(DcMotorEx.class, "Tilter");
        motorTilter.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTilter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorTilter.setPower(-1.0);
        sleep(100);
        motorTilter.setPower(0.0);

        waitForStart();
    }

    @Override
    public void runOpMode()
    {
        initialize2();
        while(opModeIsActive())
        {
            drive();
        }
    }
}
