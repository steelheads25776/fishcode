package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Bots;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Extender;

@TeleOp
public class TestTeleopDT extends LinearOpMode
{
    DriveTrain dt = new DriveTrain();
    Claw claw = new Claw();
    Extender extend = new Extender();

    DcMotorEx motorTilter;
    IMU imu;
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    double leftStickX;
    double leftStickY;
    double rightStickX;

    boolean buttonX;

    boolean armButtonA;

    double currentRotation;

    double speed = 0.8;

    int clawState = 0;
    boolean pressedA = false;

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
        dt.motorFrontLeft.setPower(((rotationY + rotationX + rightStickX) / denominator) * speed);
        dt.motorFrontRight.setPower(((rotationY - rotationX - rightStickX) / denominator) * speed);
        dt.motorBackLeft.setPower(((rotationY - rotationX + rightStickX) / denominator) * speed);
        dt.motorBackRight.setPower(((rotationY + rotationX - rightStickX) / denominator) * speed);
    }

    public void armButtons()
    {
        armButtonA = gamepad2.a;
        if(armButtonA && !pressedA)
        {
            if(clawState == 0)//claw is already open, go to closed position
            {
                claw.setPos(Claw.Positions.CLOSE);
                pressedA = true;
                clawState = 1;
            }
            else if(clawState == 1)//claw closed, go to open position
            {
                claw.setPos(Claw.Positions.OPEN);
                pressedA = true;
                clawState = 0;
            }
        }

        if(pressedA && !armButtonA)
        {
            pressedA = false;
        }
    }

    public void initialize2()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        dt.QuickInit(Bots.botlist.SPARKY);
        claw.quickInit(Bots.botlist.SPARKY);
        extend.quickInit(Bots.botlist.SPARKY);

        claw.setPos(Claw.Positions.OPEN);

        motorTilter = hardwareMap.get(DcMotorEx.class, "Tilter");
        motorTilter.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTilter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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
            armButtons();
            drive();
        }
    }
}
