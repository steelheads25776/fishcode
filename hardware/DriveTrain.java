package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bots;

public class DriveTrain
{
    public DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    public DcMotorSimple.Direction flDirection, blDirection, frDirection, brDirection;
    public DcMotor.ZeroPowerBehavior flZeroPower, blZeroPower, frZeroPower, brZeroPower;

    public volatile String flId, blId, frId, brId;



    public DriveTrain(String fl, String bl, String fr, String br)
    {
        flId = fl;
        blId = bl;
        frId = fr;
        brId = br;
    }

    public DriveTrain()
    {

    }

    public void InitMotor()
    {
        motorFrontLeft = hardwareMap.get(DcMotor.class, flId);
        motorBackLeft = hardwareMap.get(DcMotor.class, blId);
        motorFrontRight = hardwareMap.get(DcMotor.class, frId);
        motorBackRight = hardwareMap.get(DcMotor.class, brId);
    }

    public void updateDirection()
    {
        motorFrontLeft.setDirection(flDirection);
        motorBackLeft.setDirection(blDirection);
        motorFrontRight.setDirection(frDirection);
        motorBackRight .setDirection(brDirection);
    }

    public void updateZeroPower()
    {
        motorFrontLeft.setZeroPowerBehavior(flZeroPower);
        motorBackLeft.setZeroPowerBehavior(blZeroPower);
        motorFrontRight.setZeroPowerBehavior(frZeroPower);
        motorBackRight.setZeroPowerBehavior(brZeroPower);
    }

    public void QuickInit(Bots.botlist bot)
    {
        if(bot == Bots.botlist.SPARKY)
        {
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
        }
        else if(bot == Bots.botlist.TESTBOT)
        {
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
        }
    }

    public void powerLeftSide(double power) //set front left and back left to power variable
    {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
    }

    public void powerRightSide(double power) //set front right and back right to power variable
    {
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }

    public void powerFrontSide(double power) //set front left and front right to power variable
    {
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
    }

    public void powerBackSide(double power) //set back left and back right to power variable
    {
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
    }

}
