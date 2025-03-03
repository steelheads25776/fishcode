package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bots;

public class Slides
{
    DcMotorEx motorSlideLeft, motorSlideRight;
    public DcMotor.ZeroPowerBehavior zeroPowerLeft, zeroPowerRight;
    public DcMotorSimple.Direction directionLeft, directionRight;

    public void quickInit(Bots.botlist bot)
    {
        if(bot == Bots.botlist.SPARKY)
        {
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
        }
        else if(bot == Bots.botlist.TESTBOT)
        {
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
        }
    }

    public void powerLeft(double power)
    {
        motorSlideLeft.setPower(power);
    }

    public void powerRight(double power)
    {
        motorSlideRight.setPower(power);
    }

    public void powerBoth(double power)
    {
        motorSlideLeft.setPower(power);
        motorSlideRight.setPower(power);
    }

    public int[] getPositions()
    {
        int[] pos = new int[2];
        pos[0] = motorSlideLeft.getCurrentPosition();
        pos[1] = motorSlideRight.getCurrentPosition();
        return pos;
    }

    public int getLeftPos()
    {
        return motorSlideLeft.getCurrentPosition();
    }

    public int getRightPos()
    {
        return motorSlideRight.getCurrentPosition();
    }

    public void updateZeroPower()
    {
        motorSlideLeft.setZeroPowerBehavior(zeroPowerLeft);
        motorSlideRight.setZeroPowerBehavior(zeroPowerRight);
    }

    public void updateDirection()
    {
        motorSlideLeft.setDirection(directionLeft);
        motorSlideRight.setDirection(directionRight);
    }
}
