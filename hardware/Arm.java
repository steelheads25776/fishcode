package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bots;

public class Arm
{
    DcMotorEx motorArm;

    public void quickInit(Bots.botlist bot)
    {
        motorArm = hardwareMap.get(DcMotorEx.class, "Arm");

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power)
    {
        motorArm.setPower(power);
    }

    public void getPos()
    {
        motorArm.getCurrentPosition();
    }

    public void getPower()
    {
        motorArm.getPower();
    }
}
