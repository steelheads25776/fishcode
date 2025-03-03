package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bots;

public class Claw
{
    Servo servoClaw;
    double positionMaxOpen = 0.7;
    double positionHalfOpen = 0.4;
    double positionClosed = 0.0;

    public enum Positions
    {
        OPEN,
        HALF,
        CLOSE
    }

    public void quickInit(Bots.botlist bot)
    {
        servoClaw = hardwareMap.get(Servo.class, "fill");
    }
    public void setPos(Positions pos)
    {
        double newPos = 0.0;
        if(pos == Positions.OPEN)
        {
            newPos = positionMaxOpen;
        }
        if(pos == Positions.HALF)
        {
            newPos = positionHalfOpen;
        }
        if(pos == Positions.CLOSE)
        {
            newPos = positionClosed;
        }
        servoClaw.setPosition(newPos);
    }

    public void setVariablePos(double pos)
    {
        servoClaw.setPosition(pos);
    }
}
