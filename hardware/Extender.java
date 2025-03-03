package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bots;

public class Extender
{
    Servo servoExtender;
    double positionExtended = 0.0;//?
    double positionRetracted = 0.0;//?

    public enum positions
    {
        RETRACT,
        EXTEND
    }

    public void quickInit(Bots.botlist bot)
    {
        servoExtender = hardwareMap.get(Servo.class, "Extender");
    }

    public void setPos(positions position)
    {
        double newPos = 0.0;
        if(position == positions.EXTEND)
        {
            newPos = positionExtended;
        }
        else if(position == positions.RETRACT)
        {
            newPos = positionRetracted;
        }
        servoExtender.setPosition(newPos);
    }

    public void setVariablePos(double position)
    {
        servoExtender.setPosition(position);
    }
}
