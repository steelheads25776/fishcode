package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class AutonomousBlue extends OpMode
{
    DcMotor frontLeft, frontRight, backLeft, backRight;
    GoBildaPinpointDriver odometry;
    Robot bot;

    public void showTelemetry()
    {
        telemetry.addData("degrees", bot.getDeg());
        bot.update();
        telemetry.update();
    }

    public void init()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        bot = new Robot(odometry);
        bot.setDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        bot.reset();
        bot.setMotors(frontLeft, frontRight, backLeft, backRight);
        //bot.setOffsets(0, 0); // i do not know these values
    }

    public void start()
    {
        bot.rotate(-90);
    }

    public void loop()
    {
        showTelemetry();
    }
}
