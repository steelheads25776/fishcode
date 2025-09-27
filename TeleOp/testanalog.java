package org.firstinspires.ftc.teamcode.TeleOp;

import android.transition.Slide;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class testanalog extends LinearOpMode
{
    DcMotor motorIntake;
    CRServo axon;
    ColorSensor colorTest;
    AnalogInput axonEncoder;
    double rightStickX;
    double leftStickY;

    boolean buttonA;
    boolean buttonB;
    boolean buttonY;
    boolean buttonX;

    boolean dpadL;
    boolean dpadU;
    boolean dpadD;
    boolean dpadR;
    int motorRPM = 6000;

    public double rpmToPower(double rpm)
    {
        return 1 / (motorRPM/rpm);
    }

    public void testAxon()
    {
        rightStickX = gamepad1.right_stick_x;
        buttonA = gamepad1.a;//100%
        buttonB = gamepad1.b;//75%
        buttonX = gamepad1.x;//50%
        buttonY = gamepad1.y;//25%

        if(buttonA)
        {
            axon.setPower(1.0);
        }
        else if(buttonB)
        {
            axon.setPower(0.75);
        }
        else if(buttonX)
        {
            axon.setPower(0.50);
        }
        else if(buttonY)
        {
            axon.setPower(0.25);
        }
        else
        {
            axon.setPower(rightStickX);
        }

    }

    public void testMotor()
    {
        leftStickY = gamepad1.left_stick_y; //control motor manually
        dpadL = gamepad1.dpad_left;         //6000 rpm
        dpadU = gamepad1.dpad_up;           //1620 rpm
        dpadD = gamepad1.dpad_down;         //1150 rpm
        dpadR = gamepad1.dpad_right;        // 435 rpm

        if(dpadL)
        {
            motorIntake.setPower(rpmToPower(6000));
        }
        else if(dpadU)
        {
            motorIntake.setPower(rpmToPower(1620));
        }
        else if(dpadD)
        {
            motorIntake.setPower(rpmToPower(1150));
        }
        else if(dpadR)
        {
            motorIntake.setPower(435);
        }
        else
        {
            motorIntake.setPower(leftStickY);
        }
    }


    public void showTelemetry()
    {
        telemetry.addData("color sensor green", colorTest.green());
        telemetry.addData("color sensor red", colorTest.red());
        telemetry.addData("color sensor blue", colorTest.blue());
        telemetry.addData("color sensor alpha", colorTest.alpha());
        telemetry.addData("analog output", axonEncoder.getVoltage());
        telemetry.addData("axon power", axon.getPower());
        telemetry.update();
    }
    public void initialize2()
    {
        axon = hardwareMap.get(CRServo.class, "axon");
        axonEncoder = hardwareMap.get(AnalogInput.class, "axonEncoder");
        colorTest = hardwareMap.get(ColorSensor.class, "TestColor");
        waitForStart();
    }

    @Override
    public void runOpMode()
    {
        initialize2();
        while(opModeIsActive())
        {
            testAxon();
            showTelemetry();
        }
    }
}