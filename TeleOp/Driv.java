package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class Driv extends LinearOpMode
{

    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    double motorspeed = 1.0;
    double motorspeedhigh = 1.0;
    double motorspeedslow = 0.5;
    double motorspeedslower = 0.25;
    float stickLeftX;
    float stickLeftY;
    float stickRightX;
    float stickRightY;

    float triggerLeft;
    float triggerRight;

    public void drive()
    {

        stickLeftX = gamepad1.left_stick_x;
        stickLeftY = gamepad1.left_stick_y;
        stickRightX = gamepad1.right_stick_x;
        stickRightY = gamepad1.right_stick_y;

        triggerLeft = gamepad1.left_trigger;
        triggerRight = gamepad1.right_trigger;

        motorFrontLeft.setPower((stickLeftY - stickLeftX - stickRightX) * motorspeed);
        motorFrontRight.setPower((stickLeftY + stickLeftX + stickRightX) * motorspeed);
        motorBackLeft.setPower((stickLeftY + stickLeftX - stickRightX) * motorspeed);
        motorBackRight.setPower((stickLeftY - stickLeftX + stickRightX) * motorspeed);

        if(triggerLeft >= 0.2)
        {
            motorspeed = motorspeedslow;
        }
        if(triggerRight >= 0.2)
        {
            motorspeed = motorspeedslower;
        }
        else
        {
            motorspeed = motorspeedhigh;
        }
    }

    public void initialize()
    {
        // setting motor hardware to variables
        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");


        // set motor direction
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // set zero power behavior
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
    }

    @Override
    public void runOpMode()
    {
        initialize();
        while(opModeIsActive())
        {
            drive();
        }
    }
}
