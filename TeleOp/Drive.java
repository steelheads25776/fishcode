package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Drive extends LinearOpMode
{

    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    private Servo clawGrabber, clawExtend;
    private DcMotorEx clawArm;
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

    boolean armButtonA;
    boolean armLeftBumper;
    boolean armRightBumper;
    boolean armButtonX;
    double armRightStickY;

    double armExtendedPosition = 0.42;//a
    double armRetractedPosition = 0.0;//x -0.55
    double clawOpenPosition = 0.8;//left bumper
    double clawClosedPosition = 0.4;//right bumper

    boolean clawExtended = false;

    double armRotateSpeed = 0.7; //previously 1.0
    double lasterror = 0;
    int targetPos = 0;
    ElapsedTime timer;

    double sum = 0.0;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;

    public void pidController(DcMotorEx motor, ElapsedTime runtime, double rightStick)
    {
        while(rightStick > 0.0)
        {
            motor.setTargetPosition(targetPos + 1);
            runtime.reset();
            while(runtime.milliseconds() < 50)
            {

            }
        }

        while(rightStick < 0.0)
        {
            motor.setTargetPosition(targetPos - 1);
            runtime.reset();
            while(runtime.milliseconds() < 50)
            {

            }
        }


        double error = motor.getCurrentPosition() - motor.getTargetPosition();

        sum = sum + (error/10);

        double proportional = kp * error;
        double integral = ki * sum;
        double derivative = kd * (lasterror - error);

        motor.setPower((proportional + integral + derivative));
    }

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

    public void claw()
    {
        armRightBumper = gamepad2.right_bumper;//close claw
        armLeftBumper = gamepad2.left_bumper;//open claw

        //open claw
        if(armLeftBumper)
        {
            clawGrabber.setPosition(clawOpenPosition);
        }

        //close claw
        if(armRightBumper)
        {
            clawGrabber.setPosition(clawClosedPosition);
        }

    }

    public void extend()
    {
        armButtonA = gamepad2.a;//extend
        armButtonX = gamepad2.x;//retract
        //Temporary for now, will turn into one button later.



        if(armButtonX)
        {
            if(clawExtended)
            {
                clawExtend.setPosition(armRetractedPosition);
                clawExtended = false;
            }
        }

        if(armButtonA)
        {
            if(!clawExtended)
            {
                clawExtend.setPosition(armExtendedPosition);
                clawExtended = true;
            }
        }
    }

    public void rotate()
    {
        armRightStickY = gamepad2.right_stick_y;
        pidController(clawArm, timer, armRightStickY);
    }

    public void slides()
    {
        //not yet
    }

    public void initialize()
    {
        // setting motor hardware to variables
        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        clawGrabber =  hardwareMap.get(Servo.class, "Grabber");
        clawArm = hardwareMap.get(DcMotorEx.class, "Arm");
        clawExtend = hardwareMap.get(Servo.class, "Extender");


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

        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
    }

    @Override
    public void runOpMode()
    {
        initialize();
        while(opModeIsActive())
        {
            drive();
            claw();
            extend();
            rotate();
        }
    }
}
