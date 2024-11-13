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
    private DcMotorEx slideLeft, slideRight;
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
    double armLeftStickY;

    double armExtendedPosition = 0.42;//a
    double armRetractedPosition = 0.0;//x -0.55
    double clawOpenPosition = 0.8;//left bumper
    double clawClosedPosition = 0.4;//right bumper

    boolean clawExtended = false;

    double armRotateSpeed = 1.0; //previously 0.7
    double slidePower = 1.0;

    int cycleTime = 25; //(ms)   pid controller time between updating target position
    double lastErrorL = 0;
    double lastErrorR = 0;
    int targetL = 0;
    int targetR = 0;
    ElapsedTime timer;

    double sumL = 0.0;
    double sumR = 0.0;
    double kp = 0.001;
    double ki = 0.001;
    double kd = 0.001;


    int clawArmRestingPosition = 114;

    int armFerbDangerZone = 0;
    int slideFerbDangerZone = 0;

    int slideFerbGoodZone = 50;

    double brakePower = -0.2;
    int brakeActivePosition = 300;
    int slideSlowZone = 500;

    public void pidController(DcMotorEx motor, double stick, double sum, int target, double lasterror)
    {

        ElapsedTime runtime = new ElapsedTime();
        while(stick < -0.1)
        {
            motor.setTargetPosition(target + 1);
            runtime.reset();
            while(runtime.milliseconds() < 25)
            {
                //do nothing!!!
            }
        }

        while(stick > 0.1)
        {
            motor.setTargetPosition(target - 1);
            runtime.reset();
            while(runtime.milliseconds() < 25)
            {
                //do nothing!!!
            }
        }


        double error = motor.getCurrentPosition() - motor.getTargetPosition();//boyd is just better quote from boyd. btrue

        sum = sum + (error/10);

        double proportional = kp * error;
        double integral = ki * sum;
        double derivative = kd * (lasterror - error);

        lasterror = error;
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
        clawArm.setPower(armRightStickY * armRotateSpeed);

        /*
        if(clawArm.getCurrentPosition() > clawArmRestingPosition && slideLeft.getCurrentPosition() <= 15)
        {

            while(clawArm.getCurrentPosition() > clawArmRestingPosition)
            {
                clawArm.setPower(0.75);
            }
            clawArm.setPower(0.0);

            clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawArm.setTargetPosition(clawArmRestingPosition - 1);
            clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }

        if(clawArm.getCurrentPosition() > armFerbDangerZone && slideLeft.getCurrentPosition() > slideFerbDangerZone)
        {
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setTargetPosition(slideFerbGoodZone);
            slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        else
        {
            clawArm.setPower(armRightStickY * armRotateSpeed);
        }
         */

    }

    public void slides()
    {
        armLeftStickY = gamepad2.left_stick_y;

        if(slideLeft.getCurrentPosition() < slideSlowZone)
        {
            if(armLeftStickY != 0.0)
            {
                if(armLeftStickY > 0)
                {
                    slideLeft.setPower(armLeftStickY * (slidePower * 0.3));
                    slideRight.setPower(armLeftStickY * (slidePower * 0.3));
                }
                else
                {
                    slideLeft.setPower(armLeftStickY * slidePower);
                    slideRight.setPower(armLeftStickY * slidePower);
                }
            }

            if(slideLeft.getCurrentPosition() >= brakeActivePosition)
            {
                slideLeft.setPower(brakePower);
                slideRight.setPower(brakePower);
            }
            
        }

        else
        {
            if(armLeftStickY != 0.0)
            {
                slideLeft.setPower(armLeftStickY * slidePower);
                slideRight.setPower(armLeftStickY * slidePower);
            }

            if(slideLeft.getCurrentPosition() >= brakeActivePosition)
            {
                slideLeft.setPower(brakePower);
                slideRight.setPower(brakePower);
            }
        }



    }

    public void showTelemetry()
    {
        armLeftStickY = gamepad2.left_stick_y;

        telemetry.addData("clawArm:", clawArm.getCurrentPosition());

        telemetry.addData("SlideLeft:", slideLeft.getCurrentPosition());
        telemetry.addData("LeftPower:", slideLeft.getPower());
        telemetry.addData("SlideLeft target:", slideLeft.getTargetPosition());

        telemetry.addData("SlideRight:", slideRight.getCurrentPosition());
        telemetry.addData("RightPower", slideRight.getPower());
        telemetry.addData("SlideRight target:", slideRight.getTargetPosition());

        telemetry.addData("LeftStickY:", armLeftStickY);

        telemetry.update();
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

        slideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");

        //slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);


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

        clawArm.setDirection(DcMotorSimple.Direction.REVERSE);


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
            slides();
            showTelemetry();
        }
    }
}
