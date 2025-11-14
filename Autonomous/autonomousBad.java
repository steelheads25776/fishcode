package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class autonomousBad extends OpMode
{

    boolean launchEngaged = false;
    boolean magazineEngaged = false;
    double shootingPower = 0.65;//0.75
    double powerRotateCWMax = -0.11;
    double powerRotateCWSlow = -0.05;
    double powerRotateCCWMax = -0.14;
    double powerRotateCCWSlow = -0.07;
    String axonDirection = "cw";
    String launchType = "";
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorShooterRight, motorShooterLeft;

    CRServo axon;
    CRServo servoLoaderStartLeft, servoLoaderStartRight, servoLoaderAssist;
    Servo servoLifter;
    AnalogInput axonEncoder;
    DcMotor intakeMotor;
    ElapsedTime timer;
    ElapsedTime magTimer;
    String step;
    double axonTargetPosition = 41;
    double lifterServoDown = 0.78;
    double lifterServoUp = 0.50;

    private void axonToPosition(double target, String direction)
    {
        powerRotateCWMax = -0.11;
        powerRotateCWSlow = -0.05;
        powerRotateCCWMax = -0.14;
        powerRotateCCWSlow = -0.07;

        double currentPos = (axonEncoder.getVoltage() / 3.3) * 360;
        double distanceToTarget = currentPos - target;

        if(Math.abs(distanceToTarget) >= 30)
        {
            if(direction.equalsIgnoreCase("cw"))
            {
                axon.setPower(powerRotateCWMax);
                axon.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            else
            {
                axon.setPower(powerRotateCCWMax);
                axon.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }
        else if(distanceToTarget < -5)
        {
            axon.setPower(powerRotateCWSlow);
            axon.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (distanceToTarget > 5)
        {
            axon.setPower(powerRotateCCWSlow);
            axon.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if(distanceToTarget < -2)
        {
            axon.setPower(powerRotateCWSlow);
            axon.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (distanceToTarget > 2)
        {
            axon.setPower(powerRotateCCWSlow);
            axon.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            axon.setPower(0.0);
            magTimer.reset();
            if(magTimer.milliseconds() >= 100)
            {
                currentPos = (axonEncoder.getVoltage() / 3.3) * 360;
                if(Math.abs(currentPos - target) < 2)
                {
                    magazineEngaged = false;
                }
            }
        }
    }
    public void Autonomous()
    {
        if(step.equalsIgnoreCase("start"))
        {
            timer.reset();
            motorFrontLeft.setPower(-0.25);
            motorFrontRight.setPower(-0.25);
            motorBackLeft.setPower(-0.25);
            motorBackRight.setPower(-0.25);
            step = "stop";
        }
        else if(step.equalsIgnoreCase("stop") && timer.milliseconds() > 400)
        {
            timer.reset();
            motorFrontLeft.setPower(0.0);
            motorFrontRight.setPower(0.0);
            motorBackLeft.setPower(0.0);
            motorBackRight.setPower(0.0);
            step = "shoot s1";
        }
        else if(step.equalsIgnoreCase("shoot s1") && timer.milliseconds() > 400)
        {

            step = "startshoot";
        }
        if(step.equalsIgnoreCase("startshoot"))
        {
            magazineEngaged = true;
            step = "a1 - initial position";
        }
        else if (step.equalsIgnoreCase("a1 - initial position") && !magazineEngaged)
        {
            intakeMotor.setPower(0.0);

            servoLoaderAssist.setPower(-1.0);
            servoLoaderStartRight.setPower(-1.0);
            servoLoaderStartLeft.setPower(1.0);
            servoLifter.setPosition(lifterServoUp);

            timer.reset();
            step = "a1 - lifter engaged";
        }
        else if (step.equalsIgnoreCase("a1 - lifter engaged") && timer.milliseconds() > 500)
        {
            servoLifter.setPosition(lifterServoDown);
            motorShooterLeft.setPower(shootingPower);
            motorShooterRight.setPower(shootingPower);

            timer.reset();
            step = "a1 - drop lifter";
        }
        else if (step.equalsIgnoreCase("a1 - drop lifter") && timer.milliseconds() > 700)
        {
            axonTargetPosition += 120;
            if(axonTargetPosition < 0)
            {
                axonTargetPosition += 360;
            }
            else if(axonTargetPosition > 360)
            {
                axonTargetPosition -= 360;
            }
            axonDirection = "cw";
            magazineEngaged = true;
            step = "a2 - get next artifact";
        }
        else if (step.equalsIgnoreCase("a2 - get next artifact") && !magazineEngaged)
        {


            step = "a2 - initial position";

        }
        else if (step.equalsIgnoreCase("a2 - initial position") && !magazineEngaged)
        {
                /*
                servoLoaderAssist.setPower(-1.0);
                servoLoaderStartRight.setPower(-1.0);
                servoLoaderStartLeft.setPower(1.0);
                 */
            servoLifter.setPosition(lifterServoUp);

            timer.reset();
            step = "a2 - lifter engaged";
        }
        else if (step.equalsIgnoreCase("a2 - lifter engaged") && timer.milliseconds() > 500)
        {
            servoLifter.setPosition(lifterServoDown);
            motorShooterLeft.setPower(shootingPower);
            motorShooterRight.setPower(shootingPower);

            timer.reset();
            step = "a2 - drop lifter";
        }
        else if (step.equalsIgnoreCase("a2 - drop lifter") && timer.milliseconds() > 700)
        {
            axonTargetPosition += 120;
            if(axonTargetPosition < 0)
            {
                axonTargetPosition += 360;
            }
            else if(axonTargetPosition > 360)
            {
                axonTargetPosition -= 360;
            }
            axonDirection = "cw";
            magazineEngaged = true;
            step = "a3 - get next artifact";
        }
        else if (step.equalsIgnoreCase("a3 - get next artifact") && !magazineEngaged)
        {
            step = "a3 - initial position";
        }
        else if (step.equalsIgnoreCase("a3 - initial position") && !magazineEngaged)
        {
                /*
                servoLoaderAssist.setPower(-1.0);
                servoLoaderStartRight.setPower(-1.0);
                servoLoaderStartLeft.setPower(1.0);
                 */
            servoLifter.setPosition(lifterServoUp);

            timer.reset();
            step = "a3 - lifter engaged";
        }
        else if (step.equalsIgnoreCase("a3 - lifter engaged") && timer.milliseconds() > 500)
        {
            servoLifter.setPosition(lifterServoDown);
            motorShooterLeft.setPower(shootingPower);
            motorShooterRight.setPower(shootingPower);

            timer.reset();
            step = "a3 - drop lifter";
        }
        else if (step.equalsIgnoreCase("a3 - drop lifter") && timer.milliseconds() > 1000)
        {
            axonTargetPosition += 120;
            if(axonTargetPosition < 0)
            {
                axonTargetPosition += 360;
            }
            else if(axonTargetPosition > 360)
            {
                axonTargetPosition -= 360;
            }
            axonDirection = "cw";
            magazineEngaged = true;
            step = "a3 - turn off launcher";
        }
        else if (step.equalsIgnoreCase("a3 - turn off launcher") && !magazineEngaged)
        {
            servoLoaderAssist.setPower(0.0);
            servoLoaderStartRight.setPower(0.0);
            servoLoaderStartLeft.setPower(0.0);
            motorShooterLeft.setPower(0.0);
            motorShooterRight.setPower(0.0);

            launchEngaged = false;
            step = "none";
        }
    }
    public void start()
    {
        step = "start";
    }

    public void init()
    {
        timer = new ElapsedTime();
        servoLoaderStartLeft = hardwareMap.get(CRServo.class, "StartLeft");
        servoLoaderStartRight = hardwareMap.get(CRServo.class, "StartRight");
        servoLoaderAssist = hardwareMap.get(CRServo.class, "LoaderAssist");
        servoLifter = hardwareMap.get(Servo.class, "lifter");

        axon = hardwareMap.get(CRServo.class, "axon");
        axonEncoder = hardwareMap.get(AnalogInput.class, "axonEncoder");

        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        motorShooterLeft = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        motorShooterRight = hardwareMap.get(DcMotorEx.class, "RightShooter");

        motorShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorShooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void loop()
    {
        Autonomous();
        if(magazineEngaged)
        {
            axonToPosition(axonTargetPosition, axonDirection);
        }
    }
}
