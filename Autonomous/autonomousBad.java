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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Navigation;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class autonomousBad extends OpMode
{

    boolean launchEngaged = false;
    String launchStep = "None";
    boolean magazineEngaged = false;
    double navXTarget = 0;
    double navYTarget = 0;
    double navOrientationTarget = 0;
    boolean navOnlyRotate = false;
    boolean navPrecise = false;
    double navSpeedOverride = 0;
    boolean navEngaged = false;
    double shootingPower = 0.60;//0.75
    double powerRotateCWMax = -0.11;
    double powerRotateCWSlow = -0.05;
    double powerRotateCCWMax = -0.14;
    double powerRotateCCWSlow = -0.07;
    String axonDirection = "cw";
    double axonPreviousPosition = -1000;
    int axonFrozen = 0;
    String launchType = "";
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorShooterRight, motorShooterLeft;
    DcMotor intakeMotor;

    CRServo axon;
    Navigation nav;
    CRServo servoLoaderStartLeft, servoLoaderStartRight, servoLoaderAssist;
    Servo servoLifter;
    AnalogInput axonEncoder;
    ElapsedTime timer;
    ElapsedTime stepTimer;
    ElapsedTime magTimer;
    String step;
    double axonTargetPosition = 43;
    double lifterServoDown = 0.78;
    double lifterServoUp = 0.50;
    GoBildaPinpointDriver odometry;

    double distanceToTarget = 0;
    double distanceToTargetOrientation = 0;
    public void sleep(int milliseconds)
    {
        timer.reset();
        while(timer.milliseconds() < milliseconds)
        {
            //do nothing
        }
    }
    private void axonToPosition(double target, String direction)
    {
        powerRotateCWMax = -0.13;
        powerRotateCWSlow = -0.05;
        powerRotateCCWMax = -0.16;
        powerRotateCCWSlow = -0.07;

        double currentPos = (axonEncoder.getVoltage() / 3.3) * 360;
        if(Math.abs(axonPreviousPosition - currentPos) < .5)
        {
            axonFrozen++;
        }
        else
        {
            axonFrozen = 0;
        }
        if (axonFrozen >= 3)
        {
            powerRotateCWMax = powerRotateCWMax - (.01 * (axonFrozen - 2));
            powerRotateCWSlow = powerRotateCWSlow - (.01 * (axonFrozen - 2));
            powerRotateCCWMax = powerRotateCCWMax - (.01 * (axonFrozen - 2));
            powerRotateCCWSlow = powerRotateCCWSlow - (.01 * (axonFrozen - 2));
        }
        axonPreviousPosition = currentPos;
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
        else if(distanceToTarget < -8)
        {
            axon.setPower(powerRotateCWSlow);
            axon.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (distanceToTarget > 8)
        {
            axon.setPower(powerRotateCCWSlow);
            axon.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if(distanceToTarget < -3)
        {
            axon.setPower(powerRotateCWSlow);
            axon.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (distanceToTarget > 3)
        {
            axon.setPower(powerRotateCCWSlow);
            axon.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            axon.setPower(0.0);
            sleep(100);
            currentPos = (axonEncoder.getVoltage() / 3.3) * 360;
            if(Math.abs(currentPos - target) < 3)
            {
                magazineEngaged = false;
                axonPreviousPosition = -1000;
            }
        }
    }

    public void launchArtifact()
    {
        if(launchEngaged)
        {
            if(launchStep.equalsIgnoreCase("none"))
            {
                magazineEngaged = true;
                launchStep = "a1 - initial position";
            }
            else if (launchStep.equalsIgnoreCase("a1 - initial position") && !magazineEngaged)
            {
                intakeMotor.setPower(0.0);

                servoLoaderAssist.setPower(-1.0);
                servoLoaderStartRight.setPower(-1.0);
                servoLoaderStartLeft.setPower(1.0);
                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "a1 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a1 - lifter engaged") && stepTimer.milliseconds() > 500)
            {
                servoLifter.setPosition(lifterServoDown);
                motorShooterLeft.setPower(shootingPower);
                motorShooterRight.setPower(shootingPower);

                stepTimer.reset();
                launchStep = "a1 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a1 - drop lifter") && stepTimer.milliseconds() > 500)
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
                launchStep = "a2 - get next artifact";
                stepTimer.reset();
            }
            else if (launchStep.equalsIgnoreCase("a2 - get next artifact") && !magazineEngaged)
            {
                if(launchType.equalsIgnoreCase("current"))
                {
                    if(stepTimer.milliseconds() > 500)
                    {
                        servoLoaderAssist.setPower(0.0);
                        servoLoaderStartRight.setPower(0.0);
                        servoLoaderStartLeft.setPower(0.0);
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);

                        launchEngaged = false;
                        launchStep = "none";
                    }
                }
                else if(launchType.equalsIgnoreCase("all"))
                {
                    launchStep = "a2 - initial position";
                }
            }
            else if (launchStep.equalsIgnoreCase("a2 - initial position") && !magazineEngaged)
            {
                /*
                servoLoaderAssist.setPower(-1.0);
                servoLoaderStartRight.setPower(-1.0);
                servoLoaderStartLeft.setPower(1.0);
                 */
                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "a2 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a2 - lifter engaged") && stepTimer.milliseconds() > 500)
            {
                servoLifter.setPosition(lifterServoDown);
                motorShooterLeft.setPower(shootingPower);
                motorShooterRight.setPower(shootingPower);

                stepTimer.reset();
                launchStep = "a2 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a2 - drop lifter") && stepTimer.milliseconds() > 500)
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
                launchStep = "a3 - get next artifact";
            }
            else if (launchStep.equalsIgnoreCase("a3 - get next artifact") && !magazineEngaged)
            {
                launchStep = "a3 - initial position";
            }
            else if (launchStep.equalsIgnoreCase("a3 - initial position") && !magazineEngaged)
            {
                /*
                servoLoaderAssist.setPower(-1.0);
                servoLoaderStartRight.setPower(-1.0);
                servoLoaderStartLeft.setPower(1.0);
                 */
                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "a3 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a3 - lifter engaged") && stepTimer.milliseconds() > 500)
            {
                servoLifter.setPosition(lifterServoDown);
                motorShooterLeft.setPower(shootingPower);
                motorShooterRight.setPower(shootingPower);

                stepTimer.reset();
                launchStep = "a3 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a3 - drop lifter") && stepTimer.milliseconds() > 500)
            {
                stepTimer.reset();
                launchStep = "a3 - turn off launcher";
            }
            else if (launchStep.equalsIgnoreCase("a3 - turn off launcher") && stepTimer.milliseconds() > 1000)
            {
                servoLoaderAssist.setPower(0.0);
                servoLoaderStartRight.setPower(0.0);
                servoLoaderStartLeft.setPower(0.0);
                motorShooterLeft.setPower(0.0);
                motorShooterRight.setPower(0.0);

                launchEngaged = false;
                launchStep = "none";
            }
        }
    }

    public void start()
    {
        step = "start";
    }

    public void init()
    {
        timer = new ElapsedTime();
        stepTimer = new ElapsedTime();
        magTimer = new ElapsedTime();
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

        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorShooterLeft = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        motorShooterRight = hardwareMap.get(DcMotorEx.class, "RightShooter");

        motorShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorShooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        odometry.setOffsets(172, -74, DistanceUnit.MM); // don't know how to get these offsets yet
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odometry.setYawScalar(-1.0);
        odometry.resetPosAndIMU();

        nav = new Navigation(odometry);
        nav.setMotors(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);
    }

    public void showTelemetry()
    {
        telemetry.addData("Step", step);
        telemetry.addData("launchStep", launchStep);
        telemetry.addData("magazineEngaged", magazineEngaged);
        telemetry.addData("axon Posision", (axonEncoder.getVoltage() / 3.3) * 360);
        telemetry.addData("navXTarget", navXTarget);
        telemetry.addData("navYTarget", navYTarget);
        telemetry.addData("navOrientationTarget", navOrientationTarget);
        telemetry.addData("navOnlyRotate", navOnlyRotate);
        telemetry.addData("navPrecise", navPrecise);
        telemetry.addData("navEngaged", navEngaged);


        telemetry.addData("X Current", nav.getX());
        telemetry.addData("Y Current", nav.getY());
        telemetry.addData("Orientation Current", nav.getOrientationCurrent());

        telemetry.addData("distanceToTarget", distanceToTarget);
        telemetry.addData("distanceToTargetOrientation", distanceToTargetOrientation);

        telemetry.addData("Nav Test", nav.test);


        telemetry.update();
    }

    public void Autonomous()
    {
        if (step.equalsIgnoreCase("start"))
        {
            navXTarget = 500;
            navYTarget = 300;
            navOrientationTarget = 90;
            navOnlyRotate = false;
            navPrecise = false;
            navEngaged = false;
            timer.reset();

            magazineEngaged = true;
            axonDirection = "cw";

            step = "move to stack";
        }
        else if(step.equalsIgnoreCase("move to stack") && timer.milliseconds() > 1000 && !magazineEngaged)
        {
            navEngaged = true;

            step = "hit the brakes";
        }
        else if(step.equalsIgnoreCase("hit the brakes") && !navEngaged)
        {
            motorFrontLeft.setPower(-0.1);
            motorFrontRight.setPower(-0.1);
            motorBackLeft.setPower(-0.1);
            motorBackRight.setPower(-0.1);
            timer.reset();
            step = "intake stack";
        }
        else if(step.equalsIgnoreCase("intake stack") && timer.milliseconds() > 100)
        {
            intakeMotor.setPower(1.0);
            navXTarget = 900;
            navYTarget = 300;
            navOrientationTarget = 90;
            navOnlyRotate = false;
            navPrecise = true;
            navSpeedOverride = 0.35;
            navEngaged = true;
            step = "move to shoot position";
        }
        else if(step.equalsIgnoreCase("move to shoot position") && !navEngaged)
        {
            intakeMotor.setPower(0.0);
            navXTarget = -400;
            navYTarget = 200;
            navOrientationTarget = 50;
            navOnlyRotate = false;
            navPrecise = true;
            navSpeedOverride = 0;
            navEngaged = true;
            step = "fire stack";
        }
        else if(step.equalsIgnoreCase("fire stack") && !navEngaged)
        {
            launchType = "all";
            launchEngaged = true;
            launchStep = "none";
            step = "finish shooting";
        }
        else if(step.equalsIgnoreCase("finish shooting") && !launchEngaged)
        {
            step = "end";
        }

//        if (step.equalsIgnoreCase("start"))
//        {
//            navXTarget = 0;
//            navYTarget = 600;
//            navOrientationTarget = 0;
//            navOnlyRotate = false;
//            navPrecise = false;
//            navSpeedOverride = 0.2;
//            nav.distanceToTargetPrevious = 10000;
//            navEngaged = false;
//            timer.reset();
//
//            step = "start timer";
//        }
//        else if(step.equalsIgnoreCase("start timer") && timer.milliseconds() > 1000)
//        {
//            navEngaged = true;
//            step = "end";
//        }
//        else if(step.equalsIgnoreCase("end") && !navEngaged)
//        {
//
//        }
    }

    public void loop()
    {
        Autonomous();
        if(launchEngaged)
        {
            launchArtifact();
        }
        if(navEngaged)
        {
            double[] navReturn = new double[3];
            navReturn = nav.navToPosition(navXTarget, navYTarget, navOrientationTarget, navOnlyRotate, navPrecise, navSpeedOverride, navEngaged);
            if (navReturn[0] == -1)
            {
                navEngaged = false;
            }
            distanceToTarget = navReturn[1];
            distanceToTargetOrientation = navReturn[2];
        }
        if(magazineEngaged)
        {
            axonToPosition(axonTargetPosition, axonDirection);
        }
        showTelemetry();
    }
}
