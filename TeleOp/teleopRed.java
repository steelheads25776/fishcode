package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.ShooterBot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.*;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class teleopRed extends LinearOpMode
{
    double shootingPower = 0.62;//0.75
    double powerRotateCWMax = -0.11;
    double powerRotateCWSlow = -0.05;
    double powerRotateCCWMax = -0.14;//-0.14
    double powerRotateCCWSlow = -0.07;
    Limelight3A ll;
    GoBildaPinpointDriver odometry;
    Robot bot;
    ShooterBot robot;
    OpenCvWebcam testCam = null;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorShooterRight, motorShooterLeft;
    //IMU imu;
    //IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));



    CRServo axon;
    CRServo servoLoaderStartLeft, servoLoaderStartRight, servoLoaderAssist;
    Servo servoLifter;
    AnalogInput axonEncoder;
    Servo internalLight;
    DcMotor intakeMotor;

    double driveStickLeftX;
    double driveStickLeftY;
    double driveStickRightX;

    boolean driveB;
    boolean driveBPressed = false;
    boolean driveA;//intake
    boolean driveAPressed = false;
    boolean driveY;
    boolean driveX;//reset orientation
    boolean driveBumperRight;
    boolean driveBumperLeft;
    double currentOrientation;
    double currentOrientationRad;
    double axonPreviousPosition = -1000;
    int axonFrozen = 0;
    double driveTriggerLeft;//high speed on drive
    double driveTriggerRight;//slow speed on drive
    boolean driveDpadL;
    boolean driveDpadU;
    boolean driveDpadD;
    boolean driveDpadR;
    double axonMaxSpeed = 0.20;
    String rotateMode = "slow";

    double motorspeedhigh = 1.0;
    double motorspeed = 1.0;
    double motorspeednormal = 0.75;
    double motorspeedslower = 0.5;
    double slowRotateSpeed = 0.55;
    double robotRotateSpeedMultiplier = 2.0;
    double axonTargetPosition = 42;//43
    String axonDirection = "cw";
    String launchType = "";
    String launchStep = "None";

    boolean auxA, auxB, auxY, auxX;
    boolean auxDpadL, auxDpadR, auxDpadU, auxDpadD;
    boolean auxDpadLPressed, auxDpadRPressed, auxDpadUPressed, auxDpadDPressed = false;
    boolean auxBumperLeft, auxBumperRight;
    double auxStickLeftX, auxStickLeftY, auxStickRightX, auxStickRightY;
    double auxTriggerRight, auxTriggerLeft;
    boolean auxTriggerRightPressed, auxTriggerLeftPressed = false;
    boolean auxXPressed;
    boolean auxYPressed;
    boolean auxDpadPressed;
    boolean auxAPressed;
    boolean auxBPressed;
    boolean auxBumperLeftPressed;
    boolean auxBumperRightPressed;
    boolean launchEngaged = false;
    int loaderState = 0;
    int shooterState = 0;
    int intakeState = 0;
    int lifterServoState = 0;
    int magState = 0;
    boolean bumperRightPressed;
    final int initialTarget = 43;
    ElapsedTime testTimer;
    ElapsedTime stepTimer;
    ElapsedTime axonTimer;
    //boolean magazineEngaged = false;
    double maxCorrections = 3;
    int test = 1000;

    double lifterServoDown = 0.78;
    double lifterServoUp = 0.50;

    double rotationX = 0;
    double rotationY = 0;

    int velocityShooting = 1400;//1325
    boolean shootingLong;
    double velocityLongShooting = 1700;//1565, 1650
    String[] chambers = new String[3];
    String launchColor = "none";

    LLResult llresult;
    double llX = 0;
    double llY = 0;
    boolean trackingTag = false;
    ElapsedTime resetTimer;
    String fixStep = "none";
    double trackingError = 1.00;
    double trackingSpeed = 0.0;
    double distanceToGoal = 0;
    public void drive()
    {

        // Game controller left stick provide negative values for up and left and positive for down and right
        // Multiply by  left stick up value by -1.0 to make is so up and right are positive and down and left are negative
        driveStickLeftX = gamepad1.left_stick_x;
        driveStickLeftY = gamepad1.left_stick_y * -1.0;

        if(!trackingTag)
        {
            driveStickRightX = gamepad1.right_stick_x;
        }

        driveTriggerLeft = gamepad1.left_trigger;
        driveTriggerRight = gamepad1.right_trigger;
        //driveX = gamepad1.x;
        driveDpadL = gamepad1.dpad_left;

        currentOrientationRad = odometry.getHeading(AngleUnit.RADIANS) - (Math.PI/2);

        if(driveDpadL)
        {
            bot.reset();
        }

        if (driveStickLeftX == 0.0 && driveStickLeftY == 0.0)
        {
            double rotationSpeed = slowRotateSpeed;
            if (driveTriggerLeft > 0.1)
            {
                rotationSpeed = motorspeedhigh;
            }
            else if (driveTriggerRight > 0.1)
            {
                rotationSpeed = motorspeedslower;
            }

            //rotateMode = "slow";

            motorFrontLeft.setPower((driveStickRightX * rotationSpeed));
            motorFrontRight.setPower((driveStickRightX * rotationSpeed) * -1);
            motorBackLeft.setPower((driveStickRightX * rotationSpeed));
            motorBackRight.setPower((driveStickRightX * rotationSpeed) * -1);
        }
        else
        {
            driveStickRightX *= robotRotateSpeedMultiplier;
            rotationX = driveStickLeftX * Math.cos(currentOrientationRad) - driveStickLeftY * Math.sin(currentOrientationRad);
            rotationY = driveStickLeftX * Math.sin(currentOrientationRad) + driveStickLeftY * Math.cos(currentOrientationRad);

            //rotationX = rotationX * 1.1;

            double denominator = Math.max(Math.abs(rotationY) + Math.abs(rotationX) + Math.abs(driveStickRightX), 1);
            motorFrontLeft.setPower(((rotationY + rotationX + driveStickRightX) / denominator) * motorspeed);
            motorFrontRight.setPower(((rotationY - rotationX - driveStickRightX) / denominator) * motorspeed);
            motorBackLeft.setPower(((rotationY - rotationX + driveStickRightX) / denominator) * motorspeed);
            motorBackRight.setPower(((rotationY + rotationX - driveStickRightX) / denominator) * motorspeed);
        }

        if (driveTriggerLeft >= 0.1)
        {
            motorspeed = motorspeedhigh;

        }
        else if (driveTriggerRight >= 0.1)
        {
            motorspeed = motorspeedslower;

        }
        else
        {
            motorspeed = motorspeednormal;
        }
    }
    public void intake()
    {
        //leftStickY = gamepad1.left_stick_y * -1; // control intake motor value relative to left stick y
        auxA = gamepad2.a;
        auxB = gamepad2.b;
        //auxY = gamepad2.y;
        if(auxA)
        {
            auxAPressed = true;
            auxBPressed = false;
        }
        else if(auxB)
        {
            // reverse intake while B pressed
            intakeMotor.setPower(-0.45);//-0.37
            //internalLight.setPower(.5);;
            auxBPressed = true;
            auxAPressed = false;
        }
        else if(auxBPressed)
        {
            auxBPressed = false;
        }
        else if(intakeState == 1)
        {
            if(!auxAPressed)
            {
                intakeMotor.setPower(1.0);
            }
            else
            {
                intakeMotor.setPower(0.0);
                auxAPressed = false;
                intakeState = 0;
            }
        }
        else if(intakeState == 0)
        {
            if(!auxAPressed)
            {
                intakeMotor.setPower(0.0);
            }
            else
            {
                intakeMotor.setPower(1.0);
                auxAPressed = false;
                intakeState = 1;
            }
        }
    }

    public void magazine()
    {
        auxBumperLeft = gamepad2.left_bumper; //CCW rotation
        auxBumperRight = gamepad2.right_bumper; //CW rotation
        if(auxBumperLeft)
        {
            auxBumperLeftPressed = true;
        }
        if(auxBumperRight)
        {
            auxBumperRightPressed = true;
        }

        if(!auxBumperLeft && auxBumperLeftPressed)
        {
            //testAxon.changeTargetRotation(40);
            axonTargetPosition -= 120;
            if(axonTargetPosition < 0)
            {
                axonTargetPosition += 360;
            }
            else if(axonTargetPosition > 360)
            {
                axonTargetPosition -= 360;
            }
            axonDirection = "ccw";
            robot.magazineEngaged = true;
            auxBumperLeftPressed = false;
        }

        if(!auxBumperRight && auxBumperRightPressed)
        {
            //testAxon.setTargetRotation(160);
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
            robot.magazineEngaged = true;
            auxBumperRightPressed = false;
        }

    }

    public void getDistFromGoal()
    {
        double targetHeight =75;
        double cameraHeight = 30.5;
        double cameraAngle = 15 * (Math.PI/180);
        if(llresult.isValid() && llresult != null)
        {
            llY = llresult.getTy();
            double targetAngle = llY * (Math.PI/180);
            distanceToGoal = (targetHeight - cameraHeight) / Math.tan(cameraAngle + targetAngle);
        }
        else
        {
            distanceToGoal = -1000;
        }

        if(distanceToGoal <= -1000)
        {
            // Can't get distance reading so assume 150 cm
            distanceToGoal = 150;
        }

        // 100 cm is min shooting distance, 300 cm is max shooting distance
        velocityShooting = (int)((distanceToGoal - 100) * ((robot.velocityShootingMax - robot.velocityShootingMin) / 200) + robot.velocityShootingMin);
    }

    public void trackAprilTag()
    {
        driveA = gamepad1.a;
        driveB = gamepad1.b;
        robot.limelight.updateRobotOrientation(bot.getOrientationCurrent());
        llresult = robot.limelight.getLatestResult();

        if(llresult.isValid() && llresult != null)
        {
            llX = llresult.getTx();
        }
        if(driveA)
        {
            trackingTag = true;
        }
        else if(!driveA)
        {
            trackingTag = false;
        }

        if(trackingTag)
        {
            if(distanceToGoal > 250)
            {
                llX +=2;
            }
            if(llX > trackingError)// LEFT IS +, RIGHT IS -
            {
                trackingSpeed = (0.1 + (llX * 0.007));//turn left
                if(trackingSpeed > 1.0)
                {
                    trackingSpeed = 1.0;
                }
                driveStickRightX = trackingSpeed;
            }
            else if(llX < (trackingError * -1))     //turn right
            {
                trackingSpeed = (-0.1 + (llX * 0.007));
                if(trackingSpeed < -1.0)
                {
                    trackingSpeed = -1.0;
                }
                driveStickRightX = trackingSpeed;
            }
            else
            {
                driveStickRightX = 0;
            }

        }
    }
    public void shootSpecific()
    {
        auxDpadL = gamepad2.dpad_left;//shoot purple
        auxDpadR = gamepad2.dpad_right;//shoot green

        if(!launchEngaged)
        {
            if(auxDpadL)
            {
                auxDpadLPressed = true;
            }
            if(!auxDpadL && auxDpadLPressed)
            {
                launchEngaged = true;
                //launchType = "purple";
                launchColor = "purple";
                auxDpadLPressed = false;
            }

            if(auxDpadR)
            {
                auxDpadRPressed = true;
            }
            if(!auxDpadR && auxDpadRPressed)
            {
                launchEngaged = true;
                //launchType = "green";
                launchColor = "green";
                auxDpadRPressed = false;
            }
        }

        if(launchEngaged)
        {
            if(launchStep.equalsIgnoreCase("none"))
            {
                robot.magazineEngaged = true;
                launchStep = "rotate mag to color";
            }
            else if(launchStep.equalsIgnoreCase("rotate mag to color") && !robot.magazineEngaged)
            {
                motorShooterLeft.setVelocity(velocityShooting);
                motorShooterRight.setVelocity(velocityShooting);

                if(chambers[0].equalsIgnoreCase(launchColor))
                {
                    launchStep = "s - initial position";
                }
                else if(chambers[1].equalsIgnoreCase(launchColor))
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
                    robot.magazineEngaged = true;
                    launchStep = "s - initial position";
                }
                else if(chambers[2].equalsIgnoreCase(launchColor))
                {
                    axonTargetPosition -= 120;
                    if(axonTargetPosition < 0)
                    {
                        axonTargetPosition += 360;
                    }
                    else if(axonTargetPosition > 360)
                    {
                        axonTargetPosition -= 360;
                    }
                    axonDirection = "ccw";
                    robot.magazineEngaged = true;
                    launchStep = "s - initial position";
                }
                else
                {
                    // No matching color found
                    launchStep = "s - stop";
                }
            }
            else if (launchStep.equalsIgnoreCase("s - initial position") && !robot.magazineEngaged)
            {
                intakeMotor.setPower(0.0);

                servoLoaderAssist.setPower(1.0);
                servoLoaderStartRight.setPower(1.0);
                servoLoaderStartLeft.setPower(-1.0);
                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "s - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("s - lifter engaged") && stepTimer.milliseconds() > 250)//250
            {
                servoLifter.setPosition(lifterServoDown);
                if(!shootingLong)
                {
                    motorShooterLeft.setVelocity(velocityShooting);
                    motorShooterRight.setVelocity(velocityShooting);
                }
                if(shootingLong)
                {
                    motorShooterLeft.setVelocity(velocityLongShooting);
                    motorShooterRight.setVelocity(velocityLongShooting);
                }

                stepTimer.reset();
                launchStep = "s - stop";
            }

            else if (launchStep.equalsIgnoreCase("s - stop") && !robot.magazineEngaged)
            {
                if (stepTimer.milliseconds() > 1000)
                {
                    servoLoaderAssist.setPower(0.0);
                    servoLoaderStartRight.setPower(0.0);
                    servoLoaderStartLeft.setPower(0.0);
                    motorShooterLeft.setVelocity(0);
                    motorShooterRight.setVelocity(0);

                    launchEngaged = false;
                    launchStep = "none";
                }
            }
        }
    }
    public void fixShooter()
    {
        auxDpadU = gamepad2.dpad_up;
        auxDpadD = gamepad2.dpad_down;
        //auxDpadL = gamepad2.dpad_left;
        //auxDpadR = gamepad2.dpad_right;

        /*
        if(auxDpadL)
        {
            auxDpadLPressed = true;
        }
        else if(!auxDpadL && auxDpadLPressed)
        {
            servoLifter.setPosition(lifterServoDown);
            auxDpadLPressed = false;
        }

        if(auxDpadR)
        {
            auxDpadRPressed = true;
        }
        else if(!auxDpadR && auxDpadRPressed)
        {
            servoLifter.setPosition(lifterServoUp + 0.30);
            auxDpadRPressed = false;
        }

         */

        if(auxDpadD && !auxDpadDPressed)
        {
            resetTimer.reset();
            fixStep = "start";
            auxDpadDPressed = true;
        }
        else if(auxDpadDPressed && !auxDpadD)
        {
            if(fixStep.equalsIgnoreCase("start"))
            {
                motorShooterLeft.setVelocity(-500);
                motorShooterRight.setVelocity(-500);
                servoLoaderAssist.setPower(-1.0);
                resetTimer.reset();
                fixStep = "run shooter";
            }
            else if(fixStep.equalsIgnoreCase("run shooter") && resetTimer.milliseconds() >= 500)//500
            {
                motorShooterLeft.setVelocity(velocityShooting);
                motorShooterRight.setVelocity(velocityShooting);
                //resetTimer.reset();
                fixStep = "reverse assist";
            }
            else if(fixStep.equalsIgnoreCase("reverse assist") && resetTimer.milliseconds() >= 1500)//500
            {
                motorShooterLeft.setVelocity(velocityShooting);
                motorShooterRight.setVelocity(velocityShooting);
                servoLoaderAssist.setPower(1.0);
                resetTimer.reset();
                fixStep = "disable shooter";
            }
            else if(fixStep.equalsIgnoreCase("disable shooter") && resetTimer.milliseconds() >= 500)//300
            {
                motorShooterLeft.setVelocity(0);
                motorShooterRight.setVelocity(0);
                servoLoaderAssist.setPower(0.0);
                auxDpadDPressed = false;
                fixStep = "none";
            }
        }
        if(auxDpadU)
        {
            servoLoaderAssist.setPower(0.0);
            servoLoaderStartRight.setPower(0.0);
            servoLoaderStartLeft.setPower(0.0);
            //auxDpadDPressed = true;
        }

    }
    public void launch()
    {
        auxTriggerLeft = gamepad2.left_trigger;  // launch current
        auxTriggerRight = gamepad2.right_trigger;  // launch all
        auxX = gamepad2.x;
        if(!launchEngaged)
        {
            if (auxTriggerRight >= 0.3)
            {
                auxTriggerRightPressed = true;
            }
            if (auxTriggerLeft >= 0.3)
            {
                auxTriggerLeftPressed = true;
            }

            if(auxX)
            {
                shootingLong = true;
            }
            if(!auxX)
            {
                shootingLong = false;
            }

            if (auxTriggerLeft < 0.3 && auxTriggerLeftPressed)
            {
                launchEngaged = true;
                launchType = "current";
                auxTriggerLeftPressed = false;
                //shootingLong = false;
            }

            if (auxTriggerRight < 0.3 && auxTriggerRightPressed)
            {
                launchEngaged = true;
                launchType = "all";
                auxTriggerRightPressed = false;
                //shootingLong = false;
            }
        }

        if(launchEngaged)
        {
            if(launchStep.equalsIgnoreCase("none"))
            {
                robot.magazineEngaged = true;
                launchStep = "a1 - initial position";
            }
            else if (launchStep.equalsIgnoreCase("a1 - initial position") && !robot.magazineEngaged)
            {
                intakeMotor.setPower(0.0);

                servoLoaderAssist.setPower(1.0);
                servoLoaderStartRight.setPower(1.0);
                servoLoaderStartLeft.setPower(-1.0);
                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "a1 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a1 - lifter engaged") && stepTimer.milliseconds() > 300)//250
            {
                servoLifter.setPosition(lifterServoDown);
                if(!shootingLong)
                {
                    motorShooterLeft.setVelocity(velocityShooting);
                    motorShooterRight.setVelocity(velocityShooting);
                }
                if(shootingLong)
                {
                    motorShooterLeft.setVelocity(velocityLongShooting);
                    motorShooterRight.setVelocity(velocityLongShooting);
                }

                stepTimer.reset();
                launchStep = "a1 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a1 - drop lifter") && stepTimer.milliseconds() > 300)//350
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
                robot.magazineEngaged = true;
                launchStep = "a2 - get next artifact";
                stepTimer.reset();
            }
            else if (launchStep.equalsIgnoreCase("a2 - get next artifact") && !robot.magazineEngaged)
            {
                if(launchType.equalsIgnoreCase("current"))
                {
                    if(stepTimer.milliseconds() > 500)
                    {
                        servoLoaderAssist.setPower(0.0);
                        servoLoaderStartRight.setPower(0.0);
                        servoLoaderStartLeft.setPower(0.0);
                        motorShooterLeft.setVelocity(0);
                        motorShooterRight.setVelocity(0);

                        launchEngaged = false;
                        launchStep = "none";
                    }
                }
                else if(launchType.equalsIgnoreCase("all"))
                {
                    launchStep = "a2 - initial position";
                }
            }
            else if (launchStep.equalsIgnoreCase("a2 - initial position") && !robot.magazineEngaged)
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
            else if (launchStep.equalsIgnoreCase("a2 - lifter engaged") && stepTimer.milliseconds() > 300)
            {
                servoLifter.setPosition(lifterServoDown);
                if(!shootingLong)
                {
                    motorShooterLeft.setVelocity(velocityShooting);
                    motorShooterRight.setVelocity(velocityShooting);
                }
                if(shootingLong)
                {
                    motorShooterLeft.setVelocity(velocityLongShooting);
                    motorShooterRight.setVelocity(velocityLongShooting);
                }
                //motorShooterLeft.setVelocity(velocityShooting2);
                //motorShooterRight.setVelocity(velocityShooting2);

                stepTimer.reset();
                launchStep = "a2 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a2 - drop lifter") && stepTimer.milliseconds() > 300)
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
                robot.magazineEngaged = true;
                launchStep = "a3 - get next artifact";
            }
            else if (launchStep.equalsIgnoreCase("a3 - get next artifact") && !robot.magazineEngaged)
            {
                launchStep = "a3 - initial position";
            }
            else if (launchStep.equalsIgnoreCase("a3 - initial position") && !robot.magazineEngaged)
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
            else if (launchStep.equalsIgnoreCase("a3 - lifter engaged") && stepTimer.milliseconds() > 300)
            {
                servoLifter.setPosition(lifterServoDown);
                if(!shootingLong)
                {
                    motorShooterLeft.setVelocity(velocityShooting);
                    motorShooterRight.setVelocity(velocityShooting);
                }
                if(shootingLong)
                {
                    motorShooterLeft.setVelocity(velocityLongShooting);
                    motorShooterRight.setVelocity(velocityLongShooting);
                }
                //motorShooterLeft.setVelocity(velocityShooting3);
                //motorShooterRight.setVelocity(velocityShooting3);

                stepTimer.reset();
                launchStep = "a3 - turn off launcher";
            }
            else if (launchStep.equalsIgnoreCase("a3 - turn off launcher") && stepTimer.milliseconds() > 1000)
            {
                servoLoaderAssist.setPower(0.0);
                servoLoaderStartRight.setPower(0.0);
                servoLoaderStartLeft.setPower(0.0);
                motorShooterLeft.setVelocity(0);
                motorShooterRight.setVelocity(0);

                launchEngaged = false;
                launchStep = "none";
            }
        }
    }
    public void showTelemetry()
    {
        telemetry.addData("lly", llY);
        telemetry.addData("dist from goal", distanceToGoal);
        telemetry.addData("llx", llX);
        telemetry.addData("tracking speed", trackingSpeed);

        telemetry.addData("chamber control", chambers[0]);
        telemetry.addData("chamber barrel", chambers[1]);
        telemetry.addData("chamber expansion", chambers[2]);

        telemetry.addData("control hue", robot.chamberControlColor.getAnalysis().HSV[0]);
        telemetry.addData("barrel hue", robot.chamberBarrelColor.getAnalysis().HSV[0]);
        telemetry.addData("expansion hue", robot.chamberExpansionColor.getAnalysis().HSV[0]);

        telemetry.addData("control value", robot.chamberControlColor.getAnalysis().HSV[2]);
        telemetry.addData("barrel value", robot.chamberBarrelColor.getAnalysis().HSV[2]);
        telemetry.addData("expansion value", robot.chamberExpansionColor.getAnalysis().HSV[2]);
        /*
        telemetry.addData("control hub hue", robot.chamberControlColor.getAnalysis().HSV[0]);
        telemetry.addData("barrel hue", robot.chamberBarrelColor.getAnalysis().HSV[0]);
        telemetry.addData("expansion hub hue", robot.chamberExpansionColor.getAnalysis().HSV[0]);

        telemetry.addData("control hub saturation", robot.chamberControlColor.getAnalysis().HSV[1]);
        telemetry.addData("barrel saturation", robot.chamberBarrelColor.getAnalysis().HSV[1]);
        telemetry.addData("expansion hub saturation", robot.chamberExpansionColor.getAnalysis().HSV[1]);

        telemetry.addData("control hub total", robot.chamberControlColor.getAnalysis().HSV[2]);
        telemetry.addData("barrel total", robot.chamberBarrelColor.getAnalysis().HSV[2]);
        telemetry.addData("expansion hub total", robot.chamberExpansionColor.getAnalysis().HSV[2]);

         */



        telemetry.addData("Front Left Power", motorFrontLeft.getPower());
        telemetry.addData("CCW max", powerRotateCCWMax);
        telemetry.addData("CCW slow", powerRotateCCWSlow);

        telemetry.addData("Rotation X", rotationX);
        telemetry.addData("Rotation Y", rotationY);

        telemetry.addData("launch step", launchStep);
        telemetry.addData("odometry Y", bot.getY());
        telemetry.addData("odometry x", bot.getX());
        telemetry.addData("odometry", bot.getOrientationCurrent());
        telemetry.addData("radians", odometry.getHeading(AngleUnit.RADIANS));
        telemetry.addData("drive Left stick y", driveStickLeftY);
        telemetry.addData("drive Left stick x", driveStickLeftX);
        telemetry.addData("drive Right stick x", driveStickRightX);

        telemetry.addData("aux stick right Y", auxStickRightY);
        telemetry.addData("aux stick right X", auxStickRightX);
        telemetry.addData("aux stick left X", auxStickLeftX);
        telemetry.addData("aux stick left Y", auxStickLeftY);


        telemetry.addData("axon encoder", ((axonEncoder.getVoltage() / 3.3) * 360));
        //telemetry.addData("axon target", axonTargetPosition);
        //telemetry.addData("rotation direction", axonDirection);
        //telemetry.addData("axon direction", axon.getDirection());
        //telemetry.addData("test target", testAxon.getTargetRotation());
        //telemetry.addData("test angle", testAxon.log());
        //telemetry.addData("difference", ((axonEncoder.getVoltage() / 3.3) * 360) - testAxon.getCurrentAngle());
        telemetry.addData("engaged", robot.magazineEngaged);
        telemetry.addData("intake power", intakeMotor.getPower());
        telemetry.addData("magazine power", axon.getPower());
        telemetry.addData("motor velocity left", motorShooterLeft.getVelocity());
        telemetry.addData("motor velocity right", motorShooterRight.getVelocity());

        telemetry.update();
    }
    public void initialize2()
    {
        //imu = hardwareMap.get(IMU.class, "imu");
        //imu.initialize(parameters);

        servoLoaderStartLeft = hardwareMap.get(CRServo.class, "StartLeft");
        servoLoaderStartRight = hardwareMap.get(CRServo.class, "StartRight");
        servoLoaderAssist = hardwareMap.get(CRServo.class, "LoaderAssist");
        servoLifter = hardwareMap.get(Servo.class, "lifter");
        internalLight = hardwareMap.get(Servo.class, "InternalLight");

        axon = hardwareMap.get(CRServo.class, "axon");
        axonEncoder = hardwareMap.get(AnalogInput.class, "axonEncoder");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        //axon = hardwareMap.get(CRServo.class, "Magazine");

        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorShooterLeft = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        motorShooterRight = hardwareMap.get(DcMotorEx.class, "RightShooter");

        motorShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorShooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testTimer = new ElapsedTime();
        stepTimer = new ElapsedTime();
        axonTimer = new ElapsedTime();

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        odometry.setOffsets(170, 75, DistanceUnit.MM); // don't know how to get these offsets yet
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setYawScalar(-1.0);
        //odometry.resetPosAndIMU();
        bot = new Robot(odometry);
        bot.setMotors(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);
        bot.setSpeed(0.4);
        //bot.setOffsetIMU(imu);
        //bot.getIMUOffset();
        //bot.reset();


        //WebcamName webcamName = hardwareMap.get(WebcamName.class, "Internal Cam");
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());//not sure what this does or how it works...
        //testCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //testCam.setPipeline(new searchColor());
        //axonTargetPosition = -80;
        //testAxon.setRtp(true);
        internalLight.setPosition(0.4);//0.5

        servoLifter.setPosition(lifterServoDown);


        resetTimer = new ElapsedTime();
        robot = new ShooterBot();
        robot.setLaunchMotors(motorShooterLeft, motorShooterRight);
        robot.setMagazine(axon, axonEncoder);
        robot.setLoadServos(servoLifter, servoLoaderStartLeft, servoLoaderStartRight, servoLoaderAssist);
        robot.startCameraSensors();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors(robot.chamberControlColor, robot.chamberBarrelColor, robot.chamberExpansionColor)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCamera(hardwareMap.get(WebcamName.class, "Internal Cam"))
                .build();

        for(int i = 0; i < chambers.length; i++)
        {
            chambers[i] = "?";
        }

        //robot.initLimeLight();
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        robot.setLimelight(ll);
        robot.changePipeline(2);
        waitForStart();
        robot.startLimelight();
        //bot.getIMUOffset();
    }

    @Override
    public void runOpMode()
    {
        initialize2();
        robot.magazineEngaged = true;
        axonDirection = "cw";

        int axonState = 0;
        while(opModeIsActive())
        {
            if(axonState < 20)
            {
                servoLoaderAssist.setPower(1.0);
                servoLoaderStartRight.setPower(1.0);
                servoLoaderStartLeft.setPower(-1.0);
                axonState += 1;
            }
            else if(axonState == 20)
            {
                servoLoaderAssist.setPower(0.0);
                servoLoaderStartRight.setPower(0.0);
                servoLoaderStartLeft.setPower(0.0);
                axonState = 200;
            }

            fixShooter();
            //fixMagazine();
            if(!launchEngaged)
            {
                //drive();
                magazine();
                //intake();
                //shooter();
                //loading();
            }
            drive();
            trackAprilTag();
            getDistFromGoal();
            intake();
            launch();
            shootSpecific();
            showTelemetry();

            if(robot.magazineEngaged)
            {
                robot.axonToPosition(axonTargetPosition, axonDirection);
            }

            chambers = robot.getChambers();

            //testAxon.update();
            odometry.update();
        }
    }
}
