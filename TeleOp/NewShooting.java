package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class NewShooting extends LinearOpMode
{
    Servo turntable;
    CRServo servoLoaderStartLeft, servoLoaderStartRight, servoLoaderAssist;
    DcMotorEx motorShooterRight, motorShooterLeft;
    Servo liftControl;
    Servo liftExpansion;
    DcMotor motorShoot;
    boolean buttonA, buttonB, buttonX, buttonY, bumperLeft, bumperRight;
    boolean buttonAPressed, buttonBPressed, buttonXPressed, buttonYPressed, bumperLeftPressed, bumperRightPressed;
    double triggerLeft, triggerRight;

    boolean dpadUp, dpadDown, dpadUpPressed, dpadDownPressed;
    double incrementCW = 0.02755;//0.02745
    double incrementCCW = 0.02735; //0.02745
    double bump = 0.0010;
    double zeroPosition = 0.5;
    double currentPos = zeroPosition;
    int arrPos = 6;
    double[] servoPositions = new double[13];

    double velocityShooting = 1400;//1325

    double lifterDownPosition = 0.5;
    double liftControlUp = 0.71;
    double liftExpansionUp = 0.29;
    boolean launchEngaged = false;
    boolean TriggerRightPressed, TriggerLeftPressed = false;
    ElapsedTime stepTimer;
    boolean shootingLong;
    double velocityLongShooting = 1700;//1565, 1650

    String launchType = "";
    String launchStep = "None";
    Boolean safe = true;
    public void showTelemetry()
    {
        //telemetry.addData("shooter power", motorShoot.getPower());
        telemetry.addData("position", servoPositions[arrPos]);
        telemetry.addData("array position", arrPos);
        telemetry.addData("SAFE:", safe);
        telemetry.update();
    }
    public void rotate()
    {
        buttonA = gamepad1.a;//go to 1.0

        bumperLeft = gamepad1.left_bumper;// pos - 0.066 (0.2 * 0.33 = 0.066)
        bumperRight = gamepad1.right_bumper;// pos + 0.066

        dpadUp = gamepad1.dpad_up;
        dpadDown = gamepad1.dpad_down;

        if(bumperLeft)
        {
            bumperLeftPressed = true;
        }
        else if(!bumperLeft && bumperLeftPressed == true)//CCW
        {
            /*
            currentPos -= incrementCCW;
            turntable.setPosition(currentPos);
             */
            if(arrPos > 0)
            {
                arrPos -= 1;
            }
            bumperLeftPressed = false;
        }

        if(bumperRight)
        {
            bumperRightPressed = true;
        }
        else if(!bumperRight && bumperRightPressed == true)//CW
        {
            /*
            currentPos += incrementCW;
            turntable.setPosition(currentPos);
             */
            if(arrPos < 12)
            {
                arrPos += 1;
            }
            bumperRightPressed = false;
        }

        if(buttonA)
        {
            buttonAPressed = true;
        }
        else if(!buttonA && buttonAPressed == true)
        {
            arrPos = 6;
            buttonAPressed = false;
        }

        if(dpadDown)
        {
            dpadDownPressed = true;
        }
        else if(!dpadDown && dpadDownPressed == true)
        {
            currentPos -= bump;
            turntable.setPosition(currentPos);
            dpadDownPressed = false;
        }
        if(dpadUp)
        {
            dpadUpPressed = true;
        }
        else if(!dpadUp && dpadUpPressed == true)
        {
            currentPos += bump;
            turntable.setPosition(currentPos);
            dpadUpPressed = false;
        }

        turntable.setPosition(servoPositions[arrPos]);

        /*
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
         */
    }
    public void liftPositions()
    {
        buttonX = gamepad1.x;
        buttonY = gamepad1.y;

        if(buttonX)
        {
            buttonXPressed = true;
        }
        else if(!buttonX && buttonXPressed)
        {
            liftControl.setPosition(0.71);
            liftExpansion.setPosition(0.29);
            buttonXPressed = false;
        }

        if(buttonY)
        {
            buttonYPressed = true;
        }
        else if(!buttonY && buttonYPressed)
        {
            liftControl.setPosition(0.5);
            liftExpansion.setPosition(0.5);
            buttonYPressed = false;
        }
    }
    public void Shoot()
    {
        triggerLeft = gamepad1.left_trigger;  // launch current
        triggerRight = gamepad1.right_trigger;  // launch all
        //auxX = gamepad2.x;
        if(!launchEngaged)
        {
            if (triggerRight >= 0.3)
            {
                TriggerRightPressed = true;
            }
            if (triggerLeft >= 0.3)
            {
                TriggerLeftPressed = true;
            }

            if (triggerLeft < 0.3 && TriggerLeftPressed)
            {
                launchEngaged = true;
                launchType = "current";
                TriggerLeftPressed = false;
                //shootingLong = false;
            }

            if (triggerRight < 0.3 && TriggerRightPressed)
            {
                launchEngaged = true;
                launchType = "all";
                TriggerRightPressed = false;
                //shootingLong = false;
            }
        }

        if(launchEngaged)
        {
            if(launchStep.equalsIgnoreCase("none"))
            {
                launchStep = "a1 - initial position";
            }
            else if (launchStep.equalsIgnoreCase("a1 - initial position"))
            {
                motorShooterLeft.setVelocity(velocityShooting);
                motorShooterRight.setVelocity(velocityShooting);
                servoLoaderAssist.setPower(1.0);
                servoLoaderStartRight.setPower(1.0);
                servoLoaderStartLeft.setPower(-1.0);
                liftExpansion.setPosition(liftExpansionUp);
                liftControl.setPosition(liftControlUp);

                stepTimer.reset();
                launchStep = "a1 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a1 - lifter engaged") && stepTimer.milliseconds() > 300)//250
            {
                liftExpansion.setPosition(lifterDownPosition);
                liftControl.setPosition(lifterDownPosition);

                stepTimer.reset();
                launchStep = "a1 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a1 - drop lifter") && stepTimer.milliseconds() > 300)//350
            {
                arrPos += 1;
                launchStep = "a2 - get next artifact";
                stepTimer.reset();
            }
            else if (launchStep.equalsIgnoreCase("a2 - get next artifact") && stepTimer.milliseconds() > 250)
            {
                stepTimer.reset();
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
            else if (launchStep.equalsIgnoreCase("a2 - initial position"))
            {
                
                liftExpansion.setPosition(liftExpansionUp);
                liftControl.setPosition(liftControlUp);

                stepTimer.reset();
                launchStep = "a2 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a2 - lifter engaged") && stepTimer.milliseconds() > 300)
            {
                liftExpansion.setPosition(lifterDownPosition);
                liftControl.setPosition(lifterDownPosition);

                motorShooterLeft.setVelocity(velocityShooting);
                motorShooterRight.setVelocity(velocityShooting);


                //motorShooterLeft.setVelocity(velocityShooting2);
                //motorShooterRight.setVelocity(velocityShooting2);

                stepTimer.reset();
                launchStep = "a2 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a2 - drop lifter") && stepTimer.milliseconds() > 300)
            {
                arrPos += 1;
                stepTimer.reset();
                launchStep = "a3 - get next artifact";
            }
            else if (launchStep.equalsIgnoreCase("a3 - get next artifact") && stepTimer.milliseconds() > 250)
            {
                launchStep = "a3 - initial position";
            }
            else if (launchStep.equalsIgnoreCase("a3 - initial position"))
            {

                liftExpansion.setPosition(liftExpansionUp);
                liftControl.setPosition(liftControlUp);

                stepTimer.reset();
                launchStep = "a3 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a3 - lifter engaged") && stepTimer.milliseconds() > 300)
            {
                liftExpansion.setPosition(lifterDownPosition);
                liftControl.setPosition(lifterDownPosition);

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
    public void initialize2()
    {
        turntable = hardwareMap.get(Servo.class, "axon");
        liftControl = hardwareMap.get(Servo.class, "liftControl");
        liftExpansion = hardwareMap.get(Servo.class, "liftExpansion");

        servoLoaderStartLeft = hardwareMap.get(CRServo.class, "StartLeft");
        servoLoaderStartRight = hardwareMap.get(CRServo.class, "StartRight");
        servoLoaderAssist = hardwareMap.get(CRServo.class, "LoaderAssist");
        liftControl.setDirection(Servo.Direction.FORWARD);
        liftExpansion.setDirection(Servo.Direction.FORWARD);

        motorShooterLeft = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        motorShooterRight = hardwareMap.get(DcMotorEx.class, "RightShooter");

        motorShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorShooterRight.setDirection(DcMotorSimple.Direction.FORWARD);


        motorShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorShoot = hardwareMap.get(DcMotor.class, "shooter");
        servoPositions[0] = 0.3461;
        servoPositions[1] = 0.3695;
        servoPositions[2] = 0.3949;//0.3965, 0.39485
        servoPositions[3] = 0.4202;
        servoPositions[4] = 0.4463;
        servoPositions[5] = 0.4726;
        servoPositions[6] = 0.5;
        servoPositions[7] = 0.5299;//0.007
        servoPositions[8] = 0.5578;//0.003
        servoPositions[9] = 0.5874;//0.012
        servoPositions[10] = 0.6142;
        servoPositions[11] = 0.6427;
        servoPositions[12] = 0.6703;
        waitForStart();
        turntable.setPosition(currentPos);
        liftControl.setPosition(0.5);
        liftExpansion.setPosition(0.5);
    }

    @Override
    public void runOpMode()
    {
        initialize2();
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
            
            if(arrPos>10)
            {
                safe=false;
            }
            else if(arrPos<2)
            {
                safe=false;
            }
            else if(arrPos >= 2 && arrPos <= 10)
            {
                safe=true;
            }
            
            
            showTelemetry();
            rotate();
            Shoot();
        }
    }
}
