package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp
public class newTurnTable extends LinearOpMode
{
    Servo turntable;
    CRServo servoLoaderStartLeft, servoLoaderStartRight, servoLoaderAssist;
    DcMotorEx motorShooterRight, motorShooterLeft;
    Servo liftControl;
    Servo liftExpansion;
    DcMotor motorShoot;
    boolean buttonA, buttonB, buttonX, buttonY, bumperLeft, bumperRight;
    boolean buttonAPressed, buttonBPressed, buttonXPressed, buttonYPressed, bumperLeftPressed, bumperRightPressed;

    boolean dpadUp, dpadDown, dpadUpPressed, dpadDownPressed;
    double incrementCW = 0.02755;//0.02745
    double incrementCCW = 0.02735; //0.02745
    double bump = 0.0010;
    double zeroPosition = 0.5;
    double currentPos = zeroPosition;
    int arrPos = 6;
    double[] servoPositions = new double[13];

    double velocityShooting = 1400;//1325

    public void showTelemetry()
    {
        //telemetry.addData("shooter power", motorShoot.getPower());
        telemetry.addData("position", servoPositions[arrPos]);
        telemetry.addData("array position", arrPos);
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
    public void assistOn()
    {
        buttonB = gamepad1.b;

        if(buttonB)
        {
            buttonBPressed = true;
        }
        else if(!buttonB && buttonBPressed)
        {
            servoLoaderStartRight.setPower(-1.0);
            servoLoaderStartLeft.setPower(1.0);
            servoLoaderAssist.setPower(1.0);
            motorShooterLeft.setVelocity(velocityShooting);
            motorShooterRight.setVelocity(velocityShooting);
            buttonBPressed = false;
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
        while(opModeIsActive())
        {
            showTelemetry();
            rotate();
            liftPositions();
            assistOn();
        }
    }
}
