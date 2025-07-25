package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.hardware.EasyIMU;
import org.opencv.core.Mat;

public class Robot
{
    GoBildaPinpointDriver odometry;
    ElapsedTime timer = new ElapsedTime();
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorSlideLeft, motorSlideRight;
    DcMotorEx motorClawArm;
    Servo servoClawGrabber, servoClawExtend;
    TouchSensor buttonArmStop;
    IMU orientationIMU;

    public double brakePower = 0.1;
    double brakeActivePosition = 100;
    double slideSlowZone = 100;

    double globalSpeed = 0.5;

    double currentTarget = 0;

    public double test = 0.0;

    public double targetDistance = 0.0;

    public double distanceToTargetPrevious = 1000.0;
    public double distanceTraveledFromPrevious = 0.0;

    public double slidePositionPrevious = 0;
    public double slidePositionHold = 0;
    public double slidePowerHang = 0;

    public double currentDeg = 0.0;

    public double YStoppedPosition = 0.0;

    public double voltageStart = 0.0;
    public int rotateCorrections = 0;
    public int navCorrections = 0;

    public double navAcceleration = 0;

    public double orientationSlowDistance;

    public double teleOpIMUOrientation;
    public double teleOpIMUOffset;
    //public double odometryIMUResetPosition;


    public Robot(GoBildaPinpointDriver pod)
    {
        odometry = pod;
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
    }

    public Robot()
    {

    }

    //public void setOffsets(double x, double y)
    //{
    //    odometry.setOffsets(x, y);
    //}

    public void sleep(int milliseconds)
    {
        timer.reset();
        while(timer.milliseconds() < milliseconds)
        {
            //do nothing
        }
    }

    public void setOffsetIMU(IMU imu)
    {
        orientationIMU = imu;
        //teleOpIMUOffset = orientationIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void getIMUOffset()
    {
        teleOpIMUOffset = orientationIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getTeleOpIMURotation()
    {
        teleOpIMUOrientation = getRad();
        return teleOpIMUOrientation + teleOpIMUOffset;
    }


    public void setMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotorEx sl, DcMotorEx sr, DcMotorEx arm)
    {
        motorFrontLeft = fl;
        motorFrontRight = fr;
        motorBackLeft = bl;
        motorBackRight = br;

        motorSlideLeft = sl;
        motorSlideRight = sr;
        motorClawArm = arm;
    }

    public void setServos(Servo cg, Servo ce)
    {
        servoClawGrabber = cg;
        servoClawExtend = ce;
    }

    public void setButton(TouchSensor bt)
    {
        buttonArmStop = bt;
    }

    public void setSpeed(double speed)
    {
        globalSpeed = speed;
    }

    public void resetSlideAndArm()
    {
        boolean buttonAlreadyPressed = false;
        while(buttonArmStop.isPressed())
        {
            motorClawArm.setPower(0.5);
            buttonAlreadyPressed = true;
        }
        if(buttonAlreadyPressed)
        {
            sleep(100);
        }
        motorClawArm.setPower(0.0);

        motorSlideLeft.setPower(0.70);
        motorSlideRight.setPower(0.70);
        sleep(300);
        motorSlideLeft.setPower(0.0);
        motorSlideRight.setPower(0.0);
        while(!buttonArmStop.isPressed())
        {
            motorClawArm.setPower(-0.2);//-0.15
        }
        motorClawArm.setPower(0.0);
        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorClawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double[] navRotate(double orientationTarget, String rotationDirection, double rotationErrorTolerance)
    {
        double orientationCurrent;
        double distanceToTarget = 0.0;

        if(orientationTarget >= 0) // this value is set to -1.0 to end rotation
        {
            odometry.update();

            //double errorTolerance = 2; // rotation stops if +- this value in degrees
            double distanceToSlow = 40; // slow rotational speed on linear function if below this value

            double rotatePowerMin = 0.15; // set the minimum rotation speed
            if (voltageStart > 13)
            {
                rotatePowerMin = 0.1;
            }
            else if (voltageStart >= 12.8)
            {
                rotatePowerMin = 0.13;
            }
            if (rotationErrorTolerance > 2.0)
            {
                rotatePowerMin = 0.15;
            }
            orientationCurrent = getOrientationCurrent();  //function call - converts value to 0 - 359.99

            double rotatePowerMax = 1.0;
            double rotatePower = rotatePowerMax;
            distanceToTarget = orientationTarget - orientationCurrent;

            // distanceToTarget gets converted to values between +180 to -180
            if (distanceToTarget < -180)
            {
                distanceToTarget = distanceToTarget + 360;
            }
            else if (distanceToTarget > 180)
            {
                distanceToTarget = distanceToTarget - 360;
            }

            if (Math.abs(distanceToTarget) <= rotationErrorTolerance)
            {
                // stop rotation if still within error after short sleep
                rotationDirection = "none";
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);
                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                sleep(100);
                odometry.update();
                distanceToTarget = orientationTarget - getOrientationCurrent();

                // distanceToTarget gets converted to values between +180 to -180
                if (distanceToTarget < -180)
                {
                    distanceToTarget = distanceToTarget + 360;
                }
                else if (distanceToTarget > 180)
                {
                    distanceToTarget = distanceToTarget - 360;
                }
                if (Math.abs(distanceToTarget) <= rotationErrorTolerance || rotateCorrections >= 2)
                {
                    orientationTarget = -1.0;
                    //test = 1;
                }
                else
                {
                    rotatePower = rotatePowerMin;
                }
            }
            else if (Math.abs(distanceToTarget) < 8.0)
            {
                // slow when close to target
                rotationDirection = "none";
                rotatePower = rotatePowerMin;
                if (Math.abs(distanceToTarget) < rotationErrorTolerance + 1)
                {
                    rotateCorrections++;
                }
                else
                {
                    rotateCorrections = 0;
                }
            }
            else if (Math.abs(distanceToTarget) <= distanceToSlow)
            {
                if ((Math.abs(odometry.getHeadingVelocity() * (180 / Math.PI)) < 5) && Math.abs(distanceToTarget) > 10)
                {
                    rotatePower = 1.0;
                }
                else
                {
                    rotatePowerMax = 0.5;
                }
                rotationDirection = "none";
                // slow rotation using linear function based on distanceToTarget
                if (Math.abs(odometry.getHeadingVelocity() * (180 / Math.PI)) > 120)
                {
                    rotatePower = 0;
                }
                else
                {
                    if (Math.abs(odometry.getHeadingVelocity() * (180 / Math.PI)) < 5)
                    {
                        rotatePowerMin = 0.15;
                    }
                    rotatePower = ((rotatePowerMax - rotatePowerMin) * (Math.abs(distanceToTarget) / distanceToSlow)) + rotatePowerMin;
                }
            }
            else
            {
                //test = -1;
            }

            if(orientationTarget >= 0) // this value is set to -1.0 to end rotation
            {
                if ((distanceToTarget < 0 && rotationDirection == "none") || rotationDirection == "counterclockwise")
                {
                    // rotate counterclockwise
                    rotatePower = rotatePower * -1.0;
                }

                motorFrontLeft.setPower(-rotatePower);
                motorFrontRight.setPower(rotatePower);
                motorBackLeft.setPower(-rotatePower);
                motorBackRight.setPower(rotatePower);

                //test = 0;
            }
        }
        double[] rotateReturn = new double[2];
        rotateReturn[0] = distanceToTarget;
        rotateReturn[1] = orientationTarget;
        return rotateReturn;
    }

    public double slideToPosition(double slideTarget)
    {
        double slideCurrent = motorSlideLeft.getCurrentPosition();
        double errorTolerance = 20;
        double distanceToSlow = 100;

        if (slideTarget > -1)
        {
            if (Math.abs(slideTarget - slideCurrent) <= errorTolerance)
            {
                motorSlideLeft.setPower(brakePower * -1.0);
                motorSlideRight.setPower(brakePower * -1.0);
                slideTarget = -1;
            }
            else if (Math.abs(slideTarget - slideCurrent) <= distanceToSlow)
            {
                if (slideTarget < slideCurrent) //move down
                {
                    if(buttonArmStop.isPressed())
                    {
                        motorClawArm.setPower(1.0);
                    }
                    else
                    {
                        motorClawArm.setPower(0.0);
                    }
                    if (slideCurrent < 100)
                    {
                        motorSlideLeft.setPower(0.05);
                        motorSlideRight.setPower(0.05);
                    }
                    else
                    {
                        motorSlideLeft.setPower(0.0 + ((Math.abs(slideTarget - slideCurrent) / 100) * 0.2));
                        motorSlideRight.setPower(0.0 + ((Math.abs(slideTarget - slideCurrent) / 100) * 0.2));
                    }
                }
                else
                {
                    motorSlideLeft.setPower(-0.7 + ((Math.abs(slideTarget - slideCurrent) / 100) * 0.5));
                    motorSlideRight.setPower(-0.7 + ((Math.abs(slideTarget - slideCurrent) / 100) * 0.5));
                    //test = 1;
                }
            }
            else
            {
                if (slideTarget < slideCurrent) //move down
                {
                    if(buttonArmStop.isPressed())
                    {
                        motorClawArm.setPower(1.0);
                    }
                    else
                    {
                        motorClawArm.setPower(0.0);
                    }
                    motorSlideLeft.setPower(0.4);
                    motorSlideRight.setPower(0.4);
                    //test = -2;
                }
                else
                {
                    motorSlideLeft.setPower(-0.8);
                    motorSlideRight.setPower(-0.8);
                    //test = 2;
                }

            }

        }
        else if (slideTarget > -2) // gamepad2.left_stick_y engaged
        {
            if (motorSlideLeft.getCurrentPosition() >= brakeActivePosition)
            {
                motorSlideLeft.setPower(brakePower * -1.0);
                motorSlideRight.setPower(brakePower * -1.0);
                //test = -10;
            }
        }



        return slideTarget;
    }

    public double slideToPositionHang(double slideTarget, String slideDirection)
    {
        double slideCurrent = motorSlideLeft.getCurrentPosition();
        double errorTolerance = 20;
        double distanceToSlow = 100;

        //test = 9999;

        if (slideTarget > -1000)
        {
            if (slideDirection == "down")
            {
                // Lifting the robot
                if (slideCurrent <= slideTarget)
                {
                    motorSlideLeft.setPower(brakePower);
                    motorSlideRight.setPower(brakePower);
                    slideTarget = -1000;
                }
                else
                {
                    if (slideCurrent - slidePositionPrevious <= 0)
                    {
                        // Add power if not moving down;
                        slidePowerHang = slidePowerHang + 0.01;
                        test = 12345;
                    }
                    motorSlideLeft.setPower(slidePowerHang);
                    motorSlideRight.setPower(slidePowerHang);
                }
            }
            else
            {
                // Moving slide up
                if (slideCurrent >= slideTarget)
                {
                    motorSlideLeft.setPower(brakePower);
                    motorSlideRight.setPower(brakePower);
                    slideTarget = -1000;
                }
                else
                {
                    if (slideCurrent - slidePositionPrevious >= 0)
                    {
                        // Add power if not moving up;
                        slidePowerHang = slidePowerHang - 0.01;
                    }
                    motorSlideLeft.setPower(slidePowerHang);
                    motorSlideRight.setPower(slidePowerHang);
                }
            }

        }
        else
        {
            // Dynamic Braking
            // - is up and + is down
            if (slideCurrent - slidePositionPrevious > 5)
            {
                // Moving down fast
                brakePower = brakePower + 0.1;
            }
            else if (slideCurrent - slidePositionPrevious > 0)
            {
                // Moving down slow
                brakePower = brakePower + 0.01;
            }
            else if (slideCurrent - slidePositionPrevious < 0)
            {
                // Moving up slow
                brakePower = brakePower - 0.01;
            }
            else if (slideCurrent - slidePositionPrevious < -5)
            {
                // Moving up fast
                brakePower = brakePower - 0.1;
            }

            motorSlideLeft.setPower(brakePower);
            motorSlideRight.setPower(brakePower);
        }

        slidePositionPrevious = slideCurrent;

        return slideTarget;
    }

    public double armToPosition(double armTarget, String armDirection)
    {
        double armCurrent = motorClawArm.getCurrentPosition();
        double errorTolerance = 20;
        double armPower = 1.0;
        int distanceToSlow = 400;

        if (armTarget > -10000)
        {
            if(armDirection.equalsIgnoreCase("up"))
            {
                if ((armTarget - armCurrent) <= errorTolerance)
                {
                    motorClawArm.setPower(0.0);
                    armTarget = -10000;
                }
                else if ((armTarget - armCurrent) <= distanceToSlow)
                {
                    armPower = 0.5;
                    motorClawArm.setPower(0.25 + ((Math.abs(armTarget - armCurrent) / distanceToSlow) * armPower));
                }
                else
                {
                    if (armTarget > armCurrent) //move up
                    {
                        motorClawArm.setPower(armPower);
                    }
                }
            }
            else// move down
            {
                if ((armCurrent - armTarget) <= errorTolerance)
                {
                    motorClawArm.setPower(0.0);
                    armTarget = -10000;
                }
                else if ((armCurrent - armTarget) <= distanceToSlow)
                {
                    if(armCurrent < 100)
                    {
                        armPower = 0.15;
                    }
                    else if(armCurrent < 300)
                    {
                        armPower = 0.2;
                    }
                    else if(armCurrent < 500)
                    {
                        armPower = 0.3;
                    }
                    else if(armCurrent < 500)
                    {
                        armPower = 0.3;
                    }
                    else
                    {
                        armPower = 0.4;
                    }
                    motorClawArm.setPower(-0.15 - ((Math.abs(armTarget - armCurrent) / distanceToSlow) * armPower));
                }
                else
                {
                    if(armCurrent < 100)
                    {
                        armPower = 0.15;
                    }
                    else if(armCurrent < 300)
                    {
                        armPower = 0.2;
                    }
                    else if(armCurrent < 500)
                    {
                        armPower = 0.3;
                    }
                    else if(armCurrent < 1000)
                    {
                        armPower = 0.6;
                    }
                    else
                    {
                        armPower = 1;
                    }
                    motorClawArm.setPower(armPower * (-1.0));
                }
            }
        }



        else
        {
            //motorClawArm.setPower(0.0);
            armTarget = -10000;
        }

        return armTarget;
    }

    public double armToPositionHang(double armTarget, String armDirection)
    {
        double armCurrent = motorClawArm.getCurrentPosition();
        double errorTolerance = 30;
        double armPower = 0.6;
        int distanceToSlow = 200;

        if (armTarget > -10000)
        {
            if (armDirection == "down")
            {
                armPower = -0.8;
                if (armCurrent <= armTarget)
                {
                    motorClawArm.setPower(0.0);
                    armTarget = -10000;
                }
                else
                {
                    motorClawArm.setPower(armPower);
                }
            }
            else if (armDirection == "up")
            {
                armPower = 0.4;
                if (armCurrent >= armTarget)
                {
                    motorClawArm.setPower(0.0);
                    armTarget = -10000;
                }
                else
                {
                    motorClawArm.setPower(armPower);
                }
            }
        }
        else
        {
            motorClawArm.setPower(0.0);
            armTarget = -10000;
        }

        return armTarget;
    }

    public double navToXPosition(double positionXTarget, double positionOrientationTarget)
    {
        double distanceToTarget = 0.0;

        if (positionXTarget > -10000) // -10000 means no target
        {
            odometry.update();

            double powerMax = 1.0;
            double powerMin = 0.25;

            /*
            if (voltageStart > 13.2)
            {
                powerMax = 0.90;
                powerMin = 0.2;
            }
            else if (voltageStart > 13)
            {
                powerMax = 0.94;
                powerMin = 0.21;
            }
            else if (voltageStart >= 12.8)
            {
                powerMax = 0.97;
                powerMin = 0.23;
            }
             */

            double positionXCurrent = getX();

            double distanceToSlow = 200;
            double errorTolerance = 20;

            double distanceToTargetX = positionXTarget - positionXCurrent;

            distanceToTarget = Math.abs(positionXTarget - positionXCurrent);

            if (distanceToTargetPrevious == 10000)
            {
                distanceToTargetPrevious = distanceToTargetX;
                navAcceleration = 0;
            }

            navAcceleration = navAcceleration + 0.1;
            if(navAcceleration < 1)
            {
                powerMax = powerMax * navAcceleration;
            }

            double motorPower = powerMax;

            distanceTraveledFromPrevious = Math.abs(distanceToTargetX - distanceToTargetPrevious);
            distanceToTargetPrevious = distanceToTargetX;

            if (distanceToTarget < distanceToSlow)
            {
                powerMax = 0.4;
                double powerUsed = powerMin;
                // slow down

                if (distanceToTarget < 100)
                {
                    motorPower = powerUsed;
                }
                else
                {
                    motorPower = ((powerMax - powerUsed) * (distanceToTarget / distanceToSlow)) + powerUsed;
                }
            }

            if (distanceToTarget > errorTolerance)
            {
                if (distanceTraveledFromPrevious > 15 && distanceToTarget < distanceToSlow)
                {
                    motorPower = -0.1;
                }

                if (Math.abs(distanceToTarget) < errorTolerance + 10)
                {
                    navCorrections++;
                }
                else
                {
                    navCorrections = 0;
                }

                if (distanceToTargetX < 0)
                {
                    motorPower = motorPower * -1.0;
                }

                motorFrontLeft.setPower(motorPower * -1);
                motorFrontRight.setPower(motorPower);
                motorBackLeft.setPower(motorPower);
                motorBackRight.setPower(motorPower * -1);

            }
            else
            {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                sleep(100);

                odometry.update();
                positionXCurrent = getX();
                distanceToTarget = Math.abs(positionXTarget - positionXCurrent);
                if (distanceToTarget < errorTolerance || navCorrections >= 3)
                {

                    positionXTarget = -10000.0;
                    navCorrections = 0;
                }
            }
        }

        return positionXTarget;
    }


    public double navToYPosition(double positionYTarget, double positionOrientationTarget)
    {
        double distanceToTarget = 0.0;

        if (positionYTarget > -10000) // -10000 means no target
        {
            odometry.update();

            double powerMax = 0.9;
            double powerMin = 0.20;

            if (voltageStart > 13.2)
            {
                powerMax = 0.75;
                powerMin = 0.13;
            }
            else if (voltageStart > 13)
            {
                powerMax = 0.8;
                powerMin = 0.15;
            }
            else if (voltageStart >= 12.8)
            {
                powerMax = 0.85;
                powerMin = 0.18;
            }

            double positionYCurrent = getY();

            double distanceToSlow = 200;
            double errorTolerance = 15;//20

            double distanceToTargetY = positionYTarget - positionYCurrent;

            distanceToTarget = Math.abs(positionYTarget - positionYCurrent);

            if (distanceToTargetPrevious == 10000)
            {
                distanceToTargetPrevious = distanceToTargetY;
                navAcceleration = 0;
            }

            navAcceleration = navAcceleration + 0.1;
            if(navAcceleration < 1)
            {
                powerMax = powerMax * navAcceleration;
            }

            double motorPower = powerMax;

            distanceTraveledFromPrevious = Math.abs(distanceToTargetY - distanceToTargetPrevious);
            distanceToTargetPrevious = distanceToTargetY;

            if (distanceToTarget < distanceToSlow)
            {
                if(powerMax > 0.4)
                {
                    powerMax = 0.4;
                }
                double powerUsed = powerMin;
                // slow down

                if (distanceToTarget < 100)
                {
                    motorPower = powerUsed;
                }
                else
                {
                    motorPower = ((powerMax - powerUsed) * (distanceToTarget / distanceToSlow)) + powerUsed;
                }
            }

            if (distanceToTarget > errorTolerance)
            {
                if (distanceTraveledFromPrevious > 15 && distanceToTarget < distanceToSlow)
                {
                    motorPower = -0.1;
                }

                if (Math.abs(distanceToTarget) < errorTolerance + 10)
                {
                    navCorrections++;
                }
                else
                {
                    navCorrections = 0;
                }

                if (distanceToTargetY < 0)
                {
                    motorPower = motorPower * -1.0;
                }

                motorFrontLeft.setPower(motorPower * -1.0);
                motorFrontRight.setPower(motorPower * -1.0);
                motorBackLeft.setPower(motorPower * -1.0);
                motorBackRight.setPower(motorPower * -1.0);

            }
            else
            {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                sleep(100);
                odometry.update();
                positionYCurrent = getY();
                distanceToTarget = Math.abs(positionYTarget - positionYCurrent);
                if (distanceToTarget < errorTolerance || navCorrections >= 3)
                {
                    positionYTarget = -10000.0;
                    navCorrections = 0;
                }
            }
        }

        return positionYTarget;
    }

    public double[] navToPosition(double navXTarget, double navYTarget, boolean navPrecise, boolean navEngaged  )
    {
        odometry.update();

        double distanceToTarget = 0.0;
        double currentX = getX();
        double currentY = getY();
        double distanceToNavSlow = 300;
        double navErrorTolerance = 15;

        double powerMaxNav = 0.8;
        double powerMinNav = 0.20;
        double powerMotorNav;

        if (voltageStart > 13)
        {
            //powerMinRotate = 0.1;
        }
        else if (voltageStart >= 12.8)
        {
            //powerMinRotate = 0.13;
        }

        double bearingTarget = 0.0;
        double valueX = 0.0;
        double valueY = 0.0;
        double currentRotation = getTeleOpIMURotation();

        if (navEngaged)
        {
            double radiansAdd = 0.0;
            double valueRotation = 0;
            if ((navXTarget - currentX) < 0 && (navYTarget - currentY) < 0)
            {
                radiansAdd = Math.toRadians(180);
            }
            else if ((navXTarget - currentX) >= 0 && (navYTarget - currentY) < 0)
            {
                radiansAdd = Math.toRadians(180);
            }

            if ((navXTarget - currentX) >= 0 && (navYTarget - currentY) == 0)
            {
                bearingTarget = Math.toRadians(90);
            }
            else if ((navXTarget - currentX) < 0 && (navYTarget - currentY) == 0)
            {
                bearingTarget = Math.toRadians(270);
            }
            else
            {
                bearingTarget = Math.atan((navXTarget - currentX) / (navYTarget - currentY)) + radiansAdd;
            }

            valueX = Math.sin(bearingTarget);
            valueY = Math.cos(bearingTarget);

            if (valueX >= valueY && valueY != 0)
            {
                valueY = valueY / Math.abs(valueX);
                valueX = valueX / Math.abs(valueX);
            }
            else if (valueY > valueX && valueX != 0)
            {
                valueX = valueX / Math.abs(valueY);
                ;
                valueY = valueY / Math.abs(valueY);
            }


            distanceToTarget = Math.abs(Math.sqrt((Math.pow((navXTarget - currentX), 2)) + (Math.pow((navYTarget - currentY), 2))));

            if (distanceToTargetPrevious == 10000)
            {
                distanceToTargetPrevious = distanceToTarget;
                navAcceleration = 0;
            }

            navAcceleration = navAcceleration + 0.1;
            if (navAcceleration < 1)
            {
                powerMaxNav = powerMaxNav * navAcceleration;
            }

            distanceTraveledFromPrevious = Math.abs(distanceToTarget - distanceToTargetPrevious);
            distanceToTargetPrevious = distanceToTarget;

            powerMotorNav = powerMaxNav;

            if (distanceToTarget < distanceToNavSlow)
            {
                powerMaxNav = 0.5;
                double powerUsed = powerMinNav;
                // slow down

                if (distanceToTarget < 100)
                {
                    powerMotorNav = powerMinNav;
                }
                else
                {
                    powerMotorNav = ((powerMaxNav - powerMinNav) * (distanceToTarget / distanceToNavSlow)) + powerMinNav;
                }
            }

            if (distanceToTarget > navErrorTolerance) // && Math.abs(distanceToTargetOrientation) > rotationErrorTolerance)
            {
                if (distanceTraveledFromPrevious > 15 && distanceToTarget < distanceToNavSlow)
                {
                    powerMotorNav = 0.0;
                }

                if (distanceToTarget < navErrorTolerance + 10)
                {
                    navCorrections++;
                }
                else
                {
                    navCorrections = 0;
                }

                double rotationX = (valueX * Math.cos(-currentRotation) - valueY * Math.sin(-currentRotation)) * -1.0;
                double rotationY = (valueX * Math.sin(-currentRotation) + valueY * Math.cos(-currentRotation)) * -1.0;

                //rotationX = rotationX * 1.1;

                double denominator = Math.max(Math.abs(rotationY) + Math.abs(rotationX) + Math.abs(valueRotation), 1);
                motorFrontLeft.setPower(((rotationY + rotationX + valueRotation) / denominator) * powerMotorNav);
                motorFrontRight.setPower(((rotationY - rotationX - valueRotation) / denominator) * powerMotorNav);
                motorBackLeft.setPower(((rotationY - rotationX + valueRotation) / denominator) * powerMotorNav);
                motorBackRight.setPower(((rotationY + rotationX - valueRotation) / denominator) * powerMotorNav);
            }
            else
            {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                sleep(100);

                odometry.update();
                currentX = getX();
                currentY = getY();

                distanceToTarget = Math.abs(Math.sqrt((Math.pow((navXTarget - currentX), 2)) + (Math.pow((navYTarget - currentY), 2))));

                if ((distanceToTarget <= navErrorTolerance) || navCorrections >= 3)
                {

                    navEngaged = false;
                    navCorrections = 0;
                }
            }
        }
        test = bearingTarget;

        double[] navReturn = new double[3];
        if(navEngaged)
        {
            navReturn[0] = 1;
        }
        else
        {
            navReturn[0] = -1;
        }
        navReturn[1] = distanceToTarget;
        navReturn[2] = 0.0;
        return navReturn;
    }

    public double[] navToPositionAndRotate(double navXTarget, double navYTarget, double navOrientationTarget,
                                  boolean navOnlyRotate, boolean navPrecise, boolean navEngaged  )
    {
        odometry.update();

        double distanceToTarget = 0.0;
        double distanceToTargetOrientation = 0.0;
        double currentOrientation = getOrientationCurrent();
        double currentX = getX();
        double currentY = getY();
        double rotationErrorTolerance = 2.0;
        double distanceToRotationSlow = 40; // slow rotational speed on linear function if below this value
        double distanceToNavSlow = 200;
        double navErrorTolerance = 20;

        double powerMaxRotate = 1.0;
        double powerMinRotate = 0.20;
        double powerMaxNav = 1.0;
        double powerMinNav = 0.25;
        double powerMotorNav = powerMaxNav;
        if (voltageStart > 13)
        {
            powerMinRotate = 0.2;
        }
        else if (voltageStart >= 12.8)
        {
            powerMinRotate = 0.22;
        }

        double bearingTarget = 0.0;
        double valueX = 0.0;
        double valueY = 0.0;
        double valueRotation = 0.0;
        double currentRotation = getTeleOpIMURotation();
        double rotationDirection = 1.0;

        if (navEngaged)
        {
            distanceToTargetOrientation = navOrientationTarget - currentOrientation;

            if(distanceToTargetOrientation < -180)
            {
                distanceToTargetOrientation += 360;
            }
            else if(distanceToTargetOrientation > 180)
            {
                distanceToTargetOrientation -= 360;
            }

            if(distanceToTargetOrientation < 0)
            {
                rotationDirection = -1.0;
            }

            if (Math.abs(distanceToTargetOrientation) <= rotationErrorTolerance)
            {
                if(navOnlyRotate)
                {
                    // stop rotation if still within error after short sleep
                    motorFrontLeft.setPower(0.0);
                    motorFrontRight.setPower(0.0);
                    motorBackLeft.setPower(0.0);
                    motorBackRight.setPower(0.0);
                    sleep(100);
                    odometry.update();
                    distanceToTargetOrientation = navOrientationTarget - getOrientationCurrent();

                    // distanceToTarget gets converted to values between +180 to -180
                    if (distanceToTargetOrientation < -180)
                    {
                        distanceToTargetOrientation += 360;
                    }
                    else if (distanceToTargetOrientation > 180)
                    {
                        distanceToTargetOrientation -= 360;
                    }

                    if (distanceToTargetOrientation > 0)
                    {
                        rotationDirection = -1.0;
                    }
                }
                if (Math.abs(distanceToTargetOrientation) <= rotationErrorTolerance)
                {
                    valueRotation = 0.0;
                    if(navOnlyRotate)
                    {
                        motorFrontLeft.setPower(0);
                        motorFrontRight.setPower(0);
                        motorBackLeft.setPower(0);
                        motorBackRight.setPower(0);
                        navEngaged = false;
                    }
                }
                else
                {
                    valueRotation = powerMinRotate * rotationDirection;
                }
            }
            else if (Math.abs(distanceToTargetOrientation) < 8.0)
            {
                // slow when close to target
                valueRotation = powerMinRotate * rotationDirection;
            }
            else if (Math.abs(distanceToTargetOrientation) <= distanceToRotationSlow)
            {
                if ((Math.abs(odometry.getHeadingVelocity() * (180 / Math.PI)) < 5) && Math.abs(distanceToTargetOrientation) > 10)
                {
                    powerMaxRotate = 0.8;
                }
                else
                {
                    powerMaxRotate = 0.5;
                }

                // slow rotation using linear function based on distanceToTarget
                if (Math.abs(odometry.getHeadingVelocity() * (180 / Math.PI)) > 120)
                {
                    valueRotation = 0;
                }
                else
                {
                    if (Math.abs(odometry.getHeadingVelocity() * (180 / Math.PI)) < 5)
                    {
                        powerMinRotate = 0.15;
                    }
                    valueRotation = (((powerMaxRotate - powerMinRotate) *
                            (Math.abs(distanceToTargetOrientation) / distanceToRotationSlow)) + powerMinRotate) * rotationDirection;
                }
            }
            else
            {
                valueRotation = powerMaxRotate * rotationDirection;
            }

            if(navOnlyRotate)
            {
                motorFrontLeft.setPower((valueRotation) * -1);
                motorFrontRight.setPower((valueRotation));
                motorBackLeft.setPower((valueRotation) * -1);
                motorBackRight.setPower((valueRotation));
            }
            else
            {
                double radiansAdd = 0.0;
                if ((navXTarget - currentX) < 0 && (navYTarget - currentY) < 0)
                {
                    radiansAdd = Math.toRadians(180);
                }
                else if ((navXTarget - currentX) >= 0 && (navYTarget - currentY) < 0)
                {
                    radiansAdd = Math.toRadians(180);
                }

                if ((navXTarget - currentX) >= 0 && (navYTarget - currentY) == 0)
                {
                    bearingTarget = Math.toRadians(90);
                }
                else if ((navXTarget - currentX) < 0 && (navYTarget - currentY) == 0)
                {
                    bearingTarget = Math.toRadians(270);
                }
                else
                {
                    bearingTarget = Math.atan((navXTarget - currentX) / (navYTarget - currentY)) + radiansAdd;
                }

                valueX = Math.sin(bearingTarget);
                valueY = Math.cos(bearingTarget);

                if (valueX >= valueY && valueY != 0)
                {
                    valueY = valueY / Math.abs(valueX);
                    valueX = valueX / Math.abs(valueX);
                }
                else if (valueY > valueX && valueX != 0)
                {
                    valueX = valueX / Math.abs(valueY);
                    ;
                    valueY = valueY / Math.abs(valueY);
                }

                distanceToTarget = Math.abs(Math.sqrt((Math.pow((navXTarget - currentX), 2)) + (Math.pow((navYTarget - currentY), 2))));

                if (distanceToTargetPrevious == 10000)
                {
                    distanceToTargetPrevious = distanceToTarget;
                    navAcceleration = 0;
                }

                navAcceleration = navAcceleration + 0.1;
                if (navAcceleration < 1)
                {
                    powerMaxNav = powerMaxNav * navAcceleration;
                }

                distanceTraveledFromPrevious = Math.abs(distanceToTarget - distanceToTargetPrevious);

                powerMotorNav = powerMaxNav;

                if (distanceToTarget < distanceToNavSlow)
                {
                    if(navPrecise)
                    {
                        powerMaxNav = 0.5;
                        double powerUsed = powerMinNav;
                        // slow down

                        if (distanceToTarget < 100)
                        {
                            powerMotorNav = powerMinNav;
                        }
                        else
                        {
                            powerMotorNav = ((powerMaxNav - powerMinNav) * (distanceToTarget / distanceToNavSlow)) + powerMinNav;
                        }
                    }
                }

                if(!navPrecise && (distanceToTarget <= navErrorTolerance + 20))
                {
                    //motorFrontLeft.setPower(0);
                    //motorFrontRight.setPower(0);
                    //motorBackLeft.setPower(0);
                    //motorBackRight.setPower(0);
                    navEngaged = false;
                }
                else if (distanceToTarget > navErrorTolerance || Math.abs(distanceToTargetOrientation) > rotationErrorTolerance)
                {
                    if(navPrecise)
                    {
                        if (distanceTraveledFromPrevious > 15 && distanceToTarget < distanceToNavSlow)
                        {
                            powerMotorNav = 0.0;
                        }

                        if (distanceToTarget < navErrorTolerance + 10)
                        {
                            navCorrections++;
                        }
                        else
                        {
                            navCorrections = 0;
                        }
                    }

                    double rotationX = (valueX * Math.cos(-currentRotation) - valueY * Math.sin(-currentRotation)) * -1.0;
                    double rotationY = (valueX * Math.sin(-currentRotation) + valueY * Math.cos(-currentRotation)) * -1.0;

                    //rotationX = rotationX * 1.1;

                    if(distanceToTarget <= navErrorTolerance && Math.abs(distanceToTargetOrientation) > rotationErrorTolerance)
                    {
                        motorFrontLeft.setPower((valueRotation) * -1);
                        motorFrontRight.setPower((valueRotation));
                        motorBackLeft.setPower((valueRotation) * -1);
                        motorBackRight.setPower((valueRotation));
                    }
                    else
                    {
                        double denominator = Math.max(Math.abs(rotationY) + Math.abs(rotationX) + Math.abs(valueRotation), 1);
                        motorFrontLeft.setPower(((rotationY + rotationX - valueRotation) / denominator) * powerMotorNav);
                        motorFrontRight.setPower(((rotationY - rotationX + valueRotation) / denominator) * powerMotorNav);
                        motorBackLeft.setPower(((rotationY - rotationX - valueRotation) / denominator) * powerMotorNav);
                        motorBackRight.setPower(((rotationY + rotationX + valueRotation) / denominator) * powerMotorNav);
                    }
                }
                else
                {
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    sleep(100);

                    odometry.update();
                    currentOrientation = getOrientationCurrent();
                    currentX = getX();
                    currentY = getY();

                    distanceToTarget = Math.abs(Math.sqrt((Math.pow((navXTarget - currentX), 2)) + (Math.pow((navYTarget - currentY), 2))));
                    distanceToTargetOrientation = navOrientationTarget - currentOrientation;
                    if (distanceToTargetOrientation < -180)
                    {
                        distanceToTargetOrientation = distanceToTargetOrientation + 360;
                    }
                    else if (distanceToTargetOrientation > 180)
                    {
                        distanceToTargetOrientation = distanceToTargetOrientation - 360;
                    }

                    if ((distanceToTarget <= navErrorTolerance && Math.abs(distanceToTargetOrientation) <= rotationErrorTolerance)
                            || navCorrections >= 3)
                    {

                        navEngaged = false;
                        navCorrections = 0;
                    }
                }
            }
        }
        test = distanceToTargetOrientation;

        distanceToTargetPrevious = distanceToTarget;

        double[] navReturn = new double[3];
        if(navEngaged)
        {
            navReturn[0] = 1;
        }
        else
        {
            navReturn[0] = -1;
        }
        navReturn[1] = distanceToTarget;
        navReturn[2] = distanceToTargetOrientation;
        return navReturn;
    }

    public void reset()
    {
        odometry.resetPosAndIMU();
        teleOpIMUOffset = 0;
    }

    public void recalibrate()
    {
        odometry.recalibrateIMU();
    }

    public double getOrientationRaw()
    {
        return (odometry.getHeading() * (180/Math.PI)) * -1;
    }

    public double getOrientationCurrent()
    {
        return (getOrientationRaw() + 360000) % 360;
    }

    public void setDirection(GoBildaPinpointDriver.EncoderDirection xPod, GoBildaPinpointDriver.EncoderDirection yPod)
    {
        odometry.setEncoderDirections(xPod, yPod);
    }

    public double getRad()
    {
        return odometry.getHeading();
    }

    public void update()
    {
        odometry.update();
    }

    public double getX()
    {
        return odometry.getPosX();
    }

    public double getY()
    {
        return odometry.getPosY();
    }

    public double getVelX()
    {
        return odometry.getVelX();
    }

    public double getVelY()
    {
        return odometry.getVelY();
    }

    public double getTargetPos()
    {
        return currentTarget;
    }
}
