package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

public class Robot
{
    GoBildaPinpointDriver odometry;
    ElapsedTime timer = new ElapsedTime();
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorSlideLeft, motorSlideRight;
    DcMotorEx motorClawArm;
    Servo servoClawGrabber, servoClawExtend;

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

    public void setSpeed(double speed)
    {
        globalSpeed = speed;
    }


    public void forward(double distance) // distance must be positive
    {
        //odometry.resetPosAndIMU();

        currentTarget = getY() + distance;
        double slowPosition = 400;
        double slowSpeed = globalSpeed;
        double stoppedY = 0.0;
        while(getY() < currentTarget) // Y is positive while going forwards
        {
            if(currentTarget < slowPosition)
            {
                slowSpeed = (currentTarget/slowPosition) * globalSpeed;
            }
            motorFrontLeft.setPower(-slowSpeed);
            motorFrontRight.setPower(-slowSpeed);
            motorBackLeft.setPower(-slowSpeed);
            motorBackRight.setPower(-slowSpeed);
            stoppedY = currentTarget;
            update();
        }
        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
        YStoppedPosition = stoppedY;
    }

    public void backward(double distance) // distance must be positive
    {
        //odometry.resetPosAndIMU();


        currentTarget = getY() - distance;
        while(getY() > distance)//Y is negative when going backward
        {

            motorFrontLeft.setPower(globalSpeed);
            motorFrontRight.setPower(globalSpeed);
            motorBackLeft.setPower(globalSpeed);
            motorBackRight.setPower(globalSpeed);
            update();
        }
        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
    }

    public void left(double distance) //distance must be positive
    {
        //odometry.resetPosAndIMU();
        //distance = distance * -1;

        currentTarget = getX() - distance;

        while(getX() > distance) // X is negative when going left
        {
            motorFrontLeft.setPower(globalSpeed);// strafe left
            motorFrontRight.setPower(-globalSpeed);
            motorBackLeft.setPower(-globalSpeed);
            motorBackRight.setPower(globalSpeed);


            update();
        }
        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
    }

    public void right(double distance) // distance must be positive
    {
        //odometry.resetPosAndIMU();
        //distance = distance * -1;

        currentTarget = getX() + distance;

        while(getX() < distance)// X is positive when going right
        {
            motorFrontLeft.setPower(-globalSpeed);//strafe right
            motorFrontRight.setPower(globalSpeed);
            motorBackLeft.setPower(globalSpeed);
            motorBackRight.setPower(-globalSpeed);

            update();
        }
        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
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

            double rotatePowerMin = 0.10; // set the minimum rotation speed
            if (voltageStart > 13)
            {
                rotatePowerMin = 0.1;
            }
            else if (voltageStart >= 12.8)
            {
                rotatePowerMin = 0.1;
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
                sleep(200);
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

                    motorFrontLeft.setPower(0.0);
                    motorFrontRight.setPower(0.0);
                    motorBackLeft.setPower(0.0);
                    motorBackRight.setPower(0.0);
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
                    motorSlideLeft.setPower(0.0 + ((Math.abs(slideTarget - slideCurrent) / 100) * 0.2));
                    motorSlideRight.setPower(0.0 + ((Math.abs(slideTarget - slideCurrent) / 100) * 0.2));
                    //test = -1;
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

    public double armToPosition(double armTarget, boolean noSlow)
    {
        double armCurrent = motorClawArm.getCurrentPosition();
        double errorTolerance = 30;
        double armPower = 1.0;
        int distanceToSlow = 200;

        if (armTarget > -10000)
        {
            if (Math.abs(armTarget - armCurrent) <= errorTolerance)
            {
                motorClawArm.setPower(0.0);
                sleep(100);
                armCurrent = motorClawArm.getCurrentPosition();
                if (Math.abs(armTarget - armCurrent) <= errorTolerance)
                {
                    armTarget = -10000;
                }
            }
            else if (Math.abs(armTarget - armCurrent) <= distanceToSlow)
            {
                armPower = 0.4;
                if(noSlow == false)
                {
                    if (armTarget < armCurrent) //move down
                    {
                        motorClawArm.setPower(-0.1 - ((Math.abs(armTarget - armCurrent) / distanceToSlow) * armPower));
                        //motorClawArm.setPower(-0.2 - ((Math.abs(armTarget - armCurrent) / 100) * 0.8));
                        //test = -1;
                    }
                    else
                    {
                        motorClawArm.setPower(0.1 + ((Math.abs(armTarget - armCurrent) / distanceToSlow) * armPower));
                        //motorClawArm.setPower(0.2 + ((Math.abs(armTarget - armCurrent) / 100) * 0.8));
                        //test = 1;
                    }
                }
                else
                {
                    armPower = 1.0;
                    motorClawArm.setPower(armPower);
                }
            }
            else
            {
                if (armTarget > armCurrent)
                {
                    motorClawArm.setPower(armPower);
                }
                else
                {
                    motorClawArm.setPower(armPower * (-1.0));
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

    public double[] navToPosition(double positionXTarget, double positionYTarget, double positionOrientationTarget, boolean positionPrecise) //X is forward - Y is strafe (left is positive)
    {
        double distanceToTarget = 0.0;

        if (positionXTarget > -10000) // -10000 means no target
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

            double powerX = 0.0; // -1.0 - +1.0
            double powerY = 0.0; // -1.0 - +1.0
            double powerRotateMax = 0.7;
            double powerRotate = 0.0; // -1.0 - +1.0
            double denominator = 1;

            double positionXCurrent = getX();
            double positionYCurrent = getY();
            double positionOrientationCurrent = getOrientationCurrent();
            boolean waypointPast = false;

            double distanceToTargetOrientation = positionOrientationTarget - positionOrientationCurrent;
            // distanceToTarget gets converted to values between +180 to -180
            if (distanceToTargetOrientation < -180)
            {
                distanceToTargetOrientation = distanceToTargetOrientation + 360;
            }
            else if (distanceToTargetOrientation > 180)
            {
                distanceToTargetOrientation = distanceToTargetOrientation - 360;
            }

            if(Math.abs(distanceToTargetOrientation) > 1)
            {
                powerRotate = distanceToTargetOrientation / 5.0;
                if (powerRotate > 1.0)
                {
                    powerRotate = 1.0;
                }
                // turned off rotation correction
                powerRotate = 0.0;
            }

            double distanceToSlow = 200;
            double errorTolerance = 20;

            double motorPower = powerMax;

            double disanceToTargetX = positionXTarget - positionXCurrent;
            double disanceToTargetY = positionYTarget - positionYCurrent;

            if (disanceToTargetX >= disanceToTargetY)
            {
                powerX = (disanceToTargetX / Math.abs(disanceToTargetX)) * 1.2;
                powerY = disanceToTargetY / Math.abs(disanceToTargetX);
            }
            else
            {
                powerX = (disanceToTargetX / Math.abs(disanceToTargetY)) * 1.2;
                powerY = disanceToTargetY / Math.abs(disanceToTargetY);
            }

            distanceToTarget = Math.sqrt((Math.pow((positionXTarget - positionXCurrent), 2)) + (Math.pow((positionYTarget - positionYCurrent), 2)));
            test = distanceToTargetPrevious;
            if(distanceToTarget > distanceToTargetPrevious)
            {
                waypointPast = true;
                //test = 99999;
            }
            distanceTraveledFromPrevious = Math.abs(distanceToTarget - distanceToTargetPrevious);
            distanceToTargetPrevious = distanceToTarget;

            if (distanceToTarget < distanceToSlow)
            {
                powerMax = 0.4;
                double powerUsed = powerMin;
                // slow down

                if (Math.abs(positionXTarget - positionXCurrent) > Math.abs(positionYTarget - positionYCurrent))
                {
                    powerUsed = powerMin + .05;
                }
                if (positionPrecise)
                {
                    if (distanceToTarget < 100)
                    {
                        motorPower = powerUsed;
                    }
                    else
                    {
                        motorPower = ((powerMax - powerUsed) * (distanceToTarget / distanceToSlow)) + powerUsed;
                    }
                }
            }

            if (distanceToTarget > errorTolerance)
            {
                powerY = powerY * (-1.0);

                if (positionPrecise == false && (distanceToTarget < 100 || waypointPast == true))
                {
                    positionXTarget = -10000.0;
                    positionYTarget = -10000.0;
                    distanceToTargetPrevious = 1000;
                    waypointPast = false;
                }
                else
                {
                    if (distanceTraveledFromPrevious > 15 && distanceToTarget < distanceToSlow && positionPrecise == true)
                    {
                        motorPower = -0.1;
                        if (Math.abs(distanceToTarget) < errorTolerance + 10)
                        {
                            navCorrections++;
                        }
                        else
                        {
                            navCorrections = 0;
                        }
                    }
                    denominator = Math.max((Math.abs(powerX) + Math.abs(powerY) + Math.abs(powerRotate * powerRotateMax)), 1.0);
                    motorFrontLeft.setPower(((powerY - powerX - (powerRotate * powerRotateMax)) * motorPower) / denominator);
                    motorFrontRight.setPower(((powerY + powerX + (powerRotate * powerRotateMax)) * motorPower) / denominator);
                    motorBackLeft.setPower(((powerY + powerX - (powerRotate * powerRotateMax)) * motorPower) / denominator);
                    motorBackRight.setPower(((powerY - powerX + (powerRotate * powerRotateMax)) * motorPower) / denominator);
                }
            }
            else
            {
                sleep(200);
                odometry.update();
                positionXCurrent = getX();
                positionYCurrent = getY();
                distanceToTarget = Math.sqrt((Math.pow((positionXTarget - positionXCurrent), 2)) + (Math.pow((positionYTarget - positionYCurrent), 2)));
                if (distanceToTarget < errorTolerance || navCorrections >= 2)
                {

                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);

                    positionXTarget = -10000.0;
                    positionYTarget = -10000.0;
                }
            }
        }

        double[] distanceReturn = new double[2];
        distanceReturn[0] = distanceToTarget;
        distanceReturn[1] = positionXTarget;
        return distanceReturn;
    }

    public void armToGamePosition(String position)
    {
        if(position.equalsIgnoreCase("zero"))
        {

        }
        else if(position.equalsIgnoreCase("bar"))
        {

        }
        else if(position.equalsIgnoreCase("bucket"))
        {

        }
    }

    public void reset()
    {
        odometry.resetPosAndIMU();
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
