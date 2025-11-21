package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class Navigation
{
    GoBildaPinpointDriver odometry;
    ElapsedTime timer = new ElapsedTime();
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    IMU orientationIMU;

    public double teleOpIMUOrientation;
    public double teleOpIMUOffset;

    public double voltageStart = 0.0;
    public int rotateCorrections = 0;
    public int navCorrections = 0;

    double globalSpeed = 0.5;

    double currentTarget = 0;

    public double test = 0.0;


    public double navAcceleration = 0;

    public double distanceToTargetPrevious = 10000;
    public double distanceTraveledFromPrevious = 0.0;
    public Navigation(GoBildaPinpointDriver pod)
    {
        odometry = pod;
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setYawScalar(-1.0);
    }

    public Navigation()
    {

    }

    public void sleep(int milliseconds)
    {
        timer.reset();
        while(timer.milliseconds() < milliseconds)
        {
            //do nothing
        }
    }

    public void setMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br)
    {
        motorFrontLeft = fl;
        motorFrontRight = fr;
        motorBackLeft = bl;
        motorBackRight = br;

//        motorFrontLeft.setDirection(motorFrontLeft.getDirection().inverted());
//        motorFrontRight.setDirection(motorFrontRight.getDirection().inverted());
//        motorBackLeft.setDirection(motorBackLeft.getDirection().inverted());
//        motorBackRight.setDirection(motorBackRight.getDirection().inverted());
    }
    public void setSpeed(double speed)
    {
        globalSpeed = speed;
    }

    public double[] navToPosition1(double navXTarget, double navYTarget, double navOrientationTarget,
                                  boolean navOnlyRotate, boolean navPrecise, boolean navEngaged )
    {
        motorFrontLeft.setPower(0.2);
        motorFrontRight.setPower(0.2);
        motorBackLeft.setPower(0.2);
        motorBackRight.setPower(0.2);


        double[] navReturn = new double[3];
        navReturn[0] = 1;
        navReturn[1] = 0;
        navReturn[2] = 0;
        return navReturn;
    }
    public double[] navToPosition(double navXTarget, double navYTarget, double navOrientationTarget, boolean navOnlyRotate,
                                  boolean navPrecise, double navSpeedOverride, boolean navEngaged)
    {
        odometry.update();

        double distanceToTarget = 0.0;
        double distanceToTargetOrientation = 0.0;
        double currentOrientation = getOrientationCurrent();
        double currentOrientationRad = getOrientionRad();
        double currentX = getX();
        double currentY = getY();
        double rotationErrorTolerance = 2.0;
        double distanceToRotationSlow = 40; // slow rotational speed on linear function if below this value
        double distanceToNavSlow = 200;
        double navErrorTolerance = 20;

        double powerMaxRotate = 0.75;
        double powerMinRotate = 0.2;
        double powerMaxNav = 0.8;
        if(navSpeedOverride > 0)
        {
            powerMaxNav = navSpeedOverride;
        }
        double powerMinNav = 0.2;

//        double powerMaxRotate = 0.0;
//        double powerMinRotate = 0.0;
//        double powerMaxNav = 0.0;
//        double powerMinNav = 0.0;

        double rotationX = 0;
        double rotationY = 0;

        double powerMotorNav = powerMaxNav;
//        if (voltageStart > 13)
//        {
//            powerMinRotate = 0.2;
//        }
//        else if (voltageStart >= 12.8)
//        {
//            powerMinRotate = 0.22;
//        }

        double bearingTarget = 0.0;
        double valueX = 0.0;
        double valueY = 0.0;
        double valueRotation = 0.0;
        double rotationDirection = 1.0;  // 1.0 is CW -1.0 is CCW

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
                if ((Math.abs(odometry.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS) * (180 / Math.PI)) < 5) && Math.abs(distanceToTargetOrientation) > 10)
                {
                    powerMaxRotate = powerMaxRotate * 0.8;
                }
                else
                {
                    if(powerMaxRotate > 0.5)
                    {
                        powerMaxRotate = 0.5;
                    }
                }

                // slow rotation using linear function based on distanceToTarget
                if (Math.abs(odometry.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS) * (180 / Math.PI)) > 120)
                {
                    valueRotation = 0;
                }
                else
                {
                    if (Math.abs(odometry.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS) * (180 / Math.PI)) < 5)
                    {
                        if (powerMinRotate > 0.2)
                        {
                            powerMinRotate = 0.2;
                        }
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
                motorFrontLeft.setPower((valueRotation));
                motorFrontRight.setPower((valueRotation) * -1);
                motorBackLeft.setPower((valueRotation));
                motorBackRight.setPower((valueRotation) * -1);
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
                        if(powerMaxNav > 0.5)
                        {
                            powerMaxNav = 0.5;
                        }

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
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    navEngaged = false;
                }
                else if (distanceToTarget > navErrorTolerance || Math.abs(distanceToTargetOrientation) > rotationErrorTolerance)
                {
                    if(navPrecise)
                    {
                        if (distanceTraveledFromPrevious > 15 && distanceToTarget < distanceToNavSlow)
                        {
                            // tap the brakes
                            powerMotorNav = -0.20;
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
                    rotationX = (valueX * Math.cos(currentOrientationRad) - valueY * Math.sin(currentOrientationRad));
                    rotationY = (valueX * Math.sin(currentOrientationRad) + valueY * Math.cos(currentOrientationRad));

                    if(distanceToTarget <= navErrorTolerance && Math.abs(distanceToTargetOrientation) > rotationErrorTolerance)
                    {
                        motorFrontLeft.setPower((valueRotation));
                        motorFrontRight.setPower((valueRotation) * -1);
                        motorBackLeft.setPower((valueRotation));
                        motorBackRight.setPower((valueRotation) * -1);
                    }
                    else
                    {
                        double denominator = Math.max(Math.abs(rotationY) + Math.abs(rotationX) + Math.abs(valueRotation), 1);
                        motorFrontLeft.setPower(((rotationY + rotationX + valueRotation) / denominator) * powerMotorNav);
                        motorFrontRight.setPower(((rotationY - rotationX - valueRotation) / denominator) * powerMotorNav);
                        motorBackLeft.setPower(((rotationY - rotationX + valueRotation) / denominator) * powerMotorNav);
                        motorBackRight.setPower(((rotationY + rotationX - valueRotation) / denominator) * powerMotorNav);
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
        test = Math.cos(currentOrientation);

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

    //misc PinPoint functions

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
        return (odometry.getHeading(UnnormalizedAngleUnit.RADIANS) * (180/Math.PI));
    }

    public double getOrientationCurrent()
    {
        return (getOrientationRaw() + 360000) % 360;
    }

    public void setDirection(GoBildaPinpointDriver.EncoderDirection xPod, GoBildaPinpointDriver.EncoderDirection yPod)
    {
        odometry.setEncoderDirections(xPod, yPod);
    }

    public double getOrientionRad()
    {
        return odometry.getHeading(UnnormalizedAngleUnit.RADIANS);
    }

    public void update()
    {
        odometry.update();
    }

    public double getX()
    {
        // if pod is reversed multiply by -1
        double reverseDirection = 1;
        return odometry.getPosX(DistanceUnit.MM) * reverseDirection;
    }

    public double getY()
    {
        // if pod is reversed multiply by -1
        double reverseDirection = -1;
        return odometry.getPosY(DistanceUnit.MM) * reverseDirection;
    }

    public double getVelX()
    {
        // if pod is reversed multiply by -1
        double reverseDirection = 1;
        return odometry.getVelX(DistanceUnit.MM) * reverseDirection;
    }

    public double getVelY()
    {
        // if pod is reversed multiply by -1
        double reverseDirection = -1;
        return odometry.getVelY(DistanceUnit.MM) * reverseDirection;
    }

    public double getTargetPos()
    {
        return currentTarget;
    }

}
