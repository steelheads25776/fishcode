package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.hardware.EasyIMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.opencv.core.Mat;

public class Robot
{
    GoBildaPinpointDriver odometry;
    ElapsedTime timer = new ElapsedTime();
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    TouchSensor buttonArmStop;
    //IMU orientationIMU;

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
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odometry.setYawScalar(-1.0);
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

//    public void setOffsetIMU(IMU imu)
//    {
//        orientationIMU = imu;
//        //teleOpIMUOffset = orientationIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//    }
//
//    public void getIMUOffset()
//    {
//        teleOpIMUOffset = orientationIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//    }
//
//    public double getTeleOpIMURotation()
//    {
//        teleOpIMUOrientation = getRad();
//        return teleOpIMUOrientation + teleOpIMUOffset;
//    }


    public void setMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br)
    {
        motorFrontLeft = fl;
        motorFrontRight = fr;
        motorBackLeft = bl;
        motorBackRight = br;
    }

    public void setSpeed(double speed)
    {
        globalSpeed = speed;
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

    public double getRad()
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
