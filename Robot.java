package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

public class Robot
{
    GoBildaPinpointDriver odometry;
    ElapsedTime timer = new ElapsedTime();
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public String spinDirection = "Left";

    double globalSpeed = 0.5;

    double currentTarget = 0;

    public double targetDistance = 0.0;

    public double currentDeg = 0.0;

    public Robot(GoBildaPinpointDriver pod)
    {
        odometry = pod;
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
    }


    public void setOffsets(double x, double y)
    {
        odometry.setOffsets(x, y);
    }

    public void sleep(int milliseconds)
    {
        timer.reset();
        while(timer.milliseconds() < milliseconds)
        {
            //ljasfghuiasfdhuifhuisfhuid
        }
    }

    public void setMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br)
    {
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
    }

    public void setSpeed(double speed)
    {
        globalSpeed = speed;
    }


    public void forward(double distance) // distance must be positive
    {
        //odometry.resetPosAndIMU();

        currentTarget = getY() - distance;
        while(getY() > currentTarget) // Y is negative while going forwards
        {
            frontLeft.setPower(-globalSpeed);
            frontRight.setPower(-globalSpeed);
            backLeft.setPower(-globalSpeed);
            backRight.setPower(-globalSpeed);
        }
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    public void backward(double distance) // distance must be positive
    {
        //odometry.resetPosAndIMU();


        currentTarget = getY() + distance;
        while(getY() > distance)//Y is positive when going backward
        {
            frontLeft.setPower(globalSpeed);
            frontRight.setPower(globalSpeed);
            backLeft.setPower(globalSpeed);
            backRight.setPower(globalSpeed);
        }
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    public void left(double distance) //distance must be positive
    {
        //odometry.resetPosAndIMU();
        //distance = distance * -1;

        currentTarget = getX() - distance;

        while(getX() > distance) // X is negative when going left
        {
            frontLeft.setPower(globalSpeed);// strafe left
            frontRight.setPower(-globalSpeed);
            backLeft.setPower(-globalSpeed);
            backRight.setPower(globalSpeed);


            odometry.update();
        }
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    public void right(double distance) // distance must be positive
    {
        //odometry.resetPosAndIMU();
        //distance = distance * -1;

        currentTarget = getX() + distance;

        while(getX() < distance)// X is positive when going right
        {
            frontLeft.setPower(-globalSpeed);//strafe right
            frontRight.setPower(globalSpeed);
            backLeft.setPower(globalSpeed);
            backRight.setPower(-globalSpeed);

            odometry.update();
        }
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }




    public void rotate(double target)
    {
        //odometry.resetPosAndIMU();
        odometry.update();
        currentTarget = target;

        double distanceToTarget = 0.0;

        currentDeg = getDegrees();
        spinDirection = "Left";

        if(((target - currentDeg) % 360) - 180 <= 0)
        {
            spinDirection = "Right";
        }

        if(spinDirection.equalsIgnoreCase("right"))
        {
            if(target > currentDeg)
            {
                distanceToTarget = target - currentDeg;
            }
            else
            {
                distanceToTarget = target - currentDeg + 360;
            }
            //distanceToTarget = (currentDeg - (target + 360)) % 360;
            targetDistance = distanceToTarget;
            while (distanceToTarget > 2)//right
            {
                frontLeft.setPower(-globalSpeed);
                frontRight.setPower(globalSpeed);
                backLeft.setPower(-globalSpeed);
                backRight.setPower(globalSpeed);

                odometry.update();
                currentDeg = (getDeg() + 360000) % 360;
                distanceToTarget = (currentDeg - (target + 360)) % 360;
                targetDistance = distanceToTarget;
            }
        }

        else
        {
            if(target > currentDeg)
            {
                distanceToTarget = Math.abs(currentDeg - target - 360);
            }
            else
            {
                distanceToTarget = currentTarget - target;
            }
            //distanceToTarget = (currentDeg - (target + 360)) % 360;
            targetDistance = distanceToTarget;
            while (distanceToTarget > 2)//left
            {
                frontLeft.setPower(globalSpeed);
                frontRight.setPower(-globalSpeed);
                backLeft.setPower(globalSpeed);
                backRight.setPower(-globalSpeed);

                odometry.update();
                currentDeg = (getDeg() + 360000) % 360;
                distanceToTarget = (currentDeg - (target + 360)) % 360;
                targetDistance = distanceToTarget;
            }
        }


        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    public void reset()
    {
        odometry.resetPosAndIMU();
    }

    public void recalibrate()
    {
        odometry.recalibrateIMU();
    }

    public double getDegrees()
    {
        currentDeg = (getDeg() + 360000) % 360;
        return currentDeg;
    }

    public void setDirection(GoBildaPinpointDriver.EncoderDirection xPod, GoBildaPinpointDriver.EncoderDirection yPod)
    {
        odometry.setEncoderDirections(xPod, yPod);
    }

    public double getDeg()
    {
        return (odometry.getHeading() * (180/Math.PI)) * -1;
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
