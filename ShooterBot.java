package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

public class ShooterBot
{
    DcMotorEx motorShooterRight, motorShooterLeft;
    CRServo servoMagazine;
    CRServo servoLoaderStartLeft, servoLoaderStartRight, servoLoaderAssist;
    Servo servoLifter;
    AnalogInput magazineEncoder;
    DcMotor motorIntake;
    double speedIntakeNormal;
    double speedIntakeReverse;
    boolean isMagEngaged;

    public boolean magazineEngaged = false;
    double axonPreviousPosition = -1000;

    double powerRotateCWMax = -0.11;
    double powerRotateCWSlow = -0.05;
    double powerRotateCCWMax = -0.14;
    double powerRotateCCWSlow = -0.07;

    int axonFrozen = 0;

    private ElapsedTime timer;

    PredominantColorProcessor chamberLeftColor, chamberMiddleColor, chamberRightColor;
    PredominantColorProcessor.Result resultLeft, resultMiddle, resultRight;

    public ShooterBot()
    {
        this.speedIntakeNormal = 1.0;
        this.speedIntakeReverse = 0.5;
        timer = new ElapsedTime();
        isMagEngaged = false;
    }
    public ShooterBot(double intSpeed, double intReverseSpeed)
    {
        this.speedIntakeNormal = intSpeed;
        this.speedIntakeReverse = intReverseSpeed;
        timer = new ElapsedTime();
    }

    public void startCameraSensors()
    {
        chamberLeftColor = new PredominantColorProcessor.Builder()
                //.setRoi(ImageRegion.asImageCoordinates(450, 1, 750,301))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();
        chamberMiddleColor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(450, 1, 750,301))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();
        chamberRightColor = new PredominantColorProcessor.Builder()
                //.setRoi(ImageRegion.asImageCoordinates(450, 1, 750,301))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();


        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors(chamberLeftColor, chamberMiddleColor, chamberRightColor)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(hardwareMap.get(WebcamName.class, "Internal Cam"))
                .build();
    }

    public String[] getChambers()
    {
        String[] chambers = new String[3];          //0 = left, 1 = middle, 2 = right

        resultLeft = chamberLeftColor.getAnalysis();
        resultMiddle = chamberMiddleColor.getAnalysis();
        resultRight = chamberRightColor.getAnalysis();

        chambers[0]  = resultLeft.toString();
        chambers[1]  = resultMiddle.toString();
        chambers[2]  = resultRight.toString();

        return chambers;
    }

    public void setLaunchMotors(DcMotorEx shootLeft, DcMotorEx shootRight)
    {
        this.motorShooterLeft = shootLeft;
        this.motorShooterRight = shootRight;
    }

    public void setLoadServos(Servo lift, CRServo loadLeft, CRServo loadRight, CRServo assistServo)
    {
        this.servoLifter = lift;
        this.servoLoaderStartLeft = loadLeft;
        this.servoLoaderStartRight = loadRight;
        this.servoLoaderAssist = assistServo;
    }

    public void setMagazine(CRServo axonServo, AnalogInput encoder)
    {
        this.servoMagazine = axonServo;
        this.magazineEncoder = encoder;

    }

    public void setIntakeSpeed(double normal, double reverse)
    {
        this.speedIntakeNormal = normal;
        this.speedIntakeReverse = reverse;
    }

    private void sleep(int time)
    {
        timer.reset();
        if(timer.milliseconds() > time)
        {

        }
    }
    public void axonToPosition(double target, String direction)
    {
        powerRotateCWMax = -0.13;
        powerRotateCWSlow = -0.05;
        powerRotateCCWMax = -0.16;
        powerRotateCCWSlow = -0.07;

        double currentPos = (magazineEncoder.getVoltage() / 3.3) * 360;
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
                servoMagazine.setPower(powerRotateCWMax);
                servoMagazine.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            else
            {
                servoMagazine.setPower(powerRotateCCWMax);
                servoMagazine.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }
        else if(distanceToTarget < -8)
        {
            servoMagazine.setPower(powerRotateCWSlow);
            servoMagazine.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (distanceToTarget > 8)
        {
            servoMagazine.setPower(powerRotateCCWSlow);
            servoMagazine.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if(distanceToTarget < -3)
        {
            servoMagazine.setPower(powerRotateCWSlow);
            servoMagazine.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (distanceToTarget > 3)
        {
            servoMagazine.setPower(powerRotateCCWSlow);
            servoMagazine.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            servoMagazine.setPower(0.0);
            sleep(100);
            currentPos = (magazineEncoder.getVoltage() / 3.3) * 360;
            if(Math.abs(currentPos - target) < 3)
            {
                magazineEngaged = false;
                axonPreviousPosition = -1000;
            }
        }
    }

    public void activateAssistServos()
    {
        servoLoaderAssist.setPower(-1.0);
        servoLoaderStartRight.setPower(-1.0);
        servoLoaderStartLeft.setPower(1.0);
    }
    public void disableAssistServos()
    {
        servoLoaderAssist.setPower(0.0);
        servoLoaderStartRight.setPower(0.0);
        servoLoaderStartLeft.setPower(0.0);
    }
    public void lifterToPos(double position)
    {
        servoLifter.setPosition(position);
    }
    public void launchAtPower(double power)
    {
        motorShooterRight.setPower(power);
        motorShooterLeft.setPower(power);
    }
    public void intake()
    {
        motorIntake.setPower(speedIntakeNormal);
    }

    public void intakeReverse()
    {
        motorIntake.setPower(speedIntakeReverse);
    }
    public void intakeDisable()
    {
        motorIntake.setPower(0.0);
    }

    public double getMagAngle()
    {
        return (magazineEncoder.getVoltage() / 3.3) * 360;
    }
}
