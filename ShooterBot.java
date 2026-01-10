package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.qualcomm.ftccommon.FtcWifiDirectChannelSelectorActivity;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    // boolean isMagEngaged;

    public boolean magazineEngaged = false;
    double axonPreviousPosition = -1000;

    double powerRotateCWMax = -0.11;//-0.11//-0.15 -0.17, 0.20
    double powerRotateCWSlow = -0.05;
    double powerRotateCCWMax = -0.14;//-0.14, -0.18, -0.20, 0.23
    double powerRotateCCWSlow = -0.07;

    int axonFrozen = 0;

    private ElapsedTime timer;

    public PredominantColorProcessor chamberControlColor, chamberBarrelColor, chamberExpansionColor;
    public PredominantColorProcessor.Result resultControl, resultBarrel, resultExpansion;

    public Limelight3A limelight;

    String launchStep = "None";
    public boolean launchEngaged = false;
    int rotateIncrement = 0;
    int brakeEngageCount = 0;
    public String[] chambers = new String[3];
    public String motifOrder;
    //public double axonTarget;

    public ShooterBot()
    {
        this.speedIntakeNormal = 1.0;
        this.speedIntakeReverse = 0.5;
        timer = new ElapsedTime();
        //isMagEngaged = false;
    }
    public ShooterBot(double intSpeed, double intReverseSpeed)
    {
        this.speedIntakeNormal = intSpeed;
        this.speedIntakeReverse = intReverseSpeed;
        timer = new ElapsedTime();
    }
    public void setLimelight(Limelight3A ll)
    {
        limelight = ll;
    }
    public void changePipeline(int pl)
    {
        limelight.pipelineSwitch(pl);
    }
    public void startLimelight()
    {
        limelight.start();
    }

    public void startCameraSensors()
    {
        chamberControlColor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(1, 420, 351, 720)) //100, 420, 350 770
                .setSwatches(
//                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
//                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.ORANGE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.CYAN,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.PURPLE,
                        PredominantColorProcessor.Swatch.MAGENTA,
                        PredominantColorProcessor.Swatch.BLACK)
                .build();
        chamberBarrelColor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(450, 1, 800,301))//450, 1, 750, 301
                .setSwatches(
//                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
//                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.ORANGE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.CYAN,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.PURPLE,
                        PredominantColorProcessor.Swatch.MAGENTA,
                        PredominantColorProcessor.Swatch.BLACK)
                .build();
        chamberExpansionColor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(930, 420, 1280, 720))//930, 420, 1180, 770
                .setSwatches(
//                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
//                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.ORANGE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.CYAN,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.PURPLE,
                        PredominantColorProcessor.Swatch.MAGENTA,
                        PredominantColorProcessor.Swatch.BLACK)
                .build();
    }

    public String[] getChambers()
    {
        //String[] chambers = new String[3];          //0 = Barrel, 1 = Control, 2 = Expansion

        String resultBarrel = chamberBarrelColor.getAnalysis().closestSwatch.toString();
        String resultControl = chamberControlColor.getAnalysis().closestSwatch.toString();
        String resultExpansion = chamberExpansionColor.getAnalysis().closestSwatch.toString();

        String returnBarrel = "?";
        String returnControl = "?";
        String returnExpansion = "?";

        boolean testingSwatchOn = true; // turns on swatch value to return values

        if(resultBarrel.equalsIgnoreCase("GREEN") || resultBarrel.equalsIgnoreCase("ARTIFACT_GREEN"))
        {
            returnBarrel = "GREEN";
        }
        else if(resultBarrel.equalsIgnoreCase("PURPLE") || resultBarrel.equalsIgnoreCase("ARTIFACT_PURPLE")
                || resultBarrel.equalsIgnoreCase("MAGENTA") || resultBarrel.equalsIgnoreCase("BLUE"))
        {
            returnBarrel = "PURPLE";
        }
        else
        {
            returnBarrel = "EMPTY";
        }

        if(resultControl.equalsIgnoreCase("GREEN") || resultControl.equalsIgnoreCase("ARTIFACT_GREEN"))
        {
            returnControl = "GREEN";
        }
        else if(resultControl.equalsIgnoreCase("PURPLE") || resultControl.equalsIgnoreCase("ARTIFACT_PURPLE")
            || resultControl.equalsIgnoreCase("MAGENTA") || resultControl.equalsIgnoreCase("BLUE"))
        {
            returnControl = "PURPLE";
        }
        else
        {
            returnControl = "EMPTY";
        }

        if(resultExpansion.equalsIgnoreCase("GREEN") || resultExpansion.equalsIgnoreCase("ARTIFACT_GREEN"))
        {
            returnExpansion = "GREEN";
        }
        else if(resultExpansion.equalsIgnoreCase("PURPLE") || resultExpansion.equalsIgnoreCase("ARTIFACT_PURPLE")
            || resultExpansion.equalsIgnoreCase("MAGENTA") || resultExpansion.equalsIgnoreCase("BLUE"))
        {
            returnExpansion = "PURPLE";
        }
        else
        {
            returnExpansion = "EMPTY";
        }

        if(testingSwatchOn)
        {
            returnBarrel += " - " + resultBarrel;
            returnControl += " - " + resultControl;
            returnExpansion += " - " + resultExpansion;
        }
        chambers[0]  = returnBarrel;
        chambers[1]  = returnControl;
        chambers[2]  = returnExpansion;

        return chambers;
    }

    public double[] axonToOrder(double axonTargetCurrent)
    {
        double[] axonReturn = new double[2];
        double axonTarget = axonTargetCurrent;
        double axonDirection = 0;//1.0 = cw, -1.0 = ccw
        chambers = getChambers();
        String chamberOrder = chambers[0].substring(0, 1) + chambers[1].substring(0, 1) + chambers[2].substring(0, 1);

        if(motifOrder.equalsIgnoreCase("gpp"))
        {
            if(chamberOrder.equalsIgnoreCase("gpp"))
            {
                //nothing
            }
            else if(chamberOrder.equalsIgnoreCase("pgp"))
            {
                axonTarget = ((axonTarget + 360) + 120) % 360;
                axonDirection = 1.0;
            }
            else if(chamberOrder.equalsIgnoreCase("ppg"))
            {
                axonTarget = ((axonTarget + 360) - 120) % 360;
                axonDirection = -1.0;
            }
        }
        else if(motifOrder.equalsIgnoreCase("pgp"))
        {
            if(chamberOrder.equalsIgnoreCase("gpp"))
            {
                axonTarget = ((axonTarget + 360) - 120) % 360;
                axonDirection = -1.0;
            }
            else if(chamberOrder.equalsIgnoreCase("pgp"))
            {
                //nothing
            }
            else if(chamberOrder.equalsIgnoreCase("ppg"))
            {
                axonTarget = ((axonTarget + 360) + 120) % 360;
                axonDirection = 1.0;
            }
        }
        else if(motifOrder.equalsIgnoreCase("ppg"))
        {
            if(chamberOrder.equalsIgnoreCase("gpp"))
            {
                axonTarget = ((axonTarget + 360) + 120) % 360;
                axonDirection = 1.0;
            }
            else if(chamberOrder.equalsIgnoreCase("pgp"))
            {
                axonTarget = ((axonTarget + 360) - 120) % 360;
                axonDirection = -1.0;
            }
            else if(chamberOrder.equalsIgnoreCase("ppg"))
            {
                //nothing
            }
        }

        axonReturn[0] = axonTarget;
        axonReturn[1] = axonDirection;

        return axonReturn;
    }
    /*
    public void rotateMagazine(String direction)
    {

        if(direction.equalsIgnoreCase("left") || direction.equalsIgnoreCase("l") || direction.equalsIgnoreCase("ccw"))
        {
            targetPos -= 120;
            if(targetPos < 0)
            {
                axonTargetPosition += 360;
            }
            else if(axonTargetPosition > 360)
            {
                axonTargetPosition -= 360;
            }
            axonDirection = "ccw";
            robot.magazineEngaged = true;
        }
        else if(direction.equalsIgnoreCase("right") || direction.equalsIgnoreCase("r") || direction.equalsIgnoreCase("cw"))
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
        }
    }
    */

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
        while(timer.milliseconds() < time)
        {

        }
    }
    public void axonToPosition(double target, String direction)
    {
        powerRotateCWMax = -0.14;
        powerRotateCWSlow = -0.05;
        powerRotateCCWMax = -0.17;
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

        if(rotateIncrement < 5)
        {
            powerRotateCWMax = -0.05  - (rotateIncrement * .01);
            rotateIncrement++;
        }
        axonPreviousPosition = currentPos;
        double distanceToTarget = currentPos - target;

        if(Math.abs(distanceToTarget) >= 25)
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
            brakeEngageCount = 0;
        }
        else if(distanceToTarget < -4)
        {
            servoMagazine.setPower(powerRotateCWSlow);
            if(brakeEngageCount > 1)
            {
                servoMagazine.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            else
            {
                servoMagazine.setDirection(DcMotorSimple.Direction.REVERSE);
                brakeEngageCount++;
            }
        }
        else if (distanceToTarget > 4)
        {
            servoMagazine.setPower(powerRotateCCWSlow);
            if(brakeEngageCount > 1)
            {
                servoMagazine.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else
            {
                servoMagazine.setDirection(DcMotorSimple.Direction.FORWARD);
                brakeEngageCount++;
            }
        }
        else
        {
            brakeEngageCount = 100;
            servoMagazine.setPower(0.0);
            sleep(100);
            currentPos = (magazineEncoder.getVoltage() / 3.3) * 360;
            if(Math.abs(currentPos - target) < 4)
            {
                magazineEngaged = false;
                axonPreviousPosition = -1000;
                rotateIncrement = 0;
            }
        }
        //return target;
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
