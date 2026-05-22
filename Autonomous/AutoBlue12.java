package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.teamcode.Navigation;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.ShooterBot;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous
public class AutoBlue12 extends OpMode
{
    Limelight3A ll;

    boolean launchEngaged = false;
    String launchStep = "None";
    //boolean magazineEngaged = false;
    double navXTarget = 0;
    double navYTarget = 0;
    double navOrientationTarget = 0;
    boolean navOnlyRotate = false;
    boolean navPrecise = false;
    double navSpeedOverride = 0;
    boolean navEngaged = false;
    double shootingPower = 0.60;//0.75
    double powerRotateCWMax = -0.11;
    double powerRotateCWSlow = -0.05;
    double powerRotateCCWMax = -0.14;
    double powerRotateCCWSlow = -0.07;
    String axonDirection = "cw";
    double axonPreviousPosition = -1000;
    int axonFrozen = 0;
    String launchType = "";
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotorEx motorShooterRight, motorShooterLeft;
    DcMotor intakeMotor;

    //CRServo axon;
    Navigation nav;
    CRServo servoLoaderStartLeft, servoLoaderStartRight, servoLoaderAssist;
    //Servo servoLifter;
    //AnalogInput axonEncoder;
    ElapsedTime timer;
    ElapsedTime stepTimer;
    ElapsedTime magTimer;
    String step;
    double axonTargetPosition = 42;
    double lifterServoDown = 0.78;
    double lifterServoUp = 0.50;
    GoBildaPinpointDriver odometry;
    ShooterBot robot;

    double distanceToTarget = 0;
    double distanceToTargetOrientation = 0;

    double lifterDownPosition = 0.5;
    double liftControlUp = 0.72;//0.71
    double liftExpansionUp = 0.28;//0.29

    LLResult limelightResult;
    String[] chambers = new String[3];
    List<LLResultTypes.FiducialResult> fresult;
    String motifOrder = "not found";
    int tag = -1;
    int test2 = -1;
    String launchColor = "none";
    int velocityShooting = 1420;//1500
    Servo internalLight;
    boolean finished = false;
    int axonState = 0;

    int arrPos = 6;
    double[] servoPositions = new double[13];

    Servo liftControl;
    Servo liftExpansion;
    Servo turntable;
    Boolean noOrder = true;

    double[] motifOrderReturn = new double[2];
    public void sleep(int milliseconds)
    {
        timer.reset();
        while(timer.milliseconds() < milliseconds)
        {
            //do nothing
        }
    }
    /*
    public void launchArtifact()
    {
        if(launchEngaged)
        {
            if(launchStep.equalsIgnoreCase("none"))
            {
                robot.magazineEngaged = true;
                launchStep = "a1 - initial position";
            }
            else if (launchStep.equalsIgnoreCase("a1 - initial position") && !robot.magazineEngaged)
            {
                intakeMotor.setPower(1.0);

                servoLoaderAssist.setPower(-1.0);
                servoLoaderStartRight.setPower(-1.0);
                servoLoaderStartLeft.setPower(1.0);
                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "a1 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a1 - lifter engaged") && stepTimer.milliseconds() > 500)
            {
                servoLifter.setPosition(lifterServoDown);
                motorShooterLeft.setPower(shootingPower);
                motorShooterRight.setPower(shootingPower);

                stepTimer.reset();
                launchStep = "a1 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a1 - drop lifter") && stepTimer.milliseconds() > 500)
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
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);

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

                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "a2 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a2 - lifter engaged") && stepTimer.milliseconds() > 500)
            {
                servoLifter.setPosition(lifterServoDown);
                motorShooterLeft.setPower(shootingPower);
                motorShooterRight.setPower(shootingPower);

                stepTimer.reset();
                launchStep = "a2 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a2 - drop lifter") && stepTimer.milliseconds() > 500)
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

                servoLifter.setPosition(lifterServoUp);

                stepTimer.reset();
                launchStep = "a3 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a3 - lifter engaged") && stepTimer.milliseconds() > 500)
            {
                servoLifter.setPosition(lifterServoDown);
                motorShooterLeft.setPower(shootingPower);
                motorShooterRight.setPower(shootingPower);

                stepTimer.reset();
                launchStep = "a3 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a3 - drop lifter") && stepTimer.milliseconds() > 500)
            {
                stepTimer.reset();
                launchStep = "a3 - turn off launcher";
            }
            else if (launchStep.equalsIgnoreCase("a3 - turn off launcher") && stepTimer.milliseconds() > 1000)
            {
                launchEngaged = false;
                launchStep = "none";
            }
        }
    }
    */

    public void launchArtifactOrder()
    {
        if(launchEngaged)
        {
            if(launchStep.equalsIgnoreCase("none"))
            {
                motorShooterLeft.setVelocity(velocityShooting);
                motorShooterRight.setVelocity(velocityShooting);
                //robot.magazineEngaged = true;
                if(noOrder)
                {
                    timer.reset();
                    launchStep = "a1 - initial position";
                }
                else if(!noOrder)
                {
                    launchStep = "rotate to order";
                }
            }
            else if (launchStep.equalsIgnoreCase("rotate to order"))
            {
                motifOrderReturn = robot.axonToOrder(axonTargetPosition);
                //axonTargetPosition = motifOrderReturn[0];
                //axonDirection = "cw";

                if(motifOrderReturn[0] == 1)
                {
                    arrPos +=1;
                }
                if(motifOrderReturn[0] == -1)
                {
                    arrPos -=1;
                }
                if(motifOrderReturn[0] == 2)
                {
                    arrPos += 2;
                }
                /*
                else if (motifOrderReturn[1] == -2.0)
                {
                    // At least one magazine spot is empty or didn't register
                    // Don't rotate and just shoot
                    axonTargetPosition = ((axonTargetPosition + 360) - 240) % 360;  // Sets value back to starting value so doesn't rotate
                }

                 */
                //robot.magazineEngaged = true;
                stepTimer.reset();
                launchStep = "a1 - initial position";
            }
            else if (launchStep.equalsIgnoreCase("a1 - initial position") && stepTimer.milliseconds() > 300)
            {
                //intakeMotor.setPower(0.0);

                servoLoaderAssist.setPower(1.0);
                servoLoaderStartRight.setPower(1.0);
                servoLoaderStartLeft.setPower(-1.0);
                //servoLifter.setPosition(lifterServoUp);
                liftControl.setPosition(liftControlUp);
                liftExpansion.setPosition(liftExpansionUp);

                stepTimer.reset();
                launchStep = "a1 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a1 - lifter engaged") && stepTimer.milliseconds() > 200)
            {
                liftControl.setPosition(lifterDownPosition);
                liftExpansion.setPosition(lifterDownPosition);
                stepTimer.reset();
                launchStep = "a1 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a1 - drop lifter") && stepTimer.milliseconds() > 200)
            {
                arrPos += 1;
                //robot.magazineEngaged = true;
                stepTimer.reset();
                launchStep = "a2 - get next artifact";
            }
            else if (launchStep.equalsIgnoreCase("a2 - get next artifact") && stepTimer.milliseconds() > 300)
            {
                if(launchType.equalsIgnoreCase("current"))
                {
                    if(stepTimer.milliseconds() > 500)
                    {
                        servoLoaderAssist.setPower(0.0);
                        servoLoaderStartRight.setPower(0.0);
                        servoLoaderStartLeft.setPower(0.0);
                        launchEngaged = false;
                        launchStep = "none";
                    }
                }
                else if(launchType.equalsIgnoreCase("all"))
                {
                    motorShooterLeft.setVelocity((velocityShooting-60));
                    motorShooterRight.setVelocity((velocityShooting-60)); //lower speed for 2nd shot
                    launchStep = "a2 - initial position";
                }
            }
            else if (launchStep.equalsIgnoreCase("a2 - initial position"))
            {
                liftControl.setPosition(liftControlUp);
                liftExpansion.setPosition(liftExpansionUp);

                stepTimer.reset();
                launchStep = "a2 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a2 - lifter engaged") && stepTimer.milliseconds() > 200)
            {
                liftControl.setPosition(lifterDownPosition);
                liftExpansion.setPosition(lifterDownPosition);
                stepTimer.reset();
                launchStep = "a2 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a2 - drop lifter") && stepTimer.milliseconds() > 200)
            {
                arrPos += 1;
                stepTimer.reset();
                launchStep = "a3 - get next artifact";
            }
            else if (launchStep.equalsIgnoreCase("a3 - get next artifact") && stepTimer.milliseconds() > 300)
            {
                motorShooterLeft.setVelocity((velocityShooting-60));
                motorShooterRight.setVelocity((velocityShooting-60)); //lower speed for 3rd shot
                launchStep = "a3 - initial position";
            }
            else if (launchStep.equalsIgnoreCase("a3 - initial position"))
            {
                liftControl.setPosition(liftControlUp);
                liftExpansion.setPosition(liftExpansionUp);

                stepTimer.reset();
                launchStep = "a3 - lifter engaged";
            }
            else if (launchStep.equalsIgnoreCase("a3 - lifter engaged") && stepTimer.milliseconds() > 200)
            {
                liftControl.setPosition(lifterDownPosition);
                liftExpansion.setPosition(lifterDownPosition);
                stepTimer.reset();
                launchStep = "a3 - drop lifter";
            }
            else if (launchStep.equalsIgnoreCase("a3 - drop lifter") && stepTimer.milliseconds() > 200)
            {
                stepTimer.reset();
                launchStep = "a3 - turn off launcher";
            }
            else if (launchStep.equalsIgnoreCase("a3 - turn off launcher") && stepTimer.milliseconds() > 400)
            {
                chambers = robot.getChambers();
                motorShooterLeft.setVelocity((velocityShooting));
                motorShooterRight.setVelocity((velocityShooting));

                if(chambers[0].substring(0, 1).equalsIgnoreCase("p")
                        || chambers[0].substring(0, 1).equalsIgnoreCase("g"))
                {
                    // still one left in the shooting chamber.
                    launchStep = "a3 - initial position";
                }
                else if(chambers[1].substring(0, 1).equalsIgnoreCase("p")
                        || chambers[1].substring(0, 1).equalsIgnoreCase("g"))
                {
                    // still one left in the control hub chamber.
                    liftControl.setPosition(lifterDownPosition);
                    liftExpansion.setPosition(lifterDownPosition);
                    stepTimer.reset();
                    launchStep = "a2 - drop lifter";
                }
                else if(chambers[2].substring(0, 1).equalsIgnoreCase("p")
                        || chambers[2].substring(0, 1).equalsIgnoreCase("g"))
                {
                    // still one left in the expansion hub chamber.
                    liftControl.setPosition(lifterDownPosition);
                    liftExpansion.setPosition(lifterDownPosition);
                    arrPos -= 1;
                    stepTimer.reset();
                    launchStep = "a3 - get next artifact";
                }
                else
                {
                    arrPos = 6;
                    launchEngaged = false;

                    motorShooterLeft.setVelocity((0));
                    motorShooterRight.setVelocity((0));
                    launchStep = "none";
                }
            }
        }
    }

    /*
    public void rotateToOrder()
    {
        motifOrderReturn = robot.axonToOrder(axonTargetPosition);
        axonTargetPosition = motifOrderReturn[0];
        if(!finished)
        {
            if(motifOrderReturn[1] >= 1.0)
            {
                axonDirection = "cw";
                finished = true;
            }
            if(motifOrderReturn[1] <= -1.0)
            {
                axonDirection = "ccw";
                finished = true;
            }
            robot.magazineEngaged = true;
        }
    }

     */

    public void findOrder()
    {
        limelightResult = robot.limelight.getLatestResult();
        if(limelightResult.isValid() && limelightResult != null)
        {
            fresult = limelightResult.getFiducialResults();
            tag = fresult.get(0).getFiducialId();
            /*
            if(tag > 20 || tag < 24)
            {

            }
            else
            {
                tag = fresult.get(1).getFiducialId();
                if(tag > 20 || tag < 24)
                {

                }
                else
                {
                    tag = fresult.get(2).getFiducialId();
                }
            }

            if (fresult.size() > 1 && fresult.get(1) != null)
            {
                test2 = fresult.get(1).getFiducialId();
            }

             */
        }

        if(tag == 21)
        {
            motifOrder = "gpp";
        }
        else if(tag == 22)
        {
            motifOrder = "pgp";
        }
        else if(tag == 23)
        {
            motifOrder = "ppg";
        }
    }

    public void start()
    {
        step = "start";
    }

    public void init()
    {
        timer = new ElapsedTime();
        stepTimer = new ElapsedTime();
        magTimer = new ElapsedTime();
        servoLoaderStartLeft = hardwareMap.get(CRServo.class, "StartLeft");
        servoLoaderStartRight = hardwareMap.get(CRServo.class, "StartRight");

        servoLoaderStartLeft.setDirection(CRServo.Direction.REVERSE);
        servoLoaderStartRight.setDirection(CRServo.Direction.REVERSE);

        servoLoaderAssist = hardwareMap.get(CRServo.class, "LoaderAssist");
        internalLight = hardwareMap.get(Servo.class, "InternalLight");
        //servoLifter = hardwareMap.get(Servo.class, "lifter");

        //axon = hardwareMap.get(CRServo.class, "axon");
        //axonEncoder = hardwareMap.get(AnalogInput.class, "axonEncoder");

        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorShooterLeft = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        motorShooterRight = hardwareMap.get(DcMotorEx.class, "RightShooter");

        motorShooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorShooterRight.setDirection(DcMotorSimple.Direction.FORWARD);


        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");

        internalLight.setPosition(0.4);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        odometry.setOffsets(170, 75, DistanceUnit.MM); // don't know how to get these offsets yet
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setYawScalar(-1.0);
        odometry.resetPosAndIMU();

        nav = new Navigation(odometry);
        nav.setMotors(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

        robot = new ShooterBot();
        robot.setLaunchMotors(motorShooterLeft, motorShooterRight);
        //robot.setMagazine(axon, axonEncoder);
        //robot.setLoadServos(servoLifter, servoLoaderStartLeft, servoLoaderStartRight, servoLoaderAssist);
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

        ll = hardwareMap.get(Limelight3A.class, "limelight");
        robot.setLimelight(ll);
        robot.changePipeline(0);
        robot.startLimelight();

        //robot.magazineEngaged = true;
        axonDirection = "cw";

        axonState = 0;

        odometry.resetPosAndIMU();

        liftControl = hardwareMap.get(Servo.class, "liftControl");
        liftExpansion = hardwareMap.get(Servo.class, "liftExpansion");
        liftControl.setDirection(Servo.Direction.FORWARD);
        liftExpansion.setDirection(Servo.Direction.FORWARD);

        turntable = hardwareMap.get(Servo.class, "axon");

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

        arrPos = 6;
    }

    public void init_loop()
    {
        /*
        if(robot.magazineEngaged)
        {
            robot.axonToPosition(axonTargetPosition, axonDirection);
        }

     */
        findOrder();
        robot.motifOrder = motifOrder;
        chambers = robot.getChambers();

        if(axonState < 10)
        {
            servoLoaderAssist.setPower(1.0);
            servoLoaderStartRight.setPower(1.0);
            servoLoaderStartLeft.setPower(-1.0);
            axonState += 1;
        }
        else if(axonState == 10)
        {
            servoLoaderAssist.setPower(0.0);
            servoLoaderStartRight.setPower(0.0);
            servoLoaderStartLeft.setPower(0.0);
            axonState = 200;
        }

        turntable.setPosition(servoPositions[arrPos]);
        showTelemetry();
    }

    public void showTelemetry()
    {
        telemetry.addData("chambers barrel", chambers[0]);
        telemetry.addData("chambers control", chambers[1]);
        telemetry.addData("chambers expansion", chambers[2]);

        telemetry.addData("tag", tag);
        telemetry.addData("motifOrder", motifOrder);
        telemetry.addData("Step", step);
        telemetry.addData("launchStep", launchStep);
        //telemetry.addData("magazineEngaged", robot.magazineEngaged);
        //telemetry.addData("axon Position", (axonEncoder.getVoltage() / 3.3) * 360);
        telemetry.addData("navXTarget", navXTarget);
        telemetry.addData("navYTarget", navYTarget);
        telemetry.addData("navOrientationTarget", navOrientationTarget);
        telemetry.addData("navOnlyRotate", navOnlyRotate);
        telemetry.addData("navPrecise", navPrecise);
        telemetry.addData("navEngaged", navEngaged);

        telemetry.addData("X Current", nav.getX());
        telemetry.addData("Y Current", nav.getY());
        telemetry.addData("Orientation Current", nav.getOrientationCurrent());

        telemetry.addData("distanceToTarget", distanceToTarget);
        telemetry.addData("distanceToTargetOrientation", distanceToTargetOrientation);

        telemetry.addData("Nav Test", nav.test);


        telemetry.update();
    }

    public void Autonomous()
    {
        if(step.equalsIgnoreCase("start"))
        {
            //velocityShooting = 1440;//1350
            double distanceToGoal = 315;//115
            velocityShooting = (int) ((distanceToGoal - 100) * ((robot.velocityShootingMax - robot.velocityShootingMin) / 200) + robot.velocityShootingMin);
            //timer.reset();
            step = "move shoot pre 1";
        }
        else if(step.equalsIgnoreCase("move shoot pre 1"))
        {
            navXTarget = 0;
            navYTarget = 80;//1500
            navOrientationTarget = 340;
            navOnlyRotate = false;
            navPrecise = true;
            navEngaged = true;
            step = "shoot pre";
        }
        else if(step.equalsIgnoreCase("shoot pre"))
        {
            launchType = "all";
            launchEngaged = true;
            launchStep = "none";
            step = "move collect a1 p1";//"move collect a1 p1";
        }
        else if(step.equalsIgnoreCase("move collect a1 p1") && !launchEngaged)
        {
            servoLoaderAssist.setPower(0.0);
            servoLoaderStartRight.setPower(0.0);
            servoLoaderStartLeft.setPower(0.0);
            motorShooterLeft.setVelocity(0);
            motorShooterRight.setVelocity(0);
            intakeMotor.setPower(1.0);
            navSpeedOverride = -1.0;//0.4, 0.6, 0.7
            navXTarget = 0;//1090, 960, 920, 950, 980, 990
            navYTarget = 1260;
            navOrientationTarget = 0;
            navOnlyRotate = false;
            navPrecise = false;
            navEngaged = true;
            //timer.reset();
            step = "move collect a1 p2";
        }
        else if(step.equalsIgnoreCase("move collect a1 p2") && !navEngaged)
        {
            navXTarget = -200;//1090, 960, 920, 950, 980, 990
            navYTarget = 1850;
            navOrientationTarget = 270;
            navOnlyRotate = false;
            navPrecise = true;
            navEngaged = true;
            step = "move collect a1 p3";
        }
        else if(step.equalsIgnoreCase("move collect a1 p3") && !navEngaged)
        {
            navSpeedOverride = 0.60;//0.4, 0.6, 0.7
            navXTarget = -1000;//1090, 960, 920, 950, 980, 990
            navYTarget = 1850;
            navOrientationTarget = 270;
            navOnlyRotate = false;
            navPrecise = false;
            navEngaged = true;
            step = "rotate bump lever";
        }
        else if(step.equalsIgnoreCase("rotate bump lever") && !navEngaged)
        {
            double distanceToGoal = 115;//115
            velocityShooting = (int) ((distanceToGoal - 100) * ((robot.velocityShootingMax - robot.velocityShootingMin) / 200) + robot.velocityShootingMin);
            noOrder = false;

            navSpeedOverride = -1.0;//0.4, 0.6, 0.7
            navOrientationTarget = 0;
            navOnlyRotate = true;
            navPrecise = false;
            navEngaged = true;
            step = "move bump lever";
        }
        else if(step.equalsIgnoreCase("move bump lever") && !navEngaged)
        {
            //navSpeedOverride = -1.0;//0.4, 0.6, 0.7
            intakeMotor.setPower(0.0);
            navXTarget = -1030;//1090, 960, 920, 950, 980, 990
            navYTarget = 1730;
            navOrientationTarget = 0;
            navOnlyRotate = false;
            navPrecise = false;
            navEngaged = true;
            timer.reset();
            step = "move shoot a1";
        }
        else if(step.equalsIgnoreCase("move shoot a1") && !navEngaged && timer.milliseconds() > 600)
        {
            navXTarget = -250;//200
            navYTarget = 2000;//1850, 1870
            navOrientationTarget = 322;//40 , 45, 42
            navOnlyRotate = false;
            navPrecise = true;
            navEngaged = true;
//            motifOrderReturn = robot.axonToOrder(axonTargetPosition);
//
//            if(motifOrderReturn[0] == 1)
//            {
//                arrPos +=1;
//            }
//            if(motifOrderReturn[0] == -1)
//            {
//                arrPos -=1;
//            }
//            if(motifOrderReturn[0] == 2)
//            {
//                arrPos += 2;
//            }
            step = "shoot a1";
        }
        //else if(step.equalsIgnoreCase("shoot a1") && !navEngaged)
        else if(step.equalsIgnoreCase("shoot a1") && distanceToTarget < 200)
        {
            launchType = "all";
            launchEngaged = true;
            launchStep = "none";
            step = "move collect a2 p1";
        }

        else if(step.equalsIgnoreCase("move collect a2 p1") && !launchEngaged)
        {
            timer.reset();
            intakeMotor.setPower(1.0);
            navSpeedOverride = -1.0;
            navXTarget = -200;//400
            navYTarget = 1260;//1490, 1280
            navOrientationTarget = 270;
            navOnlyRotate = false;
            navPrecise = true;
            navEngaged = true;
            step = "move collect a2 p2";
        }
        else if(step.equalsIgnoreCase("move collect a2 p2") && navEngaged)
        {
            if(timer.milliseconds() > 500)
            {
                // runs the launch motors additional 500ms in case the last ball didn't clear
                servoLoaderAssist.setPower(0.0);
                servoLoaderStartRight.setPower(0.0);
                servoLoaderStartLeft.setPower(0.0);
                motorShooterLeft.setVelocity(0);
                motorShooterRight.setVelocity(0);
            }
        }
        else if(step.equalsIgnoreCase("move collect a2 p2") && !navEngaged)
        {
            servoLoaderAssist.setPower(0.0);
            servoLoaderStartRight.setPower(0.0);
            servoLoaderStartLeft.setPower(0.0);
            motorShooterLeft.setVelocity(0);
            motorShooterRight.setVelocity(0);
            navSpeedOverride = 0.7;
            navXTarget = -1130;//900, 950, 1020, 1070, 1110, -1120
            navYTarget = 1260;//1980
            navOrientationTarget = 270;//315
            navOnlyRotate = false;
            navPrecise = false;
            navEngaged = true;
            timer.reset();
            step = "move collect a2 p3";
            //step = "shoot a2";
        }
        else if(step.equalsIgnoreCase("move collect a2 p3") && !navEngaged)
        {
            navSpeedOverride = 0.8;
            navXTarget = -500;
            navYTarget = 1260;//1980
            navOrientationTarget = 270;//315
            navOnlyRotate = false;
            navPrecise = false;
            navEngaged = true;
            timer.reset();
            step = "move shoot a2 p4";
            //step = "shoot a2";
        }
        else if(step.equalsIgnoreCase("move shoot a2 p4") && !navEngaged)
        {
            navSpeedOverride = -1.0;
            navXTarget = -250;
            navYTarget = 2000;//1850
            /*
            navXTarget = 0; //0
            navYTarget = 2350;//2300// 2350
            */
            navOrientationTarget = 322;//315, 55
            navOnlyRotate = false;
            navPrecise = true;
            navEngaged = true;
            timer.reset();
            step = "mag correct order a2";
        }
        else if(step.equalsIgnoreCase("mag correct order a2") && timer.milliseconds() > 200)
        {
            /*
            double[] axonReturn = new double[2];
            axonReturn = robot.axonToOrder(axonTargetPosition);
            axonTargetPosition = axonReturn[0];
            axonDirection = "cw";
            if(axonReturn[1] == -1.0)
            {
                axonDirection = "ccw";
            }
            else if (axonReturn[1] == -2.0)
            {
                intakeMotor.setPower(1.0); // Turn on intake to bump artifact forward if not all three spots registered
            }
            robot.magazineEngaged = true;
            */
//            motifOrderReturn = robot.axonToOrder(axonTargetPosition);
//1
//            if(motifOrderReturn[0] == 1)
//            {
//                arrPos +=1;
//            }
//            if(motifOrderReturn[0] == -1)
//            {
//                arrPos -=1;
//            }
//            if(motifOrderReturn[0] == 2)
//            {
//                arrPos += 2;
//            }
            step = "shoot a2";
        }
        //else if(step.equalsIgnoreCase("shoot a2") && !navEngaged)
        else if(step.equalsIgnoreCase("shoot a2") && distanceToTarget < 200)
        {
            launchType = "all";
            launchEngaged = true;
            launchStep = "none";
            step = "start timer 3";
        }
        else if(step.equalsIgnoreCase("start timer 3") && !launchEngaged)
        {
            timer.reset();
            step = "turn off launch motors";
        }
        else if(step.equalsIgnoreCase("turn off launch motors"))
        {
            if(timer.milliseconds() > 500)
            {
                // runs the motors a little longer then shuts them down in case the last ball didn't clear
                //intakeMotor.setPower(0.0);
                servoLoaderAssist.setPower(0.0);
                servoLoaderStartRight.setPower(0.0);
                servoLoaderStartLeft.setPower(0.0);
                motorShooterLeft.setVelocity(0);
                motorShooterRight.setVelocity(0);
                step = "start timer 4";
            }
        }
        else if(step.equalsIgnoreCase("start timer 4") && !launchEngaged)
        {
            timer.reset();
            step = "move collect a4 rotate";
        }
        else if(step.equalsIgnoreCase("move collect a4 rotate"))
        {
            intakeMotor.setPower(1.0);
            navSpeedOverride = -1.0;
            navOrientationTarget = 270;
            navOnlyRotate = true;
            //timer.reset();
            navPrecise = true;
            navEngaged = true;
            step = "move collect a4 p1";
        }
        else if(step.equalsIgnoreCase("move collect a4 p1")&& !navEngaged)
        {
            intakeMotor.setPower(1.0);
            navSpeedOverride = -1.0;
            navXTarget = -200;//400
            navYTarget = 820;//670
            navOrientationTarget = 270;
            navOnlyRotate = false;
            timer.reset();
            navPrecise = false;
            navEngaged = true;
            step = "move collect a4 p2";
        }
        else if(step.equalsIgnoreCase("move collect a4 p2") && navEngaged)
        {
            if(timer.milliseconds() > 500)
            {
                // runs the launch motors additional 500ms in case the last ball didn't clear
                servoLoaderAssist.setPower(0.0);
                servoLoaderStartRight.setPower(0.0);
                servoLoaderStartLeft.setPower(0.0);
                motorShooterLeft.setVelocity(0);
                motorShooterRight.setVelocity(0);
            }
        }
        else if(step.equalsIgnoreCase("move collect a4 p2") && !navEngaged)
        {
            servoLoaderAssist.setPower(0.0);
            servoLoaderStartRight.setPower(0.0);
            servoLoaderStartLeft.setPower(0.0);
            motorShooterLeft.setVelocity(0);
            motorShooterRight.setVelocity(0);
            navSpeedOverride = 0.7;//0.6
            navXTarget = -1120;//1200, 1160, 1150, 1190 1140
            navYTarget = 670;//1490, 1280 620
            navOrientationTarget = 270;
            navOnlyRotate = false;
            navPrecise = false;//false
            navEngaged = true;
            step = "move shoot a4";
        }

        else if(step.equalsIgnoreCase("move shoot a4") && !navEngaged)
        {
            navSpeedOverride = -1.0;
            navXTarget = -200;
            navYTarget = 2600;//1850, 2000
            /*
            navXTarget = 0; //0
            navYTarget = 2350;//2300// 2350
            */
            navOrientationTarget = 305;//315, 55, 319
            navOnlyRotate = false;
            navPrecise = true;
            navEngaged = true;
            timer.reset();
            step = "timer a4";
        }
        else if(step.equalsIgnoreCase("timer a4") && timer.milliseconds() > 200)
        {
            /*
            double[] axonReturn = new double[2];
            axonReturn = robot.axonToOrder(axonTargetPosition);
            axonTargetPosition = axonReturn[0];
            axonDirection = "cw";
            if(axonReturn[1] == -1.0)
            {
                axonDirection = "ccw";
            }
            else if (axonReturn[1] == -2.0)
            {
                intakeMotor.setPower(1.0); // Turn on intake to bump artifact forward if not all three spots registered
            }
            robot.magazineEngaged = true;
            */
//            motifOrderReturn = robot.axonToOrder(axonTargetPosition);
//
//            if(motifOrderReturn[0] == 1)
//            {
//                arrPos +=1;
//            }
//            if(motifOrderReturn[0] == -1)
//            {
//                arrPos -=1;
//            }
//            if(motifOrderReturn[0] == 2)
//            {
//                arrPos += 2;
//            }
            step = "shoot a4";
        }
        //else if(step.equalsIgnoreCase("shoot a4") && !navEngaged)
        else if(step.equalsIgnoreCase("shoot a4") && distanceToTarget < 200)
        {
            launchType = "all";
            launchEngaged = true;
            launchStep = "none";
            step = "end";
        }

    }

    public void loop()
    {
        //chambers = robot.getChambers();
        //robot.chambers = chambers;
        //robot.motifOrder = motifOrder;
        //findOrder();
        Autonomous();
        if(launchEngaged)
        {
            //launchArtifact();
            launchArtifactOrder();
        }
        if(navEngaged)
        {
            double[] navReturn = new double[3];
            navReturn = nav.navToPosition(navXTarget, navYTarget, navOrientationTarget, navOnlyRotate, navPrecise, navSpeedOverride, navEngaged);
            if (navReturn[0] == -1)
            {
                navEngaged = false;
            }
            distanceToTarget = navReturn[1];
            distanceToTargetOrientation = navReturn[2];
        }
        /*
        if(robot.magazineEngaged)
        {
            robot.axonToPosition(axonTargetPosition, axonDirection);
        }

         */
        turntable.setPosition(servoPositions[arrPos]);
        showTelemetry();
    }
}
