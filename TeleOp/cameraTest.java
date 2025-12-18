package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class cameraTest extends LinearOpMode
{
    Robot bot;
    Limelight3A limelight;
    LLResult limelightResult;
    Pose3D botPose;

    GoBildaPinpointDriver odometry;
    OpenCvWebcam testCam = null;
    double distanceFromTag = 0;

    /*
    int leftGreenLow;
    int leftGreenMax;
    int leftPurpleLow;
    int leftPurpleMax;
    int middleGreenLow;
    int middleGreenMax;
    int middlePurpleLow;
    int middlePurpleMax;
    int rightGreenLow;
    int rightGreenMax;
    int rightPurpleLow;
    int rightPurpleMax;
     */

    int purpleMin;
    int purpleMax;
    int greenMin;
    int greenMax;
    String chamberLeft, chamberMiddle, chamberRight = "none";
    public void telemetryy()
    {
        telemetry.addData("Distance from tag", distanceFromTag);
        if(limelightResult.isValid() && limelightResult != null)
        {
            telemetry.addData("MT2 target X", limelightResult.getTx());
            telemetry.addData("MT2 target Y", limelightResult.getTy());
            telemetry.addData("MT2 target area", limelightResult.getTa());
            telemetry.addData("MT2 bot pose", botPose.toString());
        }
        telemetry.update();
    }
    public void limelightTest()
    {
        limelight.updateRobotOrientation(bot.getOrientationCurrent());      //In all of the examples I found this uses the internal IMU.
        //If I remember right the internal IMU uses values -180 to +180, and our getOrientation code uses 0-360
        //This might cause problems with orientation, but should be a simple fix.

        limelightResult = limelight.getLatestResult();
        if(limelightResult.isValid() && limelightResult != null)
        {
            botPose = limelightResult.getBotpose_MT2();
            distanceFromTag = bot.getDistanceFromTag(limelightResult.getTa());
        }
    }
    public void ininit()
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        odometry.setOffsets(172, -74, DistanceUnit.MM); // don't know how to get these offsets yet
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odometry.setYawScalar(-1.0);
        odometry.resetPosAndIMU();


        bot = new Robot(odometry);
        bot.reset();


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Internal Cam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());//not sure what this does or how it works...

        testCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        testCam.setPipeline(new magRecognition());
        testCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                testCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                //do nothing (yet?)
            }
        });

        waitForStart();
        limelight.start();
    }
    class  magRecognition extends OpenCvPipeline
    {
        Mat YCbCr = new Mat();
        Mat outPut = new Mat();

        Scalar color = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input)
        {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2HSV);

            Rect leftRect = new Rect(100, 420, 250, 250);
            Rect middleRect = new Rect(450, 1, 300, 300);
            Rect rightRect = new Rect(930, 420, 250, 250);

            input.copyTo(outPut);

            Imgproc.rectangle(outPut, leftRect, color, 2);
            Imgproc.rectangle(outPut, middleRect, color, 2);
            Imgproc.rectangle(outPut, rightRect, color, 2);


            Mat leftCrop = YCbCr.submat(leftRect);
            Mat middleCrop = YCbCr.submat(middleRect);
            Mat rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(middleCrop, middleCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);


            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(middleCrop);
            Scalar rightavg = Core.mean(rightCrop);

            int leftValue = (int) leftavg.val[1];
            int midValue = (int) midavg.val[1];
            int rightValue = (int) rightavg.val[1];

            if(leftValue >= greenMin && leftValue <= greenMax)
            {
                chamberLeft = "green";
            }
            else if(leftValue >= purpleMin && leftValue <= purpleMax)
            {
                chamberLeft = "purple";
            }

            if(midValue >= greenMin && midValue <= greenMax)
            {
                chamberMiddle = "green";
            }
            else if(midValue >= purpleMin && midValue <= purpleMax)
            {
                chamberMiddle = "purple";
            }

            if(rightValue >= greenMin && rightValue <= greenMax)
            {
                chamberRight = "green";
            }
            else if(rightValue >= purpleMin && rightValue <= purpleMax)
            {
                chamberRight = "purple";
            }

            /*
            if (middleColorValue == 0 && runtime.milliseconds() > 6000)
            {
                middleColorValue = (int)leftavg.val[0];
                rightColorValue = (int)rightavg.val[0];
            }
            else if (middleColorValue > (int) leftavg.val[0] + 5)
            {
                propPosition = "Middle";
            }
            else if (rightColorValue > (int) rightavg.val[0] + 5)
            {
                propPosition = "Right";
            }
            else
            {
                propPosition = "Left";
            }

             */
            return(outPut);
        }
    }

    public void runOpMode()
    {
        ininit();
        while(opModeIsActive())
        {
            limelightTest();
            telemetryy();
        }
    }
}
