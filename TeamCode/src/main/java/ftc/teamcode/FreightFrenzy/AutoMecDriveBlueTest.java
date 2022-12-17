package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import net.frogbots.skystone.control.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import net.frogbots.skystone.drivers.MaxSonarI2CXL;
import net.frogbots.skystone.meta.opmode.FrogOpMode;


import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
@Disabled
@Autonomous
public class AutoMecDriveBlueTest extends FrogOpMode
{
    MaxSonarI2CXL RightSonar;
    MaxSonarI2CXL BackSonar;
    MaxSonarI2CXL LeftSonar;
    MaxSonarI2CXL FrontSonar;
    int signalNum;
    OpenCvCamera phoneCam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;


    double fx = 98.267716535;
    double fy = 98.267716535;
    double cx = 160;
    double cy = 120;


    double tagsize = 0.166;


    int left = 1;
    int middle = 2;
    int right = 3;



    AprilTagDetection tagOfInterest = null;


    AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;


    @Override
    protected void frog_run()
    {
        RightSonar = hardwareMap.get(MaxSonarI2CXL.class, "RightSonar");
        BackSonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");
        //FrontSonar = hardwareMap.get(MaxSonarI2CXL.class, "FrontDistance");
        LeftSonar = hardwareMap.get(MaxSonarI2CXL.class, "LeftSonar");

        telemetry.setMsTransmissionInterval(20);

       waitForStart();

//        while (opModeIsActive())
//        {

        for(int i = 0; i < 4; i++) {
            double err = gyroUtils.gyroStraight(acclCtrl, .2, 0, .017);
        }
        telemetry.addData("signalnum = ", signalNum);
        telemetry.update();

        signalNum = tagOfInterest.id;

        switch (signalNum) {

            case 1:
                LeftDist(126.54, 0);
                ForwardDist(120.0, 0);
                break;

            case 2:
                LeftDist(126.54, 0);
                ForwardDist(120.0, 0);
                RightDist(84, 0);
                RotateAngle(90);
                break;

            case 3:
               LeftDist(125,0);
               ForwardDist(115, 0);
               RightDist(25,0);
               RotateAngle(-160);
               sleep(100);
               RightLDist(90, -170);
                break;

        }

        }



 //       }


    @Override
    protected void frog_init()
    {
        acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.08, 1, .05);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        phoneCam.setPipeline(aprilTagDetectionPipeline);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(720,480 , OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        int signalNum = SignalNum();

        //phoneCam.stopStreaming();

        robot.driveTrain.enablePID();
        robot.sensors.imu.init();
    }

    public void ForwardDist(double dist, double target) {
        double bcm = 0;
        bcm = BackSonar.getDistanceSync(35);
        telemetry.addData("bcm", bcm);
        telemetry.update();
        while (bcm < dist) {
            double err = gyroUtils.gyroStraight(acclCtrl, .2, target, .017);
            bcm = BackSonar.getDistanceSync();
            telemetry.addData("bcm", bcm);
            telemetry.update();
        }

        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();

    }

    public void BackDist(double dist, double target) {
        double bcm = 0;
        bcm = BackSonar.getDistanceSync(35);
        telemetry.addData("bcm" , bcm);
        telemetry.update();
        while (bcm < dist) {
            double err = gyroUtils.gyroStraight(acclCtrl, -0.2, target, .017);
            bcm = BackSonar.getDistanceSync(35);
            telemetry.addData("bcm" , bcm);
            telemetry.update();
        }
        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();
    }

    public void LeftLDist(double dist, double target) {
        double lcm = 0;
        lcm = LeftSonar.getDistanceSync(35);
        telemetry.addData("lcm", lcm);
        telemetry.update();

        while (dist < lcm) {
            double err = gyroUtils.gyroStrafe(acclCtrl, -.2, 0, .013);
            lcm = LeftSonar.getDistanceSync(35);
            telemetry.addData("lcm", lcm);
            telemetry.update();
        }
        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();
    }

    public void RightLDist(double dist, double target) {
        double lcm = 0;
        lcm = LeftSonar.getDistanceSync(35);
        telemetry.addData("lcm", lcm);
        telemetry.update();

        while (dist > lcm) {
            double err = gyroUtils.gyroStrafe(acclCtrl, .2, target, .013);
            lcm = LeftSonar.getDistanceSync(35);
            telemetry.addData("lcm", lcm);
            telemetry.update();
        }
        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();
    }
        public void LeftDist(double dist, double target) {
        double rcm = 0;
        rcm = RightSonar.getDistanceSync(35);
        telemetry.addData("rcm" , rcm);
        telemetry.update();

        while (rcm < dist) {
            double err = gyroUtils.gyroStrafe(acclCtrl, -.2, target, .013);
            rcm = RightSonar.getDistanceSync(35);
            telemetry.addData("rcm" , rcm);
            telemetry.update();
        }

           // robot.driveTrain.stopMotors();
           // acclCtrl.clr();
    }

    public void RightDist(double dist, double target) {
        double rcm = 0;
        rcm = RightSonar.getDistanceSync( 35);
        telemetry.addData("rcm" , rcm);
        telemetry.update();

        while (dist < rcm) {
            double err = gyroUtils.gyroStrafe(acclCtrl, .2, target, .013);
            rcm = RightSonar.getDistanceSync(35);
            telemetry.addData("rcm" , rcm);
            telemetry.update();
        }

        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();

    }
    public void RotateAngle(double angle) {
        double err = gyroUtils.gyroRotate(acclCtrl, angle, .015);

        if (angle < 0) {
            while (err < 1) {
                err = gyroUtils.gyroRotate(acclCtrl, angle, .015);
            }
        }
        else {
            while (err > 1) {
                err = gyroUtils.gyroRotate(acclCtrl, angle, .015);
            }
        }

          //robot.driveTrain.stopMotors();
          //acclCtrl.clr();
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }

    public int SignalNum() {
        int signalNumber = 0;

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        if(tagOfInterest == null || tagOfInterest.id == left) {
            signalNumber = 1;
        }else if (tagOfInterest.id == middle) {
            signalNumber = 2;
        }else if(tagOfInterest.id == right) {
            signalNumber = 3;
        }
        return signalNumber;
    }

}
