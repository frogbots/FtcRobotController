package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.skystone.control.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import net.frogbots.skystone.drivers.MaxSonarI2CXL;
import net.frogbots.skystone.meta.opmode.FrogOpMode;

import ftc.teamcode.FreightFrenzy.AutoNavigation;
import ftc.teamcode.FreightFrenzy.Dr4bLift;
import  ftc.teamcode.FreightFrenzy.OverheadClaw;
import ftc.teamcode.FreightFrenzy.BeamBreakSensor;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
public class AutoMecDriveBlue extends FrogOpMode
{



    int signalNum = 1;
    OpenCvCamera phoneCam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 98.267716535;
    double fy = 98.267716535;
    double cx = 160;
    double cy = 120;


    double tagsize = 0.166;


    int left = 1;
    int middle = 2;
    int right = 3;



    AprilTagDetection tagOfInterest = null;

    BeamBreakSensor beamBreak;
    OverheadClaw claw = new OverheadClaw();
    Dr4bLift lift = new Dr4bLift();
    AutoNavigation autoNav = new AutoNavigation();
    AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;



    @Override
    protected void frog_run()
    {
        autoNav.RightSonar = hardwareMap.get(MaxSonarI2CXL.class, "RightSonar");
        autoNav.BackSonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");
        //autoNav.FrontSonar = hardwareMap.get(MaxSonarI2CXL.class, "FrontDistance");
//        autoNav.LeftSonar = hardwareMap.get(MaxSonarI2CXL.class, "LeftSonar");
        lift.servo1= (Servo) hardwareMap.get(Servo.class, "servo1");
        lift.servo2= (Servo) hardwareMap.get(Servo.class, "servo2");
        claw.clawServo= (Servo) hardwareMap.get(Servo.class, "clawServo");

        telemetry.setMsTransmissionInterval(20);

       waitForStart();



        signalNum = tagOfInterest.id;

        // right parking

        //autoNav.ForwardtoCone(5,.2, 0);
        //autoNav.RightDist(20,0);
        //sleep(1000);
        //autoNav.RightDist(75,0);
        lift.MediumJunction();
        autoNav.ForwardDist(135, .2,0);
        autoNav.BackDist(106, -.2,0);
        autoNav.RotateAngle(90);
        autoNav.ForwardtoCone(11,.2,90);
        robot.driveTrain.stopMotors(); //setMotorPowers(0,0,0,0);
        sleep(3000 );
        claw.ClawOpen();
        sleep(500);
        claw.ClawClose();
        autoNav.ForwardtoCone(30,-.2,90);
        robot.driveTrain.stopMotors();
        sleep(500);
        autoNav.RotateAngle(-1);
        lift.ConeLVL1();
        sleep(500);

        /*autoNav.ForwardDist(124,0);
        autoNav.RightDist(20,0);
        robot.driveTrain.stopMotors();
        sleep(500);
        autoNav.RotateAngle(-90);
        sleep(1000 );
        claw.ClawOpen();
        sleep(500);
        lift.ConeLVL1();
        autoNav.ForwardtoCone(20,.2,-90);
        robot.driveTrain.stopMotors(); //setMotorPowers(0,0,0,0);
        sleep(500);
        claw.ClawClose();
        sleep(500);
        autoNav.ForwardtoCone(50,-.2,-90);
        robot.driveTrain.stopMotors(); //setMotorPowers(0,0,0,0);*/
        //autoNav.RotateAngle(-1);
        //sleep(500 );




// parking code

        switch (signalNum) {

            case 1:
               autoNav.Navigate(0, 120, 120);
                break;

            case 2:
               autoNav.Navigate(0, 60, 120);
                break;

            case 3:
               autoNav.Navigate(0, 20, 120);
                break;

        }

        }



    //}


    @Override
    protected void frog_init()
    {
        acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.08, 1, .05);
        autoNav = new AutoNavigation();
        lift = new Dr4bLift();
        claw = new OverheadClaw();
        beamBreak = new BeamBreakSensor();


        autoNav.acclCtrl = acclCtrl;
        autoNav.gyroUtils = gyroUtils;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        phoneCam.setPipeline(aprilTagDetectionPipeline);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(720,480 , OpenCvCameraRotation.UPSIDE_DOWN);
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
