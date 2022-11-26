package ftc.teamcode.FreightFrenzy;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.control.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class Autotest extends LinearOpMode {

    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx RL;
    private DcMotorEx RR;
    private Servo servo1;
    private Servo servo2;
    private Servo clawServo;
    private double leftStickX;
    private double leftStickY;
    private double lastpos;
    double motor_speed;
    private int signalNumber;
    OpenCvCamera phoneCam;
    SkyStoneDriveBase skyStoneDriveBase;

    public void move(int direction, int encodernum) {
        int currentPos;
        currentPos = FL.getCurrentPosition();
        telemetry.addData("currentPos", currentPos);
        telemetry.update();
        if (direction == 1) {
            if (-FL.getCurrentPosition() - currentPos < encodernum) {
                telemetry.addData("FL: ", FL.getCurrentPosition());
                MecanumDrive.cartesian(Globals.robot, .25, 0, 0);
                telemetry.addData("currentPos", currentPos);
                telemetry.update();
            }
        }
        if (direction == 2) {
          if (-FL.getCurrentPosition() + currentPos < encodernum) {
                telemetry.addData("FL: ", FL.getCurrentPosition());
                MecanumDrive.cartesian(Globals.robot, -.25, 0, 0);
                telemetry.addData("currentPos", currentPos);
                telemetry.update();
           }
        }
        if (direction == 3) {
           if (FL.getCurrentPosition()  + currentPos < encodernum) {
                telemetry.addData("FL: ", FL.getCurrentPosition());
                MecanumDrive.cartesian(Globals.robot, 0, -.25, 0);
                telemetry.addData("currentPos", currentPos);
                telemetry.update();
            }
        }
        if (direction == 4) {
          if (FL.getCurrentPosition() + currentPos < encodernum) {
                telemetry.addData("FL: ", FL.getCurrentPosition());
                MecanumDrive.cartesian(Globals.robot, 0, .25, 0);
                telemetry.addData("currentPos", currentPos);
                telemetry.update();
           }
        }
        MecanumDrive.cartesian(Globals.robot, 0, 0, 0);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        FL= (DcMotorEx) hardwareMap.get(DcMotor.class, "FL");
        FR= (DcMotorEx) hardwareMap.get(DcMotor.class, "FL");
        RL= (DcMotorEx) hardwareMap.get(DcMotor.class, "FL");
        RR= (DcMotorEx) hardwareMap.get(DcMotor.class, "FL");
        servo1= (Servo) hardwareMap.get(Servo.class, "servo1");
        servo2= (Servo) hardwareMap.get(Servo.class, "servo2");

        clawServo= (Servo) hardwareMap.get(Servo.class, "clawServo");
        skyStoneDriveBase = new SkyStoneDriveBase();
        skyStoneDriveBase.init(hardwareMap);
        skyStoneDriveBase.resetEncoders();
        skyStoneDriveBase.enableBrake(true);
        skyStoneDriveBase.enablePID();

        Globals.opMode = this;
        Globals.robot=skyStoneDriveBase;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        AprilTagAutonomousInitDetection vision = new AprilTagAutonomousInitDetection();
        AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(0.166, 98.267716535 , 98.267716535,
        160, 120);
        phoneCam.setPipeline(pipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
// nothing in here lol
            }
        });



        telemetry.setMsTransmissionInterval(20);

        waitForStart();

//        while(opModeIsActive()) {
//            coneColor = 3;
//            telemetry.addData("colournum", pipeline.signalNumber);
//            telemetry.update();
//
//
//        }
        while (opModeIsActive()) {
            if (gamepad1.y) {
                move(1, 900);
            }
            if (gamepad1.x) {
                move(1, 900);
                move(3, 900);
            }
            if (gamepad1.b) {
                move(1, 900);
                move(4, 900);
            }
        }
    }
}
