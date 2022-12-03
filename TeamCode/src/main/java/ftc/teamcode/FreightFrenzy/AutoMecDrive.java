package ftc.teamcode.FreightFrenzy;

//import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.skystone.drivers.MaxSonarI2CXL;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.control.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;

import org.openftc.easyopencv.OpenCvCamera;


@Autonomous()
public class AutoMecDrive extends LinearOpMode {

    OpenCvCamera phoneCam;

    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx RL;
    private DcMotorEx RR;
    private Servo servo1;
    private Servo servo2;
    private Servo clawServo;

    SkyStoneDriveBase skyStoneDriveBase;

    private double maxPos = 1;
    private double minPos = 0;


    //code for lift
    private double lPos = 0.48;
    private double rPos = 0.5;
    private double cPos = 0;
    private double crPos = 1;

    private double coneLVL = 0;
    private double rconeLVL = 0.98;

    private double lowJunc = 0.25;
    private double rlowJunc = 0.7;

    private double midJunc = 0.459;
    private double rmidJunc = 0.48;

    private double highJunc = 0.96;
    private double rhighJunc = 0;

    //claw Servo init + var

    private double clClose = 0;
    private double cl = clClose;
    private double clOpen = 1;

    //sensors
    MaxSonarI2CXL RightSonar;
    MaxSonarI2CXL BackSonar;
    MaxSonarI2CXL LeftSonar;

    double RcmToInch = 0;
    double Rinch = 0;
    double LcmToInch = 0;
    double Linch = 0;
    double BcmToInch = 0;
    double Binch = 0;

    // LynxDcMotorController ctrl;
    LynxModule module;

    public void move(int direction, double sensorDistance) {
        if (direction == 1) {
            //while (Binch < sensorDistance) {
                MecanumDrive.cartesian(Globals.robot, .25, 0, 0);

            //}
        }
        if (direction == 2) {
            //while (Binch < sensorDistance) {
                MecanumDrive.cartesian(Globals.robot, -.25, 0, 0);
            //}
        }
        if (direction == 3) {
            //while (Rinch < sensorDistance) {
                MecanumDrive.cartesian(Globals.robot, 0, -.25, 0);

            //}
        }
        if (direction == 4) {
            //while (Rinch < sensorDistance) {
                MecanumDrive.cartesian(Globals.robot, 0, .25, 0);

            //}
        }
        MecanumDrive.cartesian(Globals.robot, 0, 0, 0);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        //mapping out the robot
        FL = (DcMotorEx) hardwareMap.get(DcMotor.class, "FL");
        FR = (DcMotorEx) hardwareMap.get(DcMotor.class, "FR");
        RL = (DcMotorEx) hardwareMap.get(DcMotor.class, "RL");
        RR = (DcMotorEx) hardwareMap.get(DcMotor.class, "RR");
        servo1 = (Servo) hardwareMap.get(Servo.class, "servo1");
        servo2 = (Servo) hardwareMap.get(Servo.class, "servo2");
        clawServo = (Servo) hardwareMap.get(Servo.class, "clawServo");
        RightSonar = hardwareMap.get(MaxSonarI2CXL.class, "RightSonar");
        LeftSonar = hardwareMap.get(MaxSonarI2CXL.class, "LeftSonar");
        //FrontSonar = hardwareMap.get(MaxSonarI2CXL.class, "FrontDistance");
        BackSonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");
        skyStoneDriveBase = new SkyStoneDriveBase();
        skyStoneDriveBase.init(hardwareMap);
        skyStoneDriveBase.resetEncoders();
        skyStoneDriveBase.enableBrake(true);
        skyStoneDriveBase.enablePID();
        Globals.robot = skyStoneDriveBase;
        Globals.driveBase = skyStoneDriveBase;
        Globals.opMode = this;
        Globals.robot.enableBrake(true);

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        waitForStart();

        while (opModeIsActive()) {
            RcmToInch = RightSonar.getDistanceSync();
            BcmToInch = BackSonar.getDistanceSync();
            // LcmToInch = LeftSonar.getDistanceSync();

            Rinch = RcmToInch / 2.54;
            // Linch = LcmToInch / 2.54;
            Binch = BcmToInch / 2.54;

            telemetry.addData("Right Dist inch", Rinch + 3.5);
            telemetry.addData("Back Dist inch", Binch + 1.5);
            // telemetry.addData("Left Dist inch", Linch + 1);
            telemetry.setMsTransmissionInterval(20);
            telemetry.update();

           if (Rinch < 58.6) {
                move(3, 58.6);
            }
           else {

              MecanumDrive.cartesian(Globals.robot, 0, 0, 0);

           }
        }
    }


}



