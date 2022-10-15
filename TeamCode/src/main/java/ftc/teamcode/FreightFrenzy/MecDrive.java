package ftc.teamcode.FreightFrenzy;

//import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;
import org.firstinspires.ftc.teamcode.control.MecanumDrive;
import org.firstinspires.ftc.teamcode.drivers.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;
import org.firstinspires.ftc.teamcode.trajectory.StateMTrajectory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//import ftc.teamcode.Toggler;

@TeleOp
public class MecDrive extends LinearOpMode {


        private DcMotorEx FL;
        private DcMotorEx FR;
        private DcMotorEx RL;
        private DcMotorEx RR;
        private Servo servo1;
        private Servo servo2;
        private double leftStickX;
        private double leftStickY;
        private double rightStickX;
        double motor_speed;
        //MaxSonarI2CXL sensor;
        StateMTrajectory trajectory;
        SkyStoneDriveBase skyStoneDriveBase;
        //OpenCvCamera camera;

        AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;
        TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();


        // LynxDcMotorController ctrl;
        LynxModule module;


        // private TouchSensor touch;
        // private RevTouchSensor touch;
        // private DigitalChannel touch;



        @Override
        public void runOpMode() throws InterruptedException {


            trackingWheelIntegrator = new TrackingWheelIntegrator();

            //mapping out the robot
            FL= (DcMotorEx) hardwareMap.get(DcMotor.class, "FL");
            FR= (DcMotorEx) hardwareMap.get(DcMotor.class, "FR");
            RL= (DcMotorEx) hardwareMap.get(DcMotor.class, "RL");
            RR= (DcMotorEx) hardwareMap.get(DcMotor.class, "RR");
            servo1= (Servo) hardwareMap.get(Servo.class, "servo1");
            servo2= (Servo) hardwareMap.get(Servo.class, "servo2");
            //sensor= (MaxSonarI2CXL) hardwareMap.get(MaxSonarI2CXL.class, "sensor");
            skyStoneDriveBase = new SkyStoneDriveBase();
            motor_speed = 0;
            skyStoneDriveBase.init(hardwareMap);
            skyStoneDriveBase.resetEncoders();
            skyStoneDriveBase.enableBrake(true);
            skyStoneDriveBase.enablePID();
            Globals.robot=skyStoneDriveBase;
            Globals.driveBase=skyStoneDriveBase;
            Globals.trackingWheelIntegrator = trackingWheelIntegrator;
            Globals.odoModule = module;
            Globals.opMode = this;
            Globals.robot.enableBrake(true);

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            boolean AutomationLastState = false;
            acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.08, 1, .05);

            telemetry.setMsTransmissionInterval(20);
            telemetry.addData("Status", "Initialized");
            telemetry.update();


            waitForStart();

            while (opModeIsActive()) {

                if (gamepad2.x) {         //If button is pressed Auto aline will run. if not normally gamepad works
                    if (!AutomationLastState) {
                        //clearEnc();
                        trackingWheelIntegrator.setFirstTrackingVal(0,0);
                        //buildTrajectory();

                    }
                    AutomationLastState = true;
                    trajectory.followInteration();

                }
                else {
                    runGamepad();
                    if (AutomationLastState) {
                        AutomationLastState = false;
                        Globals.robot.enableBrake(true);
                        Globals.robot.disablePID();

                        trajectory.reset();

                    }
                }
            }
        }

        void runGamepad() {

            if (gamepad1.left_stick_x > .01) {
                leftStickX = gamepad1.left_stick_x;
            }
            else if (gamepad1.left_stick_x < -.01) {
                leftStickX = gamepad1.left_stick_x;
            }
            else {
                leftStickX = 0;

            }
            if (gamepad1.left_stick_y > .01 ) {
                leftStickY = gamepad1.left_stick_y;
            }
            else if (gamepad1.left_stick_y < -.01) {
                leftStickY = gamepad1.left_stick_y;
            }
            else {
                leftStickY = 0;

            }
            if (gamepad1.right_stick_x > .01 ) {
                rightStickX = gamepad1.right_stick_x;
            }
            else if (gamepad1.right_stick_x < -.01) {
                rightStickX = gamepad1.right_stick_x;
            }
            else {
                rightStickX = 0;

            }
            /*if (gamepad1.left_stick_y > .03 && gamepad1.left_stick_y < -.03) {
                leftStickY = gamepad1.left_stick_y;
            }
            else {
                leftStickY = 0;
            }

             */

            // This code sends data to the standard output about the motors/sticks
            /*
            telemetry.addData("Y-stick", gamepad1.left_stick_y);
            telemetry.addData("X-stick", gamepad1.left_stick_x);
            telemetry.addData("Turning", gamepad1.right_stick_x);
            telemetry.addData("Y-stick", leftStickY);
            telemetry.addData("X-stick", rightStickX);
            telemetry.update();
            */
            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("RL", RL.getCurrentPosition());
            telemetry.addData("RR", RR.getCurrentPosition());
            //telemetry.addData("servo_position", servo_test.getPosition());
            telemetry.update();

            // Edit this block to change the speed (always keep rightStickX below the others)
            MecanumDrive.cartesian(Globals.robot,
                    -leftStickY * motor_speed, // Main
                    leftStickX * motor_speed, // Strafe
                    rightStickX * motor_speed); // Turn

            if (gamepad1.right_bumper) {
                servo1.setPosition(1);
                servo2.setPosition(.4);
            }

            if (gamepad1.left_bumper) {
                servo1.setPosition(.40);
                servo2.setPosition(1);
            }

            double FL_motor_rotations;
            double FR_motor_rotations;
            double RL_motor_rotations;
            double RR_motor_rotations;
            double sensor_dist;

            FL_motor_rotations = (FL.getCurrentPosition()/28) / 11.73 * 12;
            FR_motor_rotations = (FL.getCurrentPosition()/28) / 11.73 * 12;
            RL_motor_rotations = (FL.getCurrentPosition()/28) / 11.73 * 12;
            RR_motor_rotations = (FL.getCurrentPosition()/28) / 11.73 * 12;
            //sensor_dist = sensor.getDistanceSync();



            //while (FL_motor_rotations < 23) {
            //    FL_motor_rotations = (FL.getCurrentPosition()/28) / 11.73 * 12;
            //    if (sensor.getDistanceSync() < 20) {
            //        FL.setMotorDisable();
            //        FR.setMotorDisable();
            //        RL.setMotorDisable();
            //        RR.setMotorDisable();

            //    }
            //    MecanumDrive.cartesian(Globals.robot, -.15, 0, 0);
            //}
            //while (FL_motor_rotations > 0) {
            //    FL_motor_rotations = (FL.getCurrentPosition()/28) / 11.73 * 12;
            //    if (sensor.getDistanceSync() < 20) {
            //        FL.setMotorDisable();
            //        FR.setMotorDisable();
            //        RL.setMotorDisable();
            //        RR.setMotorDisable();
            //    }
            //    MecanumDrive.cartesian(Globals.robot, 0, .15, 0);
            //}

            //while (sensor.getDistance())

            if (gamepad1.dpad_up) {
                motor_speed = motor_speed + .05;
            }
            if (gamepad1.dpad_down) {
                motor_speed = motor_speed - .05;
            }

            telemetry.addData("FL inches ", FL_motor_rotations);
            telemetry.addData("FR inches ", FR_motor_rotations);
            telemetry.addData("RL inches ", RL_motor_rotations);
            telemetry.addData("RR inches ", RR_motor_rotations);
            //telemetry.addData("Sensor reading", sensor_dist);
            telemetry.addData("Motor Speed", motor_speed);



    }
}
