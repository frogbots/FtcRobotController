package ftc.teamcode.FreightFrenzy;

//import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.control.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;


@TeleOp
public class MecDrive2 extends LinearOpMode {

    OpenCvCamera phoneCam;

        private DcMotorEx FL;
        private DcMotorEx FR;
        private DcMotorEx RL;
        private DcMotorEx RR;
        private Servo servo1;
        private Servo servo2;
        private Servo clawServo;
        private double leftStickX;
        private double leftStickY;
        private double rightStickX;
        double motor_speed;
        //MaxSonarI2CXL sensor;

        SkyStoneDriveBase skyStoneDriveBase;





    AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;


    private double maxPos = 1;
    private  double minPos = 0;


//code for lift
    private double lPos = 0.48;
    private double rPos = 0.5;
    private double cPos = 0;
    private double crPos = 1;

    private double coneLVL = 0;
    private double rconeLVL = 0.98;

    private double lowJunc = 0.25 ;
    private double rlowJunc = 0.7;

    private double midJunc = 0.459;
    private double rmidJunc = 0.48;

    private double highJunc = 0.96;
    private double rhighJunc =  0 ;

    //claw Servo init + var

    private double clClose = 0;
    private double cl = clClose;
    private double clOpen = 1;

        // LynxDcMotorController ctrl;
        LynxModule module;



        @Override
        public void runOpMode() throws InterruptedException {

            //mapping out the robot
            FL= (DcMotorEx) hardwareMap.get(DcMotor.class, "FL");
            FR= (DcMotorEx) hardwareMap.get(DcMotor.class, "FR");
            RL= (DcMotorEx) hardwareMap.get(DcMotor.class, "RL");
            RR= (DcMotorEx) hardwareMap.get(DcMotor.class, "RR");
            servo1= (Servo) hardwareMap.get(Servo.class, "servo1");
            servo2= (Servo) hardwareMap.get(Servo.class, "servo2");
            clawServo= (Servo) hardwareMap.get(Servo.class, "clawServo");
            //sensor= (MaxSonarI2CXL) hardwareMap.get(MaxSonarI2CXL.class, "sensor");
            skyStoneDriveBase = new SkyStoneDriveBase();
            motor_speed = 0;
            skyStoneDriveBase.init(hardwareMap);
            skyStoneDriveBase.resetEncoders();
            skyStoneDriveBase.enableBrake(true);
            skyStoneDriveBase.enablePID();
            Globals.robot=skyStoneDriveBase;
            Globals.driveBase=skyStoneDriveBase;
            Globals.opMode = this;
            Globals.robot.enableBrake(true);

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);




            boolean AutomationLastState = false;
            acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.08, 1, .05);

            telemetry.setMsTransmissionInterval(20);
            telemetry.addData("Status", "Initialized");
            telemetry.update();


            waitForStart();

            while (opModeIsActive()) {

                if (gamepad2.x) {         //If button is pressed Auto aline will run. if not normally gamepad works
                    if (!AutomationLastState) {

                    }
                    AutomationLastState = true;

                }
                else {
                    runGamepad();
                    if (AutomationLastState) {
                        AutomationLastState = false;
                        Globals.robot.enableBrake(true);
                        Globals.robot.disablePID();

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

            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("RL", RL.getCurrentPosition());
            telemetry.addData("RR", RR.getCurrentPosition());
            telemetry.addData("lvldiff", coneLVL);
            telemetry.addData("lPos", servo1.getPosition());
            telemetry.addData("rPos:", servo2.getPosition());
            telemetry.addData("Motor Speed", motor_speed);



            telemetry.update();


            MecanumDrive.cartesian(Globals.robot,
                    -leftStickY * .25, // Main
                    leftStickX * .25 , // Strafe
                    rightStickX * .20); // Turn


            if (gamepad1.y) {
                clawServo.setPosition(-0.2);
            }
            if (gamepad1.x) {
                clawServo.setPosition(.4);
            }

            //lift positions
            servo2.setPosition(rPos);
            servo1.setPosition(lPos);


            if(gamepad1.right_bumper) {
                lPos = coneLVL;
                rPos = rconeLVL;

            }
            if(gamepad1.left_bumper) {
                lPos = lowJunc;
                rPos = rlowJunc;

            }
            if(gamepad1.left_trigger == 1) {
                lPos = midJunc;
                rPos= rmidJunc;
            }
            if(gamepad1.right_trigger == 1) {
                lPos = highJunc;
                rPos = rhighJunc;
            }

            if(gamepad1.dpad_up) {
                if(lPos < maxPos) {
                    lPos = lPos + .02;
                    rPos = rPos - .02;

                }

            }
            if(gamepad1.dpad_down) {
                if (lPos > minPos) {
                    lPos = lPos - .02;
                    rPos = rPos + .02;
                }
            }


    }
}
