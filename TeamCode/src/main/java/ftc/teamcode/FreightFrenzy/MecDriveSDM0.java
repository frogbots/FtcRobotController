package ftc.teamcode.FreightFrenzy;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.skystone.drivers.MaxSonarI2CXL;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;
import org.firstinspires.ftc.teamcode.control.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;

@TeleOp
public class MecDriveSDM0 extends LinearOpMode {


    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx RL;
    private DcMotorEx RR;

    private Servo clawServo;
    private double leftStickX;
    private double leftStickY;
    private double rightStickX;



    RunToPosition rtp = new RunToPosition();
    SkyStoneDriveBase skyStoneDriveBase;


    AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;
    TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

//code for lift

    public int curPos = 0;



    //claw Servo init + var

    private double clClose = 0;
    private double cl = clClose;
    private double clOpen = 1;

    // LynxDcMotorController ctrl;
    LynxModule module;

    @Override
    public void runOpMode() throws InterruptedException {


        trackingWheelIntegrator = new TrackingWheelIntegrator();

        //mapping out the robot
        FL= (DcMotorEx) hardwareMap.get(DcMotor.class, "FL");
        FR= (DcMotorEx) hardwareMap.get(DcMotor.class, "FR");
        RL= (DcMotorEx) hardwareMap.get(DcMotor.class, "RL");
        RR= (DcMotorEx) hardwareMap.get(DcMotor.class, "RR");
        rtp.liftMotor= (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotor");
        clawServo= (Servo) hardwareMap.get(Servo.class, "clawServo");
        //beambreak= (DistanceSensor) hardwareMap.get(DistanceSensor.class, "beambreak");
        skyStoneDriveBase = new SkyStoneDriveBase();

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
        rtp.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rtp.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean AutomationLastState = false;
        acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.08, 1, .05);

        telemetry.setMsTransmissionInterval(20);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {         //If button is pressed Auto aline will run. if not normally gamepad works
                if (!AutomationLastState) {
                    //clearEnc();
                    trackingWheelIntegrator.setFirstTrackingVal(0,0);
                    //buildTrajectory();
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
        } else if (gamepad1.left_stick_x < -.01) {
            leftStickX = gamepad1.left_stick_x;
        } else {
            leftStickX = 0;
        }
        if (gamepad1.left_stick_y > .01) {
            leftStickY = gamepad1.left_stick_y;
        } else if (gamepad1.left_stick_y < -.01) {
            leftStickY = gamepad1.left_stick_y;
        } else {
            leftStickY = 0;
        }
        if (gamepad1.right_stick_x > .01) {
            rightStickX = gamepad1.right_stick_x;
        } else if (gamepad1.right_stick_x < -.01) {
            rightStickX = gamepad1.right_stick_x;
        } else {
            rightStickX = 0;
        }

        telemetry.update();

        //Claw
        if (gamepad1.triangle) {
            clawServo.setPosition(-0.2);
        }
        if (gamepad1.square) {
            clawServo.setPosition(.4);
        }

        //lift positions

        //cone pickup level
        if(gamepad1.right_bumper) {

        }
        //low junction
        if(gamepad1.left_bumper) {

        }
        //medium junction
        if(gamepad1.left_trigger == 1) {

        }
        //high junction
        if(gamepad1.right_trigger == 1) {

        }
        //manual UP movement
        if (gamepad1.dpad_up) {
            if(rtp.liftMotor.getCurrentPosition() < 3000) {
                curPos = curPos + 10;
                rtp.RunToPos(curPos, .5);
            }
        }
        //manual DOWN movement
        if (gamepad1.dpad_down) {
            if(rtp.liftMotor.getCurrentPosition() > 0) {
                curPos = curPos - 10;
                rtp.RunToPos(curPos, -.5);
            }
        }

        // Edit this block to change the speed (always keep rightStickX below the others)
        MecanumDrive.cartesian(Globals.robot,
                -leftStickY * .35, // Main
                leftStickX * .35, // Strafe
                rightStickX * .30); // Turn

        if (gamepad1.left_stick_button) {
            MecanumDrive.cartesian(Globals.robot,
                    -leftStickY * .70, // Main
                    leftStickX * .70, // Strafe
                    rightStickX * .65 ); // Turn
        }

        if (gamepad1.right_trigger == 1) {
            MecanumDrive.cartesian(Globals.robot,
                    -leftStickY * .10, // Main
                    leftStickX * .10 , // Strafe
                    rightStickX * .5 ); // Turn

        }
       telemetry.addData("LiftEnc", rtp.liftMotor.getCurrentPosition());
    }

}


