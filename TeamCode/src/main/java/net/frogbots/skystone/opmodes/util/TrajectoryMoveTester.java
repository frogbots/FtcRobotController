//package net.frogbots.skystone.opmodes.util;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.lynx.LynxDcMotorController;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.MovingStatistics;
//
//import net.frogbots.skystone.control.AcceleratedGain;
//import net.frogbots.skystone.control.PositionLogger;
//import net.frogbots.skystone.control.TrackingWheelIntegrator;
//import net.frogbots.skystone.drivers.FrogBNO055;
//import net.frogbots.skystone.hardware.components.drivebase.SkyStoneDriveBase;
//import net.frogbots.skystone.opmodes.auto.Globals;
//import net.frogbots.skystone.opmodes.util.trajectory.GripperDeployAction;
//import net.frogbots.skystone.opmodes.util.trajectory.GripperReleaseAction;
//import net.frogbots.skystone.opmodes.util.trajectory.PrepClawForUnderBridgeAction;
//import net.frogbots.skystone.opmodes.util.trajectory.PrepForNextGrabAction;
//import net.frogbots.skystone.opmodes.util.trajectory.SleepAction;
//import net.frogbots.skystone.opmodes.util.trajectory.StoneGrabAction;
//import net.frogbots.skystone.opmodes.util.trajectory.PointApproach;
//import net.frogbots.skystone.opmodes.util.trajectory.StoneReleaseAction;
//import net.frogbots.skystone.opmodes.util.trajectory.Trajectory;
//import net.frogbots.skystone.opmodes.util.trajectory.Waypoint;
//
//@TeleOp
//public class TrajectoryMoveTester extends LinearOpMode
//{
//    MovingStatistics movingStatistics = new MovingStatistics(300);
//
//    long startLoop = 0;
//
//    TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();
//
//    PositionLogger positionLogger = new PositionLogger();
//
//    LynxDcMotorController ctrl;
//    LynxModule module;
//
//    SkyStoneDriveBase skyStoneDriveBase;
//
//    Trajectory trajectory;
//
//    AutoClaw autoClaw;
//
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//
//        trackingWheelIntegrator = new TrackingWheelIntegrator();
//
//        module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");
//        ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");
//
//        FrogBNO055 frogBNO055 = new FrogBNO055(hardwareMap.get(BNO055IMU.class, "external_IMU"));
//
//        //frogBNO055.init();
//
//        telemetry.addLine("IMU init done");
//        telemetry.update();
//
//        FoundationGrippers grippers = new FoundationGrippers();
//        grippers.init(hardwareMap);
//
//        Globals.foundationGrippers = grippers;
//
//        Crane crane = new Crane();
//        crane.init(hardwareMap);
//        crane.setExtPosNormal();
//
//        AutoClaw autoClaw = new AutoClaw();
//        autoClaw.init(hardwareMap);
//        autoClaw.pivot.setPosition(AutoClaw.PIVOT_UP);
//        Globals.autoClaw = autoClaw;
//
//        skyStoneDriveBase = new SkyStoneDriveBase();
//        skyStoneDriveBase.init(hardwareMap);
//        skyStoneDriveBase.resetEncoders();
//        skyStoneDriveBase.enableBrake(true);
//        skyStoneDriveBase.enablePID();
//
//        clearEnc();
//
//        telemetry.setMsTransmissionInterval(50);
//
//        Globals.driveBase = skyStoneDriveBase;
//        Globals.opMode = this;
//        Globals.trackingWheelIntegrator = trackingWheelIntegrator;
//        Globals.odoModule = module;
//
//        buildTrajectory();
//
//        waitForStart();
//
//        autoClaw.pinch.setPosition(AutoClaw.PINCH_OPEN);
//        autoClaw.pivot.setPosition(AutoClaw.PIVOT_DOWN);
//
//        trajectory.follow();
//    }
//
//
//    void clearEnc()
//    {
//        ctrl.setMotorMode(0, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ctrl.setMotorMode(1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ctrl.setMotorMode(2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        ctrl.setMotorMode(0, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ctrl.setMotorMode(1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ctrl.setMotorMode(2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    public void buildTrajectory()
//    {
//        trajectory = new Trajectory.Builder()
//
//                /*
//                 * Collect first
//                 */
//                .addMovement(new PointApproach.Builder()
//                        .setTargetPosition(18,23.7)
//                        .setMaxPower(0.35)
//                        .setXyGain(.04)
//                        .setTargetHeading(90)
//                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
//                        .setMaxTurnPower(0.45)
//                        .setMovementThresh(1)
//                        .setHeadingThreshold(0)
//                        .stopMotorsOnDone(true)
//                        .build())
//                .addMovement(new SleepAction(500))
//                .addMovement(new StoneGrabAction())
//
//
//                /*
//                 * Deliver first
//                 */
//                .addMovement(new Waypoint.Builder() // Waypoint 1 around bridge
//                        .setTargetPosition(-5, 18)
//                        .setTargetHeading(90)
//                        .setSpeed(.5)
//                        .setTransThreshMethod(Waypoint.TranslationThreshMethod.Y_ONLY)
//                        .setMovementThresh(2)
//                        .setHeadingThreshold(0)
//                        .build())
//                .addMovement(new Waypoint.Builder() //Waypoint 2 around bridge
//                        .setTargetPosition(-20, 18)
//                        .setTargetHeading(90)
//                        .setSpeed(.65)
//                        .setTransThreshMethod(Waypoint.TranslationThreshMethod.X_ONLY)
//                        .setMovementThresh(2)
//                        .setHeadingThreshold(0)
//                        .build())
//                .addMovement(new PointApproach.Builder() //Final alignment
//                        .setTargetPosition(-65,28)
//                        .setTargetHeading(90)
//                        .setMovementThresh(1)
//                        .setHeadingThreshold(0)
//                        .stopMotorsOnDone(true)
//                        .build())
//                .addMovement(new StoneReleaseAction())
//                .addMovement(new PrepClawForUnderBridgeAction())
//
//                /*
//                 * Collect second
//                 */
//                .addMovement(new Waypoint.Builder() // Waypoint 1 around bridge
//                        .setTargetPosition(-20, 18)
//                        .setTargetHeading(90)
//                        .setSpeed(.5)
//                        .setTransThreshMethod(Waypoint.TranslationThreshMethod.Y_ONLY)
//                        .setMovementThresh(2)
//                        .setHeadingThreshold(0)
//                        .build())
//                .addMovement(new Waypoint.Builder() //Waypoint 2 around bridge
//                        .setTargetPosition(-10, 18)
//                        .setTargetHeading(90)
//                        .setSpeed(.65)
//                        .setTransThreshMethod(Waypoint.TranslationThreshMethod.X_ONLY)
//                        .setMovementThresh(2)
//                        .setHeadingThreshold(0)
//                        .build())
//                .addMovement(new PrepForNextGrabAction())
//                .addMovement(new PointApproach.Builder() //Final alignment
//                        .setTargetPosition(42,22.5)
//                        .setTargetHeading(90)
//                        .setMovementThresh(1)
//                        .setHeadingThreshold(0)
//                        .stopMotorsOnDone(true)
//                        .build())
//                .addMovement(new SleepAction(500))
//                .addMovement(new StoneGrabAction())
//
//                /*
//                 * Deliver Second
//                 */
//                .addMovement(new Waypoint.Builder() // Waypoint 1 around bridge
//                        .setTargetPosition(-5, 18)
//                        .setTargetHeading(90)
//                        .setSpeed(.5)
//                        .setTransThreshMethod(Waypoint.TranslationThreshMethod.Y_ONLY)
//                        .setMovementThresh(2)
//                        .setHeadingThreshold(0)
//                        .build())
//                .addMovement(new Waypoint.Builder() //Waypoint 2 around bridge
//                        .setTargetPosition(-20, 18)
//                        .setTargetHeading(90)
//                        .setSpeed(.65)
//                        .setTransThreshMethod(Waypoint.TranslationThreshMethod.X_ONLY)
//                        .setMovementThresh(2)
//                        .setHeadingThreshold(0)
//                        .build())
//                .addMovement(new PointApproach.Builder() //Final alignment
//                        .setTargetPosition(-57,28)
//                        .setTargetHeading(90)
//                        .setMovementThresh(1)
//                        .setHeadingThreshold(0)
//                        .stopMotorsOnDone(true)
//                        .build())
//                .addMovement(new StoneReleaseAction())
//
//                /*
//                 * Foundation
//                 */
//                .addMovement(new Waypoint.Builder() // Get in position for manuever to fnd
//                        .setTargetPosition(-60, 18)
//                        .setTargetHeading(180)
//                        .setSpeed(0.5)
//                        .setMovementThresh(2)
//                        .setHeadingThreshold(0)
//                        .build())
//                .addMovement(new PointApproach.Builder() //Turn for fnd
//                        .setTargetPosition(-60, 18)
//                        .setTargetHeading(180)
//                        .setMaxPower(.5)
//                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0008))
//                        .setMaxTurnPower(0.6)
//                        .setHeadingThreshold(2)
//                        .setMovementThresh(2)
//                        .build())
//                .addMovement(new PointApproach.Builder() //Point to grip foundation at
//                        .setTargetPosition(-64, 34)
//                        .setTargetHeading(180)
//                        .setMaxPower(.35)
//                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0008))
//                        .setMaxTurnPower(0.25)
//                        .setHeadingThreshold(0)
//                        .setMovementThresh(1)
//                        .stopMotorsOnDone(true)
//                        .build())
//                .addMovement(new GripperDeployAction())
//                .addMovement(new SleepAction(250))
//                .addMovement(new Waypoint.Builder() //Pull it out from the wall a bit
//                        .setTargetPosition(-55, 18)
//                        .setTargetHeading(180)
//                        .setSpeed(0.5)
//                        .setMovementThresh(2)
//                        .setHeadingThreshold(0)
//                        .build())
//                .addMovement(new PointApproach.Builder() //Give it some space for the turn
//                        .setTargetPosition(-50, 12.7)
//                        .setTargetHeading(270)
//                        .setMaxPower(.5)
//                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0008))
//                        .setMaxTurnPower(0.65)
//                        .setHeadingThreshold(5)
//                        .setMovementThresh(1)
//                        .build())
//                .addMovement(new PointApproach.Builder() //Position it in place
//                        .setTargetPosition(-57, 12.7)
//                        .setTargetHeading(270)
//                        .setMaxPower(.5)
//                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0008))
//                        .setMaxTurnPower(0.65)
//                        .setHeadingThreshold(0)
//                        .setMovementThresh(1)
//                        .build())
//                .addMovement(new GripperReleaseAction())
//
//                /*
//                 * Park under bridge
//                 */
//                .addMovement(new PointApproach.Builder() //Position it in place
//                        .setTargetPosition(-15, 12)
//                        .setTargetHeading(270)
//                        .setMaxPower(.5)
//                        .setMaxTurnPower(0.65)
//                        .setHeadingThreshold(0)
//                        .setMovementThresh(2)
//                        .build())
//
//
//                .build();
//    }
//}
