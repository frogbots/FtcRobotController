//package net.frogbots.skystone.opmodes.tele;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import net.frogbots.skystone.control.AccelerationControlledDrivetrain;
//import net.frogbots.skystone.control.MecanumDrive;
//import net.frogbots.skystone.hardware.components.drivebase.SkyStoneDriveBase;
//
//import net.frogbots.skystone.control.MecanumPositionTracker;
//
//@TeleOp
//public class PosTrackingTele extends LinearOpMode
//{
//    SkyStoneDriveBase skyStoneDriveBase = new SkyStoneDriveBase();
//    MecanumPositionTracker mecanumPositionTracker = new MecanumPositionTracker();
//
//    AccelerationControlledDrivetrain accelerationControlledDrivetrain;
//
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        skyStoneDriveBase.init(hardwareMap);
//        skyStoneDriveBase.enablePID();
//
//        accelerationControlledDrivetrain = new AccelerationControlledDrivetrain(skyStoneDriveBase, .04, .04, 0);
//
//        ExpansionHubEx hubEx = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
//
//        int fl, fr, rl, rr;
//
//        skyStoneDriveBase.resetEncoders();
//        skyStoneDriveBase.enableBrake(false);
//
//        waitForStart();
//
//        telemetry.setMsTransmissionInterval(20);
//
//
//        while (opModeIsActive())
//        {
//            MecanumDrive.cartesian(accelerationControlledDrivetrain,
//                    -gamepad1.left_stick_y, //Main
//                    gamepad1.left_stick_x, //Strafe
//                    gamepad1.right_stick_x*.75); //Turn - we'd don't need to negate the turning
//
//            RevBulkData bulkData = hubEx.getBulkInputData();
//
//            fl = bulkData.getMotorCurrentPosition(skyStoneDriveBase.frontLeft);
//            fr = bulkData.getMotorCurrentPosition(skyStoneDriveBase.frontRight);
//            rl = bulkData.getMotorCurrentPosition(skyStoneDriveBase.rearLeft);
//            rr = bulkData.getMotorCurrentPosition(skyStoneDriveBase.rearRight);
//
//            mecanumPositionTracker.update(fl, fr, rl, rr);
//
//            telemetry.addData("X", mecanumPositionTracker.getX());
//            telemetry.addData("Y", mecanumPositionTracker.getY());
//            telemetry.addData("Heading", mecanumPositionTracker.getHeading());
//            telemetry.addData("fl", fl);
//            telemetry.addData("fr", fr);
//            telemetry.addData("rl", rl);
//            telemetry.addData("rr", rr);
//            telemetry.update();
//        }
//    }
//
//    double turn(double current, double target, double Kp)
//    {
//        double err = target - current;
//
//        skyStoneDriveBase.setMotorPowers(-err*Kp, err*Kp);
//
//        return err;
//    }
//}
