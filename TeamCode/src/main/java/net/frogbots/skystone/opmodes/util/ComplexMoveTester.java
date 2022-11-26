//package net.frogbots.skystone.opmodes.util;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import net.frogbots.skystone.control.AccerlationControlledDrivetrainPowerGeneratorForAuto;
//import net.frogbots.skystone.control.MecanumDrive;
//import net.frogbots.skystone.control.MecanumPositionTracker;
//import net.frogbots.skystone.drivers.FrogBNO055;
//import net.frogbots.skystone.hardware.components.drivebase.MotorPowers;
//import net.frogbots.skystone.hardware.components.drivebase.SkyStoneDriveBase;
//import net.frogbots.skystone.meta.misc.Vector;
//import net.frogbots.skystone.meta.opmode.FrogOpMode;
//
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.RevBulkData;
//
//@TeleOp
//public class ComplexMoveTester extends LinearOpMode
//{
//    MecanumPositionTracker tracker;
//    SkyStoneDriveBase skyStoneDriveBase;
//    ExpansionHubEx hubEx;
//    int fl, fr, rl, rr;
//
//    private AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;
//
//    @Override
//    public void runOpMode()
//    {
//        acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.03, 1, .05);
//
//        ExpansionHubEx hubEx = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
//
//        tracker = new MecanumPositionTracker();
//        skyStoneDriveBase = new SkyStoneDriveBase();
//        skyStoneDriveBase.init(hardwareMap);
//        skyStoneDriveBase.resetEncoders();
//        skyStoneDriveBase.enableBrake(true);
//        skyStoneDriveBase.enablePID();
//        telemetry.setMsTransmissionInterval(20);
//
//        FrogBNO055 frogBNO055 = new FrogBNO055(hardwareMap.get(BNO055IMU.class, "external_IMU"));
//
//        frogBNO055.init();
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            double imuHeading = frogBNO055.pollAndGetYaw();
//
//            System.out.println("Imu heading: " + imuHeading);
//
//            RevBulkData bulkData = hubEx.getBulkInputData();
//
//            fl = bulkData.getMotorCurrentPosition(skyStoneDriveBase.frontLeft);
//            fr = bulkData.getMotorCurrentPosition(skyStoneDriveBase.frontRight);
//            rl = bulkData.getMotorCurrentPosition(skyStoneDriveBase.rearLeft);
//            rr = bulkData.getMotorCurrentPosition(skyStoneDriveBase.rearRight);
//
//            tracker.update(fl, fr, rl, rr);
//
//            //double turnErr = tracker.getHeading() - 0;
//            double turnErr = imuHeading - -35;
//
//            double xErr = tracker.getX() - -27;
//            double yErr = tracker.getY() - 32.5;
//
//            Vector driveVector = new Vector();
//            driveVector.addCartesian(xErr*-.06, yErr*-.06);
//
//            MotorPowers rawPowers = MecanumDrive.calcPolar(driveVector.getMag(), driveVector.getDir()-imuHeading, turnErr*.012);
//
//            MotorPowers powers = acclCtrl.getAccelerationControlledPowers(rawPowers);
//
//            skyStoneDriveBase.setMotorPowers(powers);
//
//            /*telemetry.addData("X", tracker.getX());
//            telemetry.addData("Y", tracker.getY());
//            telemetry.addData("Heading", imuHeading);
//            telemetry.addData("fl", fl);
//            telemetry.addData("fr", fr);
//            telemetry.addData("rl", rl);
//            telemetry.addData("rr", rr);*/
//
//            System.out.println("X: " + tracker.getX() + " Y: " + tracker.getY());
//
//
//            telemetry.update();
//        }
//    }
//}
