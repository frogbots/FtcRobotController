//package net.frogbots.skystone.opmodes.util;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.lynx.LynxDcMotorController;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.MovingStatistics;
//import com.qualcomm.robotcore.util.Range;
//
//import net.frogbots.skystone.control.AcceleratedGain;
//import net.frogbots.skystone.control.MecanumDrive;
//import net.frogbots.skystone.control.PositionLogger;
//import net.frogbots.skystone.control.TrackingWheelIntegrator;
//import net.frogbots.skystone.drivers.FrogBNO055;
//import net.frogbots.skystone.hardware.components.drivebase.MotorPowers;
//import net.frogbots.skystone.hardware.components.drivebase.SkyStoneDriveBase;
//import net.frogbots.skystone.meta.misc.Vector;
//
//@TeleOp
//public class TrackingWheelMoveTester extends LinearOpMode
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
//    AcceleratedGain turnGain = new AcceleratedGain(.012, 0.0001);
//
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");
//        ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");
//
//        FrogBNO055 frogBNO055 = new FrogBNO055(hardwareMap.get(BNO055IMU.class, "external_IMU"));
//
//        frogBNO055.init();
//
//        FoundationGrippers grippers = new FoundationGrippers();
//
//        grippers.init(hardwareMap);
//
//        Crane crane = new Crane();
//        crane.init(hardwareMap);
//
//        crane.setExtPosNormal();
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
//        waitForStart();
//
//        //Collect first
//        moveToWaypoint(15, 25.5, 0.5, -90, new AcceleratedGain(.012, 0.0004), .45, .5, 0);
//        skyStoneDriveBase.stopMotors();
//        pollSleep(500);
//        trackingWheelIntegrator.setHeadingOffsetDegrees(frogBNO055.pollAndGetYaw());
//
//        //Bridge pass on way to build
//        moveThroughWaypoint(-5, 18, -90, 1, 5, 0);
//        moveThroughWaypoint(-20, 18, -90, 1, 5, 0);
//
//        //Deliver first
//        moveToWaypoint(-57, 26, -90, 0.5, 0);
//        skyStoneDriveBase.stopMotors();
//        pollSleep(500);
//        trackingWheelIntegrator.setHeadingOffsetDegrees(frogBNO055.pollAndGetYaw());
//
//        //Bridge pass on way to quarry
//        moveThroughWaypoint(-20, 18, -90, 1, 5, 0);
//        moveThroughWaypoint(-10, 18, -90, 1, 5, 0);
//
//        //Collect second
//        moveToWaypoint(25.5, 25, -90, 0.5, 0);
//        skyStoneDriveBase.stopMotors();
//        pollSleep(500);
//        trackingWheelIntegrator.setHeadingOffsetDegrees(frogBNO055.pollAndGetYaw());
//
//        //Bridge pass on way to build
//        moveThroughWaypoint(-5, 18, -90, 1, 5, 0);
//        moveThroughWaypoint(-20, 18, -90, 1, 5, 0);
//
//        //Deliver second
//        moveToWaypoint(-57, 26, -90, 0.5, 0);
//        skyStoneDriveBase.stopMotors();
//        pollSleep(500);
//        trackingWheelIntegrator.setHeadingOffsetDegrees(frogBNO055.pollAndGetYaw());
//
//        //Bridge pass on way to quarry
//        moveThroughWaypoint(-5, 18, -90, 1, 5, 0);
//        moveThroughWaypoint(-10, 18, -90, 1, 5, 0);
//
//        //Collect third
//        moveToWaypoint(25.5, 25, -90, 0.5, 0);
//        skyStoneDriveBase.stopMotors();
//        pollSleep(500);
//        trackingWheelIntegrator.setHeadingOffsetDegrees(frogBNO055.pollAndGetYaw());
//
//        //Bridge pass on way to build
//        moveThroughWaypoint(-5, 18, -90, 1, 5, 0);
//        moveThroughWaypoint(-20, 18, -90, 1, 5, 0);
//
//        //Deliver third
//        moveToWaypoint(-57, 26, -90, 0.5, 0);
//        skyStoneDriveBase.stopMotors();
//        pollSleep(500);
//
//        //Bridge pass on way to quarry
//        moveThroughWaypoint(-20, 18, -90, 1, 5, 0);
//        moveThroughWaypoint(-10, 18, -90, 1, 5, 0);
//
//        //Collect fourth
//        moveToWaypoint(25.5, 25, -90, 0.5, 0);
//        skyStoneDriveBase.stopMotors();
//        pollSleep(500);
//        trackingWheelIntegrator.setHeadingOffsetDegrees(frogBNO055.pollAndGetYaw());
//
//        //Bridge pass on way to build
//        moveThroughWaypoint(-5, 18, -90, 1, 5, 0);
//        moveThroughWaypoint(-20, 18, -90, 1, 5, 0);
//
//        //Deliver fourth
//        moveToWaypoint(-57, 26, -90, 0.5, 0);
//        skyStoneDriveBase.stopMotors();
//        pollSleep(500);
//        trackingWheelIntegrator.setHeadingOffsetDegrees(frogBNO055.pollAndGetYaw());
//
//        //Get in position for manuever to fnd
//        moveThroughWaypoint(-60, 18, -180, .5, 2, 0);
//
//        //Turn for fnd
//        moveToWaypoint(-60, 18, .5, -180, new AcceleratedGain(.012, 0.0008), .6, 2, 2);
//
//        //Point to grip foundation at
//        moveToWaypoint(-65.5, 31, 0.35, -180, new AcceleratedGain(.012, 0.0002), .25, 0.5, 0);
//        grippers.gripStall();
//
//        //Pull it out from the wall a bit
//        moveThroughWaypoint(-55, 18, -180, .5, 2, 0);
//
//        //Give it some space for the turn
//        moveToWaypoint(-50, 12.7, .5, -90, new AcceleratedGain(.012, .0004), .65, 1, 5);
//
//        //Position it in place
//        moveToWaypoint(-57, 12.7, -90, 1, 0);
//        grippers.release();
//
//        //Park under bridge
//        moveThroughWaypoint(-15, 12, -90, .5, 2, 0);
//
//
//        skyStoneDriveBase.stopMotors();
//
//        while (opModeIsActive())
//        {
//            updateTracking();
//        }
//
////        startLoop = System.currentTimeMillis();
////
////        while (!this.isStopRequested() && this.isStarted())
////        {
////            startLoop = System.currentTimeMillis();
////
////            LynxModule.BulkData bulkData = module.getBulkData();
////
////            int left = bulkData.getMotorCurrentPosition(0);
////            int right = bulkData.getMotorCurrentPosition(1);
////            int aux = bulkData.getMotorCurrentPosition(2);
////
////            trackingWheelIntegrator.update(left, right, aux, 0);
////            //trackingWheelIntegratorArc.update(left, right, aux, telemetry);
////
////            telemetry.addData("X", trackingWheelIntegrator.getX());
////            telemetry.addData("Y", trackingWheelIntegrator.getY());
////            //telemetry.addData("A_X", trackingWheelIntegratorArc.getX());
////            //telemetry.addData("A_Y", trackingWheelIntegratorArc.getY());
////            telemetry.addData("wheelH", trackingWheelIntegrator.getHeading());
////            telemetry.addData("L", left*-1);
////            telemetry.addData("R", right*-1);
////            telemetry.addData("Aux", aux);
////            telemetry.addData("S", trackingWheelIntegrator.speed());
////            telemetry.addData("U", 1000/movingStatistics.getMean());
////
////            telemetry.update();
////
////            move();
////
////            //positionLogger.add(trackingWheelIntegrator.getX(), trackingWheelIntegrator.getY());
////
////            long delta = System.currentTimeMillis() - startLoop;
////            movingStatistics.add(delta);
////        }
//
//        /*new Thread(new Runnable()
//        {
//            @Override
//            public void run()
//            {
//                positionLogger.dump();
//
//                AppUtil.getInstance().showToast(UILocation.BOTH, "Dump done");
//            }
//        }).start();*/
//    }
//
//    void move()
//    {
//        double turnError = trackingWheelIntegrator.getHeading() - 180;
//
//        double targX = -65.5;
//        double targY = 27.5;
//
//        double xErr = targX - trackingWheelIntegrator.getX();
//        double yErr = targY - trackingWheelIntegrator.getY();
//
//        Vector driveVector = new Vector();
//        driveVector.addCartesian(xErr, yErr);
//        driveVector.rotateDegrees(-trackingWheelIntegrator.getHeading());
//
//        driveVector.setCartesian(driveVector.getX()*1.2, driveVector.getY());
//
//        double mag = Range.clip(driveVector.getMag()*.06, -.25, .25);
//
//        double turnCorrection = Range.clip(turnError*turnGain.getControlledGain(), -.25, .25);
//
//        MotorPowers pows = MecanumDrive.calcPolar(mag, driveVector.getDir(), turnCorrection);
//
//        skyStoneDriveBase.setMotorPowers(pows);
//    }
//
//    void updateTracking()
//    {
//        LynxModule.BulkData bulkData = module.getBulkData();
//
//        int left = bulkData.getMotorCurrentPosition(0);
//        int right = bulkData.getMotorCurrentPosition(1);
//        int aux = bulkData.getMotorCurrentPosition(2);
//
//        trackingWheelIntegrator.update(left, right, aux);
//
//        telemetry.addData("X", trackingWheelIntegrator.getX());
//        telemetry.addData("Y", trackingWheelIntegrator.getY());
//        telemetry.addData("wheelH", trackingWheelIntegrator.getHeading());
//        telemetry.update();
//    }
//
//    void moveToWaypoint(double targetx, double targety, double targetHeading, double thresh, double angleThresh)
//    {
//        moveToWaypoint(targetx, targety, 0.5, targetHeading, new AcceleratedGain(.012, 0.0002), .25, thresh, angleThresh);
//    }
//
//    void moveToWaypoint(double targetx, double targety, double maxPow, double targetHeading, AcceleratedGain headingGain, double maxTurnPower, double thresh, double angleThresh)
//    {
//        double error = 5;
//
//        boolean angleGood = false;
//
//        while ((Math.abs(error) > thresh || !angleGood) && opModeIsActive())
//        {
//            updateTracking();
//
//            double turnError = trackingWheelIntegrator.getHeading() - targetHeading;
//
//            double xErr = targetx - trackingWheelIntegrator.getX();
//            double yErr = targety - trackingWheelIntegrator.getY();
//
//            if(angleThresh == 0)
//            {
//                angleGood = true;
//            }
//            else
//            {
//                angleGood = Math.abs(turnError) < angleThresh;
//            }
//
//            Vector driveVector = new Vector();
//            driveVector.addCartesian(xErr, yErr);
//            driveVector.rotateDegrees(-trackingWheelIntegrator.getHeading());
//
//            driveVector.setCartesian(driveVector.getX()*1.5, driveVector.getY());
//
//            double mag = Range.clip(driveVector.getMag()*.06, -maxPow, maxPow);
//
//            double turnCorrection = Range.clip(turnError*headingGain.getControlledGain(), -maxTurnPower, maxTurnPower);
//
//            MotorPowers pows = MecanumDrive.calcPolar(mag, driveVector.getDir(), turnCorrection);
//
//            error = Vector.calcMag(xErr, yErr);
//
//            skyStoneDriveBase.setMotorPowers(pows);
//        }
//    }
//
//    void pollSleep(long ms)
//    {
//        long start = System.currentTimeMillis();
//
//        while (System.currentTimeMillis() - start < ms)
//        {
//            updateTracking();
//        }
//    }
//
//    void moveThroughWaypoint(double targetx, double targety, double targetHeading, double speed, double thresh, double angleThresh)
//    {
//        AcceleratedGain headingGain = new AcceleratedGain(.012, 0.0002);
//
//        double error = 5;
//
//        boolean angleGood = false;
//
//        while ((Math.abs(error) > thresh || !angleGood) && opModeIsActive())
//        {
//            updateTracking();
//
//            double turnError = trackingWheelIntegrator.getHeading() - targetHeading;
//            double xErr = targetx - trackingWheelIntegrator.getX();
//            double yErr = targety - trackingWheelIntegrator.getY();
//
//            if(angleThresh == 0)
//            {
//                angleGood = true;
//            }
//            else
//            {
//                angleGood = Math.abs(turnError) < angleThresh;
//            }
//
//            Vector driveVector = new Vector();
//            driveVector.addCartesian(xErr, yErr);
//            driveVector.rotateDegrees(-trackingWheelIntegrator.getHeading());
//
//            driveVector.setCartesian(driveVector.getX()*1.5, driveVector.getY());
//
//            double turnCorrection = Range.clip(turnError*headingGain.getControlledGain(), -.25, .25);
//
//            MotorPowers pows = MecanumDrive.calcPolar(speed, driveVector.getDir(), turnCorrection);
//
//            error = Vector.calcMag(xErr, yErr);
//
//            skyStoneDriveBase.setMotorPowers(pows);
//        }
//    }
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
//}
