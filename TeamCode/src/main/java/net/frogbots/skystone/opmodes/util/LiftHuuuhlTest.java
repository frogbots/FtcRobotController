//package net.frogbots.skystone.opmodes.util;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;
//
//@TeleOp
//public class LiftHuuuhlTest extends LinearOpMode
//{
//    ExpansionHubMotor liftMotor;
//
//    @Override
//    public void runOpMode()
//    {
//        liftMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("lift");
//
//        waitForStart();
//
//        telemetry.setMsTransmissionInterval(20);
//
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        liftMotor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(10, 3, 0));
//        liftMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(10, .05, 0));
//
//        liftMotor.setTargetPosition(0);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        liftMotor.setPower(1);
//
//        while (opModeIsActive())
//        {
//            liftMotor.setTargetPosition(435);
//
//            telemetry.addData("Encoder", liftMotor.getCurrentPosition());
//            telemetry.addData("Current amps", liftMotor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//            telemetry.update();
//        }
//    }
//}
