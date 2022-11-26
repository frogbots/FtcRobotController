//package net.frogbots.skystone.opmodes.util;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;
//
//@TeleOp
//public class LiftTest extends LinearOpMode
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
//        liftMotor.setTargetPosition(0);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (opModeIsActive())
//        {
//            if(gamepad1.dpad_up)
//            {
//                liftMotor.setPower(1);
//                liftMotor.setTargetPosition(5000);
//            }
//            else if(gamepad1.dpad_down)
//            {
//                liftMotor.setPower(1);
//                liftMotor.setTargetPosition(0);
//            }
//
//            telemetry.addData("Encoder", liftMotor.getCurrentPosition());
//            telemetry.addData("Current amps", liftMotor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//            telemetry.update();
//        }
//    }
//}
