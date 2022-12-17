//package net.frogbots.skystone.opmodes.util;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;
//import net.frogbots.skystone.meta.misc.Toggler;
//
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;
//
//@TeleOp
//public class LiftTuner extends TunableLinearOpMode
//{
//    ExpansionHubMotor liftMotor;
//    public double delta = 0;
//
//    Toggler deltaUpToggler = new Toggler();
//    Toggler deltaDownToggler = new Toggler();
//    VoltageSensor voltageSensor;
//
//    boolean isRtp = true;
//
//    public int liftPosition = 0;
//
//    @Override
//    public void runOpMode()
//    {
//        liftMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("lift");
//        voltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");
//
//        telemetry.setMsTransmissionInterval(20);
//
//        waitForStart();
//
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        liftMotor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(10, 3, 0));
//        liftMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(10, .05, 0));
//
//        liftMotor.setTargetPosition(0);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (opModeIsActive())
//        {
//            delta = getInt("delta");
//
//            if(deltaUpToggler.shouldToggle(gamepad1.dpad_up))
//            {
//                liftPosition += delta;
//
//                if(!isRtp)
//                {
//                    isRtp = true;
//                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//            }
//            else if(deltaDownToggler.shouldToggle(gamepad1.dpad_down))
//            {
//                liftPosition -= delta;
//
//                if(!isRtp)
//                {
//                    isRtp = true;
//                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//            }
//
//            if(gamepad1.x)
//            {
//                liftPosition = getInt("pos");
//
//                if(!isRtp)
//                {
//                    isRtp = true;
//                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//            }
//            else if(gamepad1.y)
//            {
//                liftPosition = -10;
//
//                if(!isRtp)
//                {
//                    isRtp = true;
//                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//            }
//
//            liftMotor.setTargetPosition(liftPosition);
//
//            if(gamepad1.right_bumper)
//            {
//                isRtp = false;
//
//                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                liftMotor.setPower(0);
//            }
//
//            if(!isRtp)
//            {
//                if(liftMotor.getCurrentPosition() < 150)
//                {
//                    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                }
//            }
//            else
//            {
//                liftMotor.setPower(1);
//            }
//
//            telemetry.addData("Voltage", voltageSensor.getVoltage());
//            telemetry.addData("Encoder", liftMotor.getCurrentPosition());
//            telemetry.addData("Current amps", liftMotor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//            telemetry.update();
//        }
//    }
//}
