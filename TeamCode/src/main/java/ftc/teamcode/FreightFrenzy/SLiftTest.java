
package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import net.frogbots.skystone.hardware.components.drivebase.SkyStoneDriveBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class SLiftTest extends LinearOpMode {

    private DcMotorEx liftMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        RunToPosition RTP = new RunToPosition();
        SkyStoneDriveBase base = new SkyStoneDriveBase();

        RTP.liftMotor= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "liftMotor");


        RTP.liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(20);
        waitForStart();



        while (opModeIsActive()) {

            telemetry.addData("liftmotor", RTP.liftMotor.getCurrentPosition());
            telemetry.update();

            System.out.println("Encoder val = " + RTP.liftMotor.getCurrentPosition());

            if (gamepad1.triangle) {
                RTP.RunToPos(2000, 1);

            }

            }



    }

}
=======
package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SLiftTest extends LinearOpMode {

    private DcMotorEx Motor;

    @Override
    public void runOpMode() throws InterruptedException {


        Motor= (DcMotorEx) hardwareMap.get(DcMotor.class, "Motor");

        telemetry.setMsTransmissionInterval(20);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("motor", Motor.getCurrentPosition());

            if (gamepad1.triangle) {
                Motor.setTargetPosition(1500);
            }
            if (gamepad1.circle) {
                Motor.setTargetPosition(100);
            }
            if (Motor.getCurrentPosition() < 2000) {
                if (gamepad1.dpad_down) {
                    Motor.setPower(-.3);
                }
                if (gamepad1.dpad_up) {
                    Motor.setPower(.4);

                } if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                    Motor.setPower(.1);
                }
            }
            else {
                Motor.setPower(0);
            }
            telemetry.update();
        }

    }

}

