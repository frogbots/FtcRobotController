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
