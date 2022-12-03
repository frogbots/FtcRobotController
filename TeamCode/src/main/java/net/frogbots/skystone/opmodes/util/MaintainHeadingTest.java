package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.frogbots.skystone.control.AccelerationControlledDrivetrain;
import net.frogbots.skystone.control.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import net.frogbots.skystone.meta.opmode.FrogOpMode;

@TeleOp
public class MaintainHeadingTest extends FrogOpMode
{

    AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;

    @Override
    protected void frog_run()
    {
        while (opModeIsActive())
        {
            double err = gyroUtils.gyroStrafe(acclCtrl, -.2, 0, .013);
            //double err = gyroUtils.gyroStraight(acclCtrl, .2, 0, .017);
            //double err = gyroUtils.gyroRotate(acclCtrl, 180, .01);

            telemetry.addData("Err", err);
            telemetry.update();
        }
    }

    @Override
    protected void frog_init()
    {
        acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.08, 1, .05);

        robot.driveTrain.enablePID();
        robot.sensors.imu.init();

        telemetry.setMsTransmissionInterval(50);
    }
}
