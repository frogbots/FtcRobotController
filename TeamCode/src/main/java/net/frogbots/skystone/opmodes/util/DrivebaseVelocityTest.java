package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.frogbots.skystone.meta.opmode.FrogOpMode;
@Disabled
@TeleOp
public class DrivebaseVelocityTest extends FrogOpMode
{
    @Override
    protected void frog_run()
    {
        while (opModeIsActive())
        {
            robot.driveTrain.setMotorPowers(.1, 0,0,0);
        }
    }

    @Override
    protected void frog_init()
    {
        robot.driveTrain.enablePID();
    }
}
