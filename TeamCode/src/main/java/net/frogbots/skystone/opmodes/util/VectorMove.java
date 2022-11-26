package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.frogbots.skystone.control.MecanumDrive;
import net.frogbots.skystone.meta.opmode.FrogOpMode;

@TeleOp
public class VectorMove extends FrogOpMode
{
    @Override
    protected void frog_run()
    {
        while (opModeIsActive())
        {
            MecanumDrive.polar(robot.driveTrain, .2, 225 ,0);
        }
    }

    @Override
    protected void frog_init()
    {
        robot.driveTrain.enablePID();
    }
}
