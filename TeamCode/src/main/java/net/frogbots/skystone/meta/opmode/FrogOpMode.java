package net.frogbots.skystone.meta.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import net.frogbots.skystone.control.GyroUtils;
import net.frogbots.skystone.hardware.Robot;
import net.frogbots.skystone.opmodes.auto.Globals;

/**
 * Created by michael on 10/2/18.
 */

public abstract class FrogOpMode extends LinearOpMode
{
    public Robot robot;
    public GyroUtils gyroUtils;

    public static final boolean AUTO_TESTING = true;

    @Override
    public final void runOpMode() throws InterruptedException
    {
        Globals.opMode = this;

        telemetry.setMsTransmissionInterval(50);
        robot = new Robot();

        if(AUTO_TESTING)
        {
            robot.init(hardwareMap, true);
        }
        else
        {
            while (true)
            {
                telemetry.addLine("Reset Encoders? (Y/B)");
                telemetry.update();

                sleep(10);

                if(gamepad2.y)
                {
                    telemetry.addLine("RESET");
                    telemetry.update();

                    robot.init(hardwareMap, true);

                    break;
                }
                else if(gamepad2.b)
                {
                    telemetry.addLine("NOT RESET");
                    telemetry.update();

                    robot.init(hardwareMap, false);

                    break;
                }

                if(isStopRequested() || isStarted())
                {
                    return;
                }
            }
        }

        gyroUtils = new GyroUtils(robot.sensors.imu, robot.driveTrain);

        frog_init();

        waitForStart();

        frog_run();
    }

    protected abstract void frog_run();

    protected abstract void frog_init();
}
