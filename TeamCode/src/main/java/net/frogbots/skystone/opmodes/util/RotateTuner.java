package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;
@Disabled
@TeleOp
public class RotateTuner extends TunableLinearOpMode
{
    Servo rotate;

    @Override
    public void runOpMode() throws InterruptedException
    {
        rotate = hardwareMap.servo.get("rotate");

        waitForStart();

        while (opModeIsActive())
        {
            rotate.setPosition(getDouble("rotate"));

            sleep(20);
        }
    }
}
