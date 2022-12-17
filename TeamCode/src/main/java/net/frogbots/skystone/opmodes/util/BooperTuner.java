package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;
@Disabled
@TeleOp
public class BooperTuner extends TunableLinearOpMode
{
    Servo booper;

    @Override
    public void runOpMode() throws InterruptedException
    {
        booper = hardwareMap.servo.get("booper");

        waitForStart();

        while (opModeIsActive())
        {
            booper.setPosition(getDouble("booper"));
            sleep(20);
        }
    }
}
