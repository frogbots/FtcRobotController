package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

@TeleOp
public class ExtTuner extends TunableLinearOpMode
{
    Servo ext;

    @Override
    public void runOpMode() throws InterruptedException
    {
        ext = hardwareMap.servo.get("ext");

        waitForStart();

        while (opModeIsActive())
        {
            ext.setPosition(getDouble("ext"));

            sleep(20);
        }
    }
}
