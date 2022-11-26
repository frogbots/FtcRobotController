package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

@TeleOp
public class CapstoneTuner extends TunableLinearOpMode
{
    Servo capstone;

    @Override
    public void runOpMode() throws InterruptedException
    {
        capstone = hardwareMap.servo.get("capstone");

        waitForStart();

        while (opModeIsActive())
        {
            capstone.setPosition(getDouble("capstone"));
        }
    }
}
