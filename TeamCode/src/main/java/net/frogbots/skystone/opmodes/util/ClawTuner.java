package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

@TeleOp
public class ClawTuner extends TunableLinearOpMode
{
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException
    {
        claw = hardwareMap.servo.get("claw");

        waitForStart();

        while (opModeIsActive())
        {
            claw.setPosition(getDouble("claw"));

            sleep(20);
        }
    }
}
