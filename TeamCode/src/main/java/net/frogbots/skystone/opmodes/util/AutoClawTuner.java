package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;
@Disabled
@TeleOp
public class AutoClawTuner extends TunableLinearOpMode
{
    Servo pivot;
    Servo pinch;

    @Override
    public void runOpMode() throws InterruptedException
    {
        pivot = hardwareMap.servo.get("autoClawPivot");
        pinch = hardwareMap.servo.get("autoClawPinch");

        waitForStart();

        while (opModeIsActive())
        {
            pivot.setPosition(getDouble("pivot"));
            pinch.setPosition(getDouble("pinch"));

            sleep(20);
        }
    }
}
