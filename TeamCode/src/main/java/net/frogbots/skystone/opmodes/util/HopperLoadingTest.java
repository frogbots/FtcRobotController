package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class HopperLoadingTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        AnalogInput hopperLoading = hardwareMap.analogInput.get("hopperLoadingSensor");

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Hopper loading sensor voltage", hopperLoading.getVoltage());
            telemetry.update();
        }
    }
}
