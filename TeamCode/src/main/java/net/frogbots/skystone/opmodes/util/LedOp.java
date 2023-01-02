package net.frogbots.skystone.opmodes.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by michael on 8/6/18.
 */

@TeleOp(name = "Michael LED test")
public class LedOp extends LinearOpMode
{
    ColorWheel colorWheel = new ColorWheel();
    DotstarStrip dotstarStrip;

    @Override
    public void runOpMode()
    {
        LynxModule module = hardwareMap.get(LynxModule.class, "Expansion Hub 2");

        LynxI2cConfigureChannelCommand cmd = new LynxI2cConfigureChannelCommand(module, 1, LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K);
        try {
            cmd.sendReceive();
        } catch (Exception e) {
            e.printStackTrace();
        }

        /*
         * NOTE: You MUST set the correct I2C mode, or else this will NOT work! In order to use
         * REV_SPEEDHACK_FOR_400KHz, you need to use REV Extensions 2 to set the speed of the I2C
         * port to which the bridge is attached to 400KHz. By using the REV_SPEEDHACK_FOR_400KHz
         * mode, you can increase your FPS by more than 2x!
         */
        dotstarStrip = new DotstarStrip("bridge", 30, hardwareMap, SC18IS602B.I2cMode.REV_SPEEDHACK_FOR_400KHz);
        dotstarStrip.init();

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        long startT = 0;
        long deltaT = 0;

        while (opModeIsActive())
        {
            startT = System.currentTimeMillis();

            for(DotstarPixel pixel : dotstarStrip.getAllPixels())
            {
                pixel.queueHue(colorWheel.next(10), 1f);
            }

            colorWheel.incrementStartPositionAndReset(2.5f);

            dotstarStrip.apply();

            deltaT = System.currentTimeMillis() - startT;

            telemetry.addData("Loop", deltaT);
            telemetry.addData("FPS", 1000/deltaT);
            telemetry.update();
        }

        try {
            Thread.sleep(25);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        for(DotstarPixel pixel : dotstarStrip.getAllPixels())
        {
            pixel.queueRgb(0,0,0);
        }
        dotstarStrip.apply();
        try {
            Thread.sleep(25);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
