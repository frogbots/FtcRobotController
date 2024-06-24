package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;
@Disabled
@TeleOp
public class FoundationGripTuner extends TunableLinearOpMode
{
    Servo leftGripper;
    Servo rightGripper;

    double LG_DOWN = 0.8357;
    double RG_DOWN = 0.2907;

    double LG_UP = 0.6892;
    double RG_UP = 0.4508;

    @Override
    public void runOpMode()
    {
        leftGripper = hardwareMap.servo.get("leftGripper");
        rightGripper = hardwareMap.servo.get("rightGripper");

        waitForStart();

        while (opModeIsActive())
        {
            double lg = getDouble("lg");
            double rg = getDouble("rg");

            if(lg != 0)
            {
                leftGripper.setPosition(lg);
            }

            if(rg != 0)
            {
                rightGripper.setPosition(rg);
            }

            sleep(20);
        }
    }
}
