package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.frogbots.skystone.meta.misc.Vector;
@Disabled
@TeleOp
public class VectorTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        Vector vector = new Vector();

        while (opModeIsActive())
        {
            vector.clear();
            vector.addCartesian(gamepad2.left_stick_x, -gamepad2.left_stick_y);

            if(vector.getMag() > .5)
            {
                double dir = vector.getDir();

                boolean zeroTo45 = dir < 45.0 && dir > 0.0;
                boolean zeroTo315 = dir < 360.0 && dir > 315.0;
                boolean exact = (dir == 0.0) | (dir == 360.0);
                boolean right = zeroTo45 | zeroTo315 | exact;

                boolean fortyFiveTo135 = dir > 45.0 && dir < 135.0;
                boolean up = fortyFiveTo135;

                boolean oneThirtyFiveTo225 = dir > 135.0 && dir < 225.0;
                boolean left = oneThirtyFiveTo225;

                boolean twoTwentyFiveTo315 = dir > 225.0 && dir < 315.0;
                boolean down = twoTwentyFiveTo315;

                if(right)
                {
                    telemetry.addData("Dir", "Right");
                }
                else if(up)
                {
                    telemetry.addData("Dir", "Up");
                }
                else if(left)
                {
                    telemetry.addData("Dir", "Left");
                }
                else if(down)
                {
                    telemetry.addData("Dir", "Down");
                }
            }



            telemetry.update();
        }
    }
}
