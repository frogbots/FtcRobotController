package net.frogbots.skystone.control;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by michael on 11/12/18.
 */

public class EncoderLimitedMotorControl
{
    public static void process(DcMotor motor, double pow, double minEnc, double maxEnc, boolean override)
    {
        if(override)
        {
            motor.setPower(pow);
        }
        else
        {
            if(motor.getCurrentPosition() < minEnc)
            {
                if(pow > 0)
                {
                    motor.setPower(pow);
                }
                else
                {
                    motor.setPower(0);
                }
            }
            else if(motor.getCurrentPosition() > maxEnc)
            {
                if(pow < 0)
                {
                    motor.setPower(pow);
                }
                else
                {
                    motor.setPower(0);
                }
            }
            else
            {
                motor.setPower(pow);
            }
        }
    }
}
