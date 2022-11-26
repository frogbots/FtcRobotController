package net.frogbots.skystone.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;

public class VexMotor
{
    private CRServo baseCrServo;
    private double MAX_POWER_DUE_TO_PWM_RANGE = .85;

    public VexMotor(CRServo baseCrServo)
    {
        this.baseCrServo = baseCrServo;
    }

    public void setPower(double pos)
    {
        Range.throwIfRangeIsInvalid(pos, -1, 1);

        baseCrServo.setPower(Range.scale(pos, -1, 1, -MAX_POWER_DUE_TO_PWM_RANGE, MAX_POWER_DUE_TO_PWM_RANGE));
    }
}
