package net.frogbots.skystone.control;

import com.qualcomm.robotcore.hardware.Servo;

public class VariableSpeedServoController
{
    private Servo servo;
    private double incrementRate;
    private double targetPosition;
    private double currentPosition;

    public VariableSpeedServoController(Servo servo)
    {
        this.servo = servo;
    }

    public void setIncrementRate(double incrementRate)
    {
        this.incrementRate = Math.abs(incrementRate);
    }

    public void setPosition(double position)
    {
        targetPosition = position;
    }

    public void notifyCurrentPosition(double currentPosition)
    {
        this.currentPosition = currentPosition;
    }

    public void update()
    {
        if(currentPosition == targetPosition)
        {
            return;
        }

        if(currentPosition < targetPosition)
        {
            currentPosition += incrementRate;
            currentPosition = Math.min(currentPosition, targetPosition);
        }
        else if(currentPosition > targetPosition)
        {
            currentPosition -= incrementRate;
            currentPosition = Math.max(currentPosition, targetPosition);
        }

        servo.setPosition(currentPosition);
    }
}
