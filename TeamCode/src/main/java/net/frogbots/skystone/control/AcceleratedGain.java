package net.frogbots.skystone.control;

public class AcceleratedGain
{
    double setpoint;
    double acceleration;
    double val;

    public AcceleratedGain(double setpoint, double acceleration)
    {
        this.setpoint = setpoint;
        this.acceleration = acceleration;
    }

    public double getControlledGain()
    {
        val += acceleration;

        val = Math.min(setpoint, val);

        return val;
    }
}
