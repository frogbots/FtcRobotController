package net.frogbots.skystone.control;

import net.frogbots.skystone.meta.misc.Vector;

public class MecanumPositionTracker
{
    private double x, y;
    private int lastPosFL, lastPosFR, lastPosRR, lastPosRL;
    private int dPosFL, dPosFR, dPosRR, dPosRL;
    private double dX, dY, heading;
    private int avgLeft, avgRight;
    Vector trackingVector = new Vector();
    Vector instantVector = new Vector();
    private double TICKS_PER_DEGREE = 18.07;
    private double TICKS_PER_INCH_NORMAL = 43.13;
    private double TICKS_PER_INCH_STRAFE = 50;

    public void update(int cPosFL, int cPosFR, int cPosRL, int cPosRR)
    {
        dPosFL = cPosFL - lastPosFL;
        dPosFR = cPosFR - lastPosFR;
        dPosRL = cPosRL - lastPosRL;
        dPosRR = cPosRR - lastPosRR;

        dY = (+dPosFL+dPosFR+dPosRL+dPosRR)/4.0;
        dX = (+dPosFL-dPosFR-dPosRL+dPosRR)/4.0;

        avgLeft  = (cPosFL + cPosRL)/2;
        avgRight = (cPosRR + cPosFR)/2;

        heading = (avgRight-avgLeft)/TICKS_PER_DEGREE;

        instantVector.clear();
        instantVector.addCartesian(dX/TICKS_PER_INCH_STRAFE, dY/TICKS_PER_INCH_NORMAL);

        trackingVector.addPolar(instantVector.getMag(), heading+instantVector.getDir());

        lastPosFL = cPosFL;
        lastPosFR = cPosFR;
        lastPosRL = cPosRL;
        lastPosRR = cPosRR;

        x = trackingVector.getX();
        y = trackingVector.getY();
    }

    public void clear()
    {
        trackingVector.clear();
    }

    public double getX()
    {
        return x;
    }

    public double getY()
    {
        return y;
    }

    public void setX(double x)
    {
        trackingVector.clear();
        trackingVector.addCartesian(x,0);
    }

    public double getHeading()
    {
        return heading;
    }
}