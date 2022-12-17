package net.frogbots.skystone.control;

import net.frogbots.skystone.meta.misc.Vector;

public class TrackingWheelIntegratorRoadRunner
{


    double dH_rad;
    double dH;
    double heading_rad;

    private int dPosLeft, dPosRight, dPosAux;

    boolean init;

    private double dX, dY, heading;

    Vector trackingVector = new Vector();
    Vector instantVector = new Vector();

    private int lastPosLeft, lastPosRight, lastPosAux;

    private double TICKS_PER_DEGREE = -401.6311;
    private double TICKS_PER_INCH_NORMAL = 1713.75;
    private double TICKS_PER_INCH_STRAFE = 1702.84;
    double AUX_SPIN_SLIP = 0.101919839;

    double fieldX, fieldY;


    public void update(int cPosLeft, int cPosRight, int cPosAux)
    {
        cPosLeft *= -1;
        cPosRight *= -1;

        if(!init)
        {
            init = true;
            lastPosLeft = cPosLeft;
            lastPosRight = cPosRight;
            lastPosAux = cPosAux;
            return;
        }

        dPosLeft  = cPosLeft  - lastPosLeft;
        dPosRight = cPosRight - lastPosRight;
        dPosAux   = cPosAux  - lastPosAux;

        dH = (dPosLeft-dPosRight)/TICKS_PER_DEGREE;
        dH_rad = Math.toRadians((dPosRight - dPosLeft) / TICKS_PER_DEGREE);
        heading_rad = Math.toRadians((cPosLeft-cPosRight)/TICKS_PER_DEGREE);

        dX = ((dPosLeft + dPosRight) / 2.0) / TICKS_PER_INCH_NORMAL;
        dY = dPosAux/TICKS_PER_INCH_STRAFE - dH * AUX_SPIN_SLIP;
        //double robotYDelta = STRAFE_OFFSET / LEFT_RIGHT_DISTANCE * (leftPosDelta - rightPosDelta) + strafePosDelta;

        double fieldXDelta = Math.sin(dH_rad) / dH_rad * dX
                - (1 - Math.cos(dH_rad)) / dH_rad * dY;

        double fieldYDelta = (1 - Math.cos(dH_rad)) / dH_rad
                + Math.sin(dH_rad) / dH_rad * dY;

        fieldX += Math.cos(heading_rad) * fieldXDelta - Math.sin(heading_rad) * fieldYDelta;
        fieldY += Math.sin(heading_rad) * fieldXDelta + Math.cos(heading_rad) * fieldYDelta;
    }

    public double getX()
    {
        return fieldX;
    }

    public double getY()
    {
        return fieldY;
    }

    public double getHeading()
    {
        return heading;
    }
}
