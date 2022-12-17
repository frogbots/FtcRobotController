package net.frogbots.skystone.control;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.MovingStatistics;

import net.frogbots.skystone.meta.misc.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TrackingWheelIntegratorArc
{
    private double x, y;

    private double oldX, oldY;

    private int lastPosLeft, lastPosRight, lastPosAux;

    private int dPosLeft, dPosRight, dPosAux;

    double dH, dH_rad;

    private double dX, dY, heading;
    Vector trackingVector = new Vector();
    Vector instantVector = new Vector();
    private double TICKS_PER_DEGREE = -401.6311;
    private double TICKS_PER_INCH_NORMAL = 1713.75;
    private double TICKS_PER_INCH_STRAFE = 1702.84;
    double AUX_SPIN_SLIP = 0.101919839;

    boolean init = false;

    MovingStatistics speedometer = new MovingStatistics(50);

    long lastUpdateTime;

    public void update(int cPosLeft, int cPosRight, int cPosAux, Telemetry telemetry)
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

        heading = (cPosLeft-cPosRight)/TICKS_PER_DEGREE;

        dH = (dPosLeft-dPosRight)/TICKS_PER_DEGREE;

        dH_rad = Math.toRadians(dH);

        dY = ((dPosLeft+dPosRight)/2.0);
        dX = dPosAux;

        dY /= TICKS_PER_INCH_NORMAL;
        dX /= TICKS_PER_INCH_STRAFE;

        dX -= dH * AUX_SPIN_SLIP;

        telemetry.addData("DH", dH);

        if(Math.abs(dH_rad) > 0.0)
        {
            double radiusOfMovement = ((dPosRight/TICKS_PER_INCH_NORMAL)+(dPosLeft/TICKS_PER_INCH_NORMAL))/(2*dH_rad);
            double radiusOfStrafe = dX/dH_rad;

            telemetry.addData("RM", radiusOfMovement);

            dY = (radiusOfMovement * Math.sin(dH_rad)) - (radiusOfStrafe * (1.0 - Math.cos(dH_rad)));
            dX = radiusOfMovement * (1.0 - Math.cos(dH_rad)) + (radiusOfStrafe * Math.sin(dH_rad));
        }
        else
        {
            telemetry.addData("RM", 0);
        }

        instantVector.clear();
        instantVector.addCartesian(dX, dY);

        trackingVector.addPolar(instantVector.getMag(), heading+instantVector.getDir());

        lastPosLeft = cPosLeft;
        lastPosRight = cPosRight;
        lastPosAux = cPosAux;

        oldX = x;
        oldY = y;

        x = trackingVector.getX();
        y = trackingVector.getY();

        if(lastUpdateTime != 0)
        {
            double deltaDisplacement = Math.sqrt(Math.pow(x-oldX, 2) + Math.pow(y-oldY, 2));
            long deltaTime = System.currentTimeMillis() - lastUpdateTime;
            double speed = deltaDisplacement / (deltaTime/1000.0);

            speedometer.add(speed);
        }

        lastUpdateTime = System.currentTimeMillis();
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

    public double getHeading()
    {
        return heading;
    }

    public double speed()
    {
        return speedometer.getMean();
    }
}