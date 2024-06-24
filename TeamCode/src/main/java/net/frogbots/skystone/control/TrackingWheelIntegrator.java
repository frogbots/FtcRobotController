package net.frogbots.skystone.control;

import android.view.InputDevice;

import com.qualcomm.robotcore.util.MovingStatistics;

import net.frogbots.skystone.meta.misc.Vector;

import org.firstinspires.ftc.robotcore.internal.ui.InputManager;

public class TrackingWheelIntegrator
{
    private double x, y;

    private double oldX, oldY;

    private int lastPosLeft, lastPosRight, lastPosAux;

    private int dPosLeft, dPosRight, dPosAux;

    double dH;

    private double dX, dY, heading;
    Vector trackingVector = new Vector();
    Vector instantVector = new Vector();
    private double TICKS_PER_DEGREE = -401.6311;
    private double TICKS_PER_INCH_NORMAL = 1713.75;
    private double TICKS_PER_INCH_STRAFE = 1702.84;
    double AUX_SPIN_SLIP = 0.101919839;

    boolean init = false;

    boolean useImu = false;

    double wheelHead;

    double lastImu;

    MovingStatistics speedometer = new MovingStatistics(50);

    long lastUpdateTime;

    double headingOffsetDegrees;

    public void setHeadingOffsetDegrees(double imuHeading)
    {
        this.headingOffsetDegrees = heading - imuHeading;

        System.out.println(String.format("Sync heading w/ IMU... IMU says %f wheels say %f offset is %f", imuHeading, heading, headingOffsetDegrees));
    }

    public void update(int cPosLeft, int cPosRight, int cPosAux)
    {
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

        heading = (cPosRight-cPosLeft)/TICKS_PER_DEGREE;

        dH = (dPosRight-dPosLeft)/TICKS_PER_DEGREE;

        dY = -((dPosLeft+dPosRight)/2.0);
        dX = dPosAux;

        dY /= TICKS_PER_INCH_NORMAL;
        dX /= TICKS_PER_INCH_STRAFE;

        dX -= dH * AUX_SPIN_SLIP;

        instantVector.setCartesian(dX, dY);

        trackingVector.addPolar(instantVector.getMag(), heading+instantVector.getDir()-headingOffsetDegrees);

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

    public double getWheelHead()
    {
        return wheelHead;
    }
}