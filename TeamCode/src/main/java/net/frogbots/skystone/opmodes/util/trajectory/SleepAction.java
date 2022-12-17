package net.frogbots.skystone.opmodes.util.trajectory;

import net.frogbots.skystone.opmodes.auto.Globals;

public class SleepAction implements MovementPerformer
{
    long start;
    int ms;

    @Override
    public void run()
    {
        start = System.currentTimeMillis();

        while ((System.currentTimeMillis()-start) < ms && Globals.opMode.opModeIsActive())
        {
            Globals.updateTracking();
        }
    }

    public SleepAction(int ms)
    {
        this.ms = ms;
    }
}
