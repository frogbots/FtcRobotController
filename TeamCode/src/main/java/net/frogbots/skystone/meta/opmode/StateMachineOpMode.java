package net.frogbots.skystone.meta.opmode;

import net.frogbots.skystone.meta.statemachine.StateMachine;

/**
 * Created by michael on 10/2/18.
 */

public abstract class StateMachineOpMode extends FrogOpMode
{
    private long initialT;

    @Override
    public final void frog_run()
    {
        initialT = System.currentTimeMillis();

        beforeStart();

        while (noYieldOpModeIsActive())
        {
            if(runIteration() == StateMachine.ReturnState.PROCEED)
            {
                break;
            }
        }

        System.out.println("TOTAL OPMODE RUN TIME: " + (System.currentTimeMillis()-initialT)/1000d);

        beforeStop();
    }

    private boolean noYieldOpModeIsActive()
    {
        return !this.isStopRequested() && this.isStarted();
    }

    public void beforeStart()
    {

    }

    protected abstract void beforeStop();

    protected abstract StateMachine.ReturnState runIteration();
}
