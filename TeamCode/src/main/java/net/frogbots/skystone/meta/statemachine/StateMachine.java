package net.frogbots.skystone.meta.statemachine;

import com.qualcomm.robotcore.hardware.Gamepad;

import net.frogbots.skystone.control.GyroUtils;
import net.frogbots.skystone.hardware.Robot;

/**
 * Created by michael on 10/2/18.
 */

public abstract class StateMachine<STATE_VAR extends Enum<STATE_VAR>>
{
    public STATE_VAR state;
    public Robot robot;
    public GyroUtils gyroUtils;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    private long stateStartTime;

    public enum ReturnState
    {
        KEEP_RUNNING_ME,
        PROCEED
    }

    public StateMachine(Robot robot, GyroUtils gyroUtils, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.robot = robot;
        this.gyroUtils = gyroUtils;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void switchState(STATE_VAR state)
    {
        System.out.println("[" + getName() + "] Switching state to: " + state.toString());
        this.state = state;
        stateStartTime = System.currentTimeMillis();
    }

    public long getElapsedStateTime()
    {
        return System.currentTimeMillis() - stateStartTime;
    }

    public void nested(StateMachine stateMachine, STATE_VAR switchToWhenFinished)
    {
        if(stateMachine.runIteration() == ReturnState.PROCEED)
        {
            switchState(switchToWhenFinished);
        }
    }

    public abstract String getName();
    public abstract ReturnState runIteration();
}
