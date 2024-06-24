//package net.frogbots.skystone.opmodes.tele;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import net.frogbots.skystone.control.GyroUtils;
//import net.frogbots.skystone.hardware.Robot;
//import net.frogbots.skystone.meta.misc.Toggler;
//import net.frogbots.skystone.meta.statemachine.StateMachine;
//
//public class LiftControlStateMachine extends StateMachine<LiftControlStateMachine.State>
//{
//    public static int LIFT_BRAKE_POSITION = 150;
//
//    public static int[] liftPosition = new int[] {-5, 435, 1000, 1520, 2040, 2560, 3080, 3600, 4120, 4640, 5160};
//    private int liftPositionIdx;
//
//    Toggler incrementPositionEvent = new Toggler();
//    Toggler decrementPositionEvent = new Toggler();
//
//    Toggler liftUpToggler = new Toggler();
//    Toggler liftDownToggler = new Toggler();
//
//    public LiftControlStateMachine(Robot robot, GyroUtils gyroUtils, Gamepad gamepad1, Gamepad gamepad2)
//    {
//        super(robot, gyroUtils, gamepad1, gamepad2);
//
//        state = State.INACTIVE;
//    }
//
//    enum State
//    {
//        INACTIVE,
//        LIFTING,
//        WATCH_LIFT_LOWERING_FOR_BRAKE
//    }
//
//    @Override
//    public String getName()
//    {
//        return "LiftControlStateMachine";
//    }
//
//    @Override
//    public ReturnState runIteration()
//    {
//        controls();
//
//        switch (state)
//        {
//            case WATCH_LIFT_LOWERING_FOR_BRAKE:
//            {
//                if(robot.lift.liftMotor.getCurrentPosition() < LIFT_BRAKE_POSITION)
//                {
//                    robot.lift.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    switchState(State.INACTIVE);
//                }
//
//                break;
//            }
//        }
//
//        return ReturnState.KEEP_RUNNING_ME;
//    }
//
//    private void controls()
//    {
//        if(liftUpToggler.shouldToggle(gamepad2.right_trigger > .5))
//        {
//            switchState(State.LIFTING);
//            robot.lift.liftMotor.setTargetPosition(liftPosition[liftPositionIdx]);
//            robot.lift.liftMotor.setPower(1);
//            robot.lift.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        }
//        else if(liftDownToggler.shouldToggle(gamepad2.left_trigger > .5))
//        {
//            if(robot.crane.getCommandedRotatePosition() == Crane.RotatePosition.NORMAL)
//            {
//                robot.lift.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                robot.lift.liftMotor.setPower(0);
//
//                switchState(State.WATCH_LIFT_LOWERING_FOR_BRAKE);
//            }
//        }
//        else
//        {
//            handleManualLiftPositionAdj();
//        }
//    }
//
//    private void handleManualLiftPositionAdj()
//    {
//        if(gamepad2.left_bumper && gamepad2.right_bumper)
//        {
//            liftPositionIdx = 0;
//
//            robot.lift.liftMotor.setTargetPosition(liftPosition[liftPositionIdx]);
//        }
//        else if(incrementPositionEvent.shouldToggle(gamepad2.right_bumper))
//        {
//            liftPositionIdx++;
//
//            liftPositionIdx = Math.min(liftPositionIdx, liftPosition.length-1);
//
//            robot.lift.liftMotor.setTargetPosition(liftPosition[liftPositionIdx]);
//        }
//        else if(decrementPositionEvent.shouldToggle(gamepad2.left_bumper))
//        {
//            liftPositionIdx--;
//
//            liftPositionIdx = Math.max(liftPositionIdx, 0);
//
//            robot.lift.liftMotor.setTargetPosition(liftPosition[liftPositionIdx]);
//        }
//    }
//
//    public int getIdx()
//    {
//        return liftPositionIdx+1;
//    }
//}
