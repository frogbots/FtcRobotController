package net.frogbots.skystone.hardware.components.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import net.frogbots.skystone.hardware.RobotComponent;

public abstract class DriveTrainBase implements DriveTrain, RobotComponent
{
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;

    private boolean pidEnabledNow = false;

    private MotorPowers reusablePowsObject = new MotorPowers();

    private int frontLeftSoftResetOffset;
    private int frontRightSoftResetOffest;
    private int rearLeftSoftResetOffset;
    private int rearRightSoftResetOffset;

    @Override
    public void setMotorPowers(MotorPowers pows)
    {
        normalize(pows);

        frontLeft.setPower(pows.frontLeft);
        frontRight.setPower(pows.frontRight);
        rearRight.setPower(pows.rearRight);
        rearLeft.setPower(pows.rearLeft);
    }

    @Override
    public int encoderFrontLeft()
    {
        return frontLeft.getCurrentPosition() - frontLeftSoftResetOffset;
    }

    @Override
    public int encoderFrontRight()
    {
        return frontRight.getCurrentPosition() - frontRightSoftResetOffest;
    }

    @Override
    public int encoderRearLeft()
    {
        return rearLeft.getCurrentPosition() - rearLeftSoftResetOffset;
    }

    @Override
    public int encoderRearRight()
    {
        return rearRight.getCurrentPosition() - rearRightSoftResetOffset;
    }

    @Override
    public void enableBrake(boolean brake)
    {
        if(brake)
        {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else
        {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void enablePID()
    {
        if(!pidEnabledNow)
        {
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            pidEnabledNow = true;
        }
    }

    @Override
    public void disablePID()
    {
        if(pidEnabledNow)
        {
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            pidEnabledNow = false;
        }
    }

    @Override
    public void stopMotors()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    @Override
    public void setMotorPowers(double pow)
    {
        reusablePowsObject.frontLeft = pow;
        reusablePowsObject.frontRight = pow;
        reusablePowsObject.rearLeft = pow;
        reusablePowsObject.rearRight = pow;

        setMotorPowers(reusablePowsObject);
    }

    @Override
    public void setMotorPowers(double left, double right)
    {
        //Left
        reusablePowsObject.frontLeft = left;
        reusablePowsObject.rearLeft = left;

        //Right
        reusablePowsObject.frontRight = right;
        reusablePowsObject.rearRight = right;

        setMotorPowers(reusablePowsObject);
    }

    @Override
    public void setMotorPowers(double fl, double fr, double rl, double rr)
    {
        reusablePowsObject.frontLeft = fl;
        reusablePowsObject.frontRight = fr;
        reusablePowsObject.rearLeft = rl;
        reusablePowsObject.rearRight = rr;

        setMotorPowers(reusablePowsObject);
    }

    @Override
    public void resetEncoders()
    {
        frontLeftSoftResetOffset = 0;
        frontRightSoftResetOffest = 0;
        rearLeftSoftResetOffset = 0;
        rearRightSoftResetOffset = 0;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(pidEnabledNow)
        {

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else //open loop
        {
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void softResetEncoders()
    {
        frontLeftSoftResetOffset = frontLeft.getCurrentPosition();
        frontRightSoftResetOffest = frontRight.getCurrentPosition();
        rearLeftSoftResetOffset = rearLeft.getCurrentPosition();
        rearRightSoftResetOffset = rearRight.getCurrentPosition();
    }

    private static void normalize(MotorPowers powers)
    {
        double ratio;
        double maxLeft;
        double maxRight;
        double max;

        /*
         * Are any of the computed wheel powers greater than 1?
         */
        if(Math.abs(powers.frontLeft)          > 1
                || Math.abs(powers.frontRight) > 1
                || Math.abs(powers.rearLeft)   > 1
                || Math.abs(powers.rearRight)  > 1)
        {
            /*
             * Yeah, figure out which one
             */
            maxLeft  = Math.max(Math.abs(powers.frontLeft),  Math.abs(powers.rearLeft));
            maxRight = Math.max(Math.abs(powers.frontRight), Math.abs(powers.rearRight));
            max      = Math.max(maxLeft, maxRight);

            ratio = 1 / max; //Create a ratio to normalize them all

            powers.frontLeft  *= ratio;
            powers.frontRight *= ratio;
            powers.rearLeft   *= ratio;
            powers.rearRight  *= ratio;
        }
    }
}
