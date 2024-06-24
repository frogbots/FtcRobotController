package net.frogbots.skystone.control;

import com.qualcomm.hardware.bosch.BNO055IMU;

import net.frogbots.skystone.drivers.FrogBNO055;
import net.frogbots.skystone.hardware.components.drivebase.DriveTrain;
import net.frogbots.skystone.hardware.components.drivebase.MotorPowers;
import net.frogbots.skystone.meta.misc.AngleManipulator;


public class GyroUtils
{
    private FrogBNO055 imu;
    private DriveTrain driveTrain;

    public GyroUtils(FrogBNO055 imu, DriveTrain driveTrain)
    {
        this.imu = imu;
        this.driveTrain = driveTrain;
    }

    private double getHeading()
    {
        return imu.pollAndGetYaw();
    }

    //------------------------------------------------------------------------------------------------
    // Rotate
    //------------------------------------------------------------------------------------------------

    public double gyroRotate(double target, double Kp)
    {
        double err = AngleManipulator.manipulateDiscontinuity(target, getHeading());
        driveTrain.setMotorPowers(-err*Kp, err*Kp);
        return err;
    }

    public double gyroRotate(AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl, double target, double Kp)
    {
        double err = AngleManipulator.manipulateDiscontinuity(target, getHeading());

        double turnPower = err*Kp;

        driveTrain.setMotorPowers(acclCtrl.getAccelerationControlledPowers(new MotorPowers(-turnPower, turnPower)));

        return err;
    }

    public double gyroRotate(AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl, double target, double Kp, double minPow)
    {
        double err = AngleManipulator.manipulateDiscontinuity(target, getHeading());

        double turnPower = err*Kp;

        turnPower = enforceMinPower(turnPower, minPow);

        driveTrain.setMotorPowers(acclCtrl.getAccelerationControlledPowers(new MotorPowers(-turnPower, turnPower)));

        return err;
    }

    //------------------------------------------------------------------------------------------------
    // Forward / backward
    //------------------------------------------------------------------------------------------------

    public double gyroStraight(double pow, double target, double Kp)
    {
        return setPowersWithHeadingCorrection(MecanumDrive.calcCartesian(pow, 0, 0), target, Kp);
    }

    public double gyroStraight(AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl, double pow, double target, double Kp)
    {
        return setPowersWithHeadingCorrection(acclCtrl, MecanumDrive.calcCartesian(pow, 0, 0), target, Kp);
    }

    //------------------------------------------------------------------------------------------------
    // Strafing
    //------------------------------------------------------------------------------------------------

    public double gyroStrafe(double pow, double target, double Kp)
    {
        return setPowersWithHeadingCorrection(MecanumDrive.calcCartesian(0, pow, 0), target, Kp);
    }

    public double gyroStrafe(AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl, double pow, double target, double Kp)
    {
        return setPowersWithHeadingCorrection(acclCtrl, MecanumDrive.calcCartesian(0, pow, 0), target, Kp);
    }

    //------------------------------------------------------------------------------------------------
    // Misc
    //------------------------------------------------------------------------------------------------
    public double setPowersWithHeadingCorrection(MotorPowers powers, double target, double Kp)
    {
        double err = AngleManipulator.manipulateDiscontinuity(target, getHeading());

        //System.out.println("Gyro err " + err);

        powers.frontLeft  -= err*Kp;
        powers.rearLeft   -= err*Kp;
        powers.frontRight += err*Kp;
        powers.rearRight  += err*Kp;

        driveTrain.setMotorPowers(powers);

        return err;
    }

    public double setPowersWithHeadingCorrection(AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl, MotorPowers powers, double target, double Kp)
    {
        powers = acclCtrl.getAccelerationControlledPowers(powers);

        double err = AngleManipulator.manipulateDiscontinuity(target, getHeading());

        //System.out.println("setPowersWithHeadingCorrection gyro err " + err);

        powers.frontLeft  -= err*Kp;
        powers.rearLeft   -= err*Kp;
        powers.frontRight += err*Kp;
        powers.rearRight  += err*Kp;

        driveTrain.setMotorPowers(powers);

        //System.out.println("setPowersWithHeadingCorrection prenormalized powers: " + powers.toString());

        return err;
    }

    public static double enforceMinPower(double input, double minPower)
    {
        minPower = Math.abs(minPower); //make sure minPower is always positive

        if(input < 0) //Input power is negative
        {
            /*
             * Return the smallest (err, biggest negative) of either the user's input power, or minPower
             */
            input = Math.min(input, -minPower);
        }
        else // Input power is positive
        {
            /*
             * Return the biggest of either the user's input power, or minPower
             */
            input = Math.max(input, minPower);
        }

        return input;
    }
}
