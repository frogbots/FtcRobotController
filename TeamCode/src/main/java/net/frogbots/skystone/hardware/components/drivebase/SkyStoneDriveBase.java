package net.frogbots.skystone.hardware.components.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Created by michael on 10/2/18.
 */

public class SkyStoneDriveBase extends DriveTrainBase
{
    @Override
    public void init(HardwareMap hardwareMap)
    {
        frontLeft = (DcMotorEx)hardwareMap.dcMotor.get("FL");
        frontRight = (DcMotorEx)hardwareMap.dcMotor.get("FR");
        rearLeft = (DcMotorEx)hardwareMap.dcMotor.get("RL");
        rearRight = (DcMotorEx)hardwareMap.dcMotor.get("RR");

        resetEncoders();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(10, 5, 0));
        frontRight.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(10, 5, 0));
        rearRight.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(10, 5, 0));
        rearRight.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(10, 5, 0));
    }
}
