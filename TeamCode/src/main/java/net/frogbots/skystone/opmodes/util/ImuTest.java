package net.frogbots.skystone.opmodes.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.frogbots.skystone.drivers.FrogBNO055;

@TeleOp
public class ImuTest extends LinearOpMode
{
    FrogBNO055 imu;

    @Override
    public void runOpMode()
    {
        imu = new FrogBNO055( hardwareMap.get(BNO055IMU.class, "external_IMU"));

        imu.init();

        waitForStart();

        telemetry.setMsTransmissionInterval(20);

        while (opModeIsActive())
        {
            imu.poll();
            telemetry.addData("Yaw", imu.getYaw());
            telemetry.addData("Pitch", imu.getPitch());
            telemetry.addData("Roll", imu.getRoll());
            telemetry.update();
        }
    }
}
