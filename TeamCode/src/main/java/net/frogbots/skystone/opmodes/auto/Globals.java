package net.frogbots.skystone.opmodes.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import net.frogbots.skystone.control.TrackingWheelIntegrator;
import net.frogbots.skystone.cv.SkystoneIndex;
import net.frogbots.skystone.cv.SkystonePosition;
import net.frogbots.skystone.hardware.Robot;
import net.frogbots.skystone.hardware.components.drivebase.SkyStoneDriveBase;
import net.frogbots.skystone.meta.misc.Alliance;

public class Globals
{
    public static volatile SkystoneIndex skystoneIndex = SkystoneIndex.POS2;
    public static Alliance alliance = Alliance.BLUE;
    public static LinearOpMode opMode;
    public static TrackingWheelIntegrator trackingWheelIntegrator;
    public static SkyStoneDriveBase driveBase;
    public static LynxModule odoModule;

    public static void requestOpModeStop()
    {
        opMode.requestOpModeStop();
    }

    public static void setIndex(SkystonePosition position)
    {
        if(Globals.alliance == Alliance.RED)
        {
            if(position == SkystonePosition.LEFT)
            {
                skystoneIndex = SkystoneIndex.POS1;
            }
            else if(position == SkystonePosition.RIGHT)
            {
                skystoneIndex = SkystoneIndex.POS3;
            }
        }
        else if(Globals.alliance == Alliance.BLUE)
        {
            if(position == SkystonePosition.LEFT)
            {
                skystoneIndex = SkystoneIndex.POS3;
            }
            else if(position == SkystonePosition.RIGHT)
            {
                skystoneIndex = SkystoneIndex.POS1;
            }
        }

        if(position == SkystonePosition.CENTER)
        {
            skystoneIndex = SkystoneIndex.POS2;
        }
    }

    public static void updateTracking()
    {
        LynxModule.BulkData bulkData = odoModule.getBulkData();

        int left = bulkData.getMotorCurrentPosition(0);
        int right = bulkData.getMotorCurrentPosition(1);
        int aux = bulkData.getMotorCurrentPosition(2);

        trackingWheelIntegrator.update(left, right, aux);

        opMode.telemetry.addData("X", trackingWheelIntegrator.getX());
        opMode.telemetry.addData("Y", trackingWheelIntegrator.getY());
        opMode.telemetry.addData("wheelH", trackingWheelIntegrator.getHeading());
        opMode.telemetry.update();
    }
}
