package net.frogbots.skystone.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import net.frogbots.skystone.hardware.components.Sensors;
import net.frogbots.skystone.hardware.components.drivebase.SkyStoneDriveBase;


/**
 * Created by michael on 10/2/18.
 */

public class Robot
{
    public Sensors sensors = new Sensors();
    public SkyStoneDriveBase driveTrain = new SkyStoneDriveBase();

    public void init(HardwareMap hardwareMap, boolean resetEncoders)
    {


        driveTrain.init(hardwareMap);

        if(resetEncoders)
        {
            driveTrain.resetEncoders();
        }

        sensors.init(hardwareMap);
    }
}
