package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.frogbots.skystone.drivers.MaxSonarI2CXL;

@TeleOp
public class I2cTest extends LinearOpMode {

    MaxSonarI2CXL Sensor;


    @Override
    public void runOpMode() throws InterruptedException {

        Sensor= (MaxSonarI2CXL) hardwareMap.get(MaxSonarI2CXL.class, "Sensor");

        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("sens1", Sensor.getDistanceSync());
            telemetry.update();
        }
    }
}
