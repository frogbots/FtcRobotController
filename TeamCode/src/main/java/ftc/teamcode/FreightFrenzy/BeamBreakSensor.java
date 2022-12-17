package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BeamBreakSensor {

    public DistanceSensor beambreak;


    public boolean JunctionBreak() {
        if (beambreak.getDistance(DistanceUnit.CM) > 7.5 && beambreak.getDistance(DistanceUnit.CM) < 12) {
            return true;
        }
        return false;
    }
    public boolean ConeBreak() {
        if (beambreak.getDistance(DistanceUnit.CM) > 7.5 && beambreak.getDistance(DistanceUnit.CM) < 12) {
            return true;
        }
        return false;
    }
}



