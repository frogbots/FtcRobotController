package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.frogbots.skystone.control.AccelerationControlledDrivetrain;
import net.frogbots.skystone.control.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import net.frogbots.skystone.drivers.MaxSonarI2CXL;
import net.frogbots.skystone.meta.opmode.FrogOpMode;

@Autonomous
public class AutoMecDriveRed extends FrogOpMode
{
    MaxSonarI2CXL RightSonar;
    MaxSonarI2CXL BackSonar;
    MaxSonarI2CXL LeftSonar;
    MaxSonarI2CXL FrontSonar;
    double rcmToInch = 0;
    double rinch = 0;
    double lcmToInch = 0;
    double linch = 0;
    double bcmToInch = 0;
    double binch = 0;
    double FcmToInch = 0;
    double Finch = 0;
    double err = 0;





    AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;

    @Override
    protected void frog_run()
    {

        LeftSonar = hardwareMap.get(MaxSonarI2CXL.class, "LeftSonar");
        BackSonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");
        //FrontSonar = hardwareMap.get(MaxSonarI2CXL.class, "FrontDistance");


        telemetry.setMsTransmissionInterval(20);

        waitForStart();



        while (opModeIsActive())
        {





            linch = lcmToInch/2.54;
            binch = bcmToInch/2.54;
            telemetry.addData("Left Dist inch", lcmToInch);
            telemetry.addData("Back Dist inch", bcmToInch);

            telemetry.update();

            if(linch < 52) {
                lcmToInch = LeftSonar.getDistanceSync();
                double err = gyroUtils.gyroStrafe(acclCtrl, .2, 0, .013);
                //double err = gyroUtils.gyroStraight(acclCtrl, .2, 0, .017);
                //double err = gyroUtils.gyroRotate(acclCtrl, 180, .01);
            } else if(binch < 44) {
                bcmToInch = BackSonar.getDistanceSync();
               err = gyroUtils.gyroStraight(acclCtrl, .2, 0, .017);

            } else {
                robot.driveTrain.stopMotors();
            }






        }
    }

    @Override
    protected void frog_init()
    {
        acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.08, 1, .05);

        robot.driveTrain.enablePID();
        robot.sensors.imu.init();

        telemetry.setMsTransmissionInterval(50);
    }
    public void ForwardDist(double dist) {
        double bcm = 0;
        while (bcm < dist) {
            bcm = BackSonar.getDistanceSync();
            double err = gyroUtils.gyroStraight(acclCtrl, .2, 0, .017);
        }
    }
    public void BackDist(double dist) {
        double bcm = 0;
        bcm = BackSonar.getDistanceSync();
        while (bcm > dist) {
            double err = gyroUtils.gyroStraight(acclCtrl, -.2, 0, .017);
            bcm = BackSonar.getDistanceSync();
        }
    }
    public void LeftDist(double dist) {
        double lcm = 0;
        lcm = LeftSonar.getDistanceSync();
        while (lcm > dist) {
            double err = gyroUtils.gyroStrafe(acclCtrl, -.2, 0, .013);
            lcm = LeftSonar.getDistanceSync();
        }

    }
    public void RightDist(double dist) {
        double lcm = 0;
        double err = gyroUtils.gyroStrafe(acclCtrl, .2, 0, .013);
    }
}
