package ftc.teamcode.FreightFrenzy;

import net.frogbots.skystone.control.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import net.frogbots.skystone.drivers.MaxSonarI2CXL;
import net.frogbots.skystone.control.GyroUtils;

public class AutoNavigation {

    static final int ROBOT_WIDTH = 31;
    static final int ROBOT_HEIGHT = 16;
    static final int ROBOT_START = 91 - ROBOT_WIDTH / 2;
    static final int SENSOR_MIN = 20;

    public int Junction_3_0_X = 122;
    public int Junction_3_0_Y = 122;
    public int Junction_4_X = 121;
    public int Junction_4_Y = 61;
    public int Junction_3_1_X = 61;
    public int Junction_3_1_Y = 122;

    public int ConeStack_X = SENSOR_MIN;
    public int ConeStack_Y = 152 - ROBOT_WIDTH / 2;

    public int[] XCoord = {25, 91, 152};
    public int[] YCoord = {25, 91, 152};

    public double curXPos;
    public double curYPos;
    public int curOrientation = 0;

    public int targetXPos = Junction_3_1_X;
    public int targetYPos = Junction_3_1_Y;
    public int navX = XCoord[0];
    public int navY = YCoord[0];

    public GyroUtils gyroUtils;

    public MaxSonarI2CXL RightSonar;
    public MaxSonarI2CXL BackSonar;
    public MaxSonarI2CXL LeftSonar;
    //public MaxSonarI2CXL FrontSonar;

    AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;


    public void ForwardDist(double dist, double target) {
        double bcm = 0;
        bcm = BackSonar.getDistanceSync(35);
        while (bcm < dist) {
            double err = gyroUtils.gyroStraight(acclCtrl, .2, target, .017);
            bcm = BackSonar.getDistanceSync();
        }

        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();

    }

    public void BackDist(double dist, double target) {
        double bcm = 0;
        bcm = BackSonar.getDistanceSync(35);
        while (bcm > dist) {
            double err = gyroUtils.gyroStraight(acclCtrl, -0.2, target, .017);
            bcm = BackSonar.getDistanceSync(35);
        }
        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();
    }

    public void LeftLDist(double dist, double target) {
        double lcm = 0;
        lcm = LeftSonar.getDistanceSync(35);

        while (dist < lcm) {
            double err = gyroUtils.gyroStrafe(acclCtrl, -.2, 0, .013);
            lcm = LeftSonar.getDistanceSync(35);
        }
        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();
    }

    public void RightLDist(double dist, double target) {
        double lcm = 0;
        lcm = LeftSonar.getDistanceSync(35);


        while (dist > lcm) {
            double err = gyroUtils.gyroStrafe(acclCtrl, .2, target, .013);
            lcm = LeftSonar.getDistanceSync(35);
        }
        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();
    }

    public void LeftDist(double dist, double target) {
        double rcm = 0;
        rcm = RightSonar.getDistanceSync(35);
System.out.println("lcm " + rcm);
        while (rcm < dist) {
            double err = gyroUtils.gyroStrafe(acclCtrl, -.2, target, .013);
            rcm = RightSonar.getDistanceSync(35);
            System.out.println("lcm " + rcm);
        }

        // robot.driveTrain.stopMotors();
        // acclCtrl.clr();
    }

    public void RightDist(double dist, double target) {
        double rcm = 0;
        rcm = RightSonar.getDistanceSync(35);
        while (rcm > dist) {
            double err = gyroUtils.gyroStrafe(acclCtrl, .2, target, .013);
            rcm = RightSonar.getDistanceSync(35);

        }

        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();

    }

    public void RotateAngle(double angle) {
        double err = gyroUtils.gyroRotate(acclCtrl, angle, .015);

        if (angle < 0) {
            while (err < 1) {
                err = gyroUtils.gyroRotate(acclCtrl, angle, .015);
            }
        } else {
            while (err > 1) {
                err = gyroUtils.gyroRotate(acclCtrl, angle, .015);
            }
        }

        //super.robot.driveTrain.stopMotors();
        //acclCtrl.clr();
    }

    public void Navigate(int curOrientation, int targetXPos, int targetYPos) {

        double curXPos = RightSonar.getDistanceSync();
        double curYPos = BackSonar.getDistanceSync();

        System.out.println("currXPos " + curXPos);
        System.out.println("currYPos " + curYPos);

        for (int i = 0; i < 3; i++) {
            if (targetXPos >= XCoord[i]) {
                navX = XCoord[i];
            }
            if (targetYPos >= YCoord[i]) {
                navY = YCoord[i];
            }

        }

        System.out.println("navX " + navX);
        System.out.println("navY " + navY);

                if (curXPos > navX) {
                    RightDist(navX, curOrientation);
                } else {
                    LeftDist(navX, curOrientation);
                }

                if (curYPos < navY) {
                    ForwardDist(navY, curOrientation);
                } else {
                    BackDist(navY,curOrientation);
                }

            if (curXPos > targetXPos) {
                RightDist(targetXPos, curOrientation);
            } else {
                LeftDist(targetXPos, curOrientation);
            }

            if (targetYPos > curYPos) {
                ForwardDist(targetYPos, curOrientation);
            } else {
                BackDist(targetYPos,curOrientation);
            }
        }

    }
