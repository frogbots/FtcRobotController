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

    public double[] XCoord = {20, 72, 120};
    public double[] YCoord = {20, 72, 120};

    public double curXPos;
    public double curYPos;
    public int curOrientation = 0;

    public double targetXPos = Junction_3_1_X;
    public double targetYPos = Junction_3_1_Y;
    public double navX = XCoord[0];
    public double navY = YCoord[0];

    public GyroUtils gyroUtils;

    public MaxSonarI2CXL RightSonar;
    public MaxSonarI2CXL BackSonar;
    public MaxSonarI2CXL LeftSonar;
    //public MaxSonarI2CXL FrontSonar;

    AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;

    public void ForwardtoCone(double t, double power, double Orientation)
    {   double err;
        for(int i = 0; i < t; i++)
            err = gyroUtils.gyroStraight(acclCtrl, power, Orientation, .017);
    }

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
        while (rcm < dist) {
            double err = gyroUtils.gyroStrafe(acclCtrl, -.2, target, .013);
            rcm = RightSonar.getDistanceSync(35);
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
        double err = gyroUtils.gyroRotate(acclCtrl, angle, .015, .1);

        if (angle < 0) {
            while (err < 1) {
                err = gyroUtils.gyroRotate(acclCtrl, angle, .015, .1);
            }
        } else {
            while (err > 1) {
                err = gyroUtils.gyroRotate(acclCtrl, angle, .015, .1);
            }
        }

        //super.robot.driveTrain.stopMotors();
        //acclCtrl.clr();
    }

    public double getPathPos(double targetPos, double[] paths) {

        int i;
        double c,c1;
        double navPos = paths[0];

        for (i = 0; i < paths.length - 1; i++) {

            c = Math.abs(targetPos - paths[i + 1]);
            c1= Math.abs(targetPos - paths[i]);

            if (c <= c1) {
                navPos = paths[i+1];
            }

            System.out.println("getpathpos - navPos =" + navPos);


        }
        return navPos;
    }

    public void Navigate(int curOrientation, double targetXPos, double targetYPos) {

        double curXPos = RightSonar.getDistanceSync();
        double curYPos = BackSonar.getDistanceSync();

        navX = getPathPos(curXPos, XCoord);
        navY = getPathPos(curYPos,YCoord);

        System.out.println("get to path");
        System.out.println("currXPos " + curXPos);
        System.out.println("currYPos " + curYPos);

        System.out.println("navX " + navX);
        System.out.println("navY " + navY);

            if (curYPos < navY) {
                ForwardDist(navY, curOrientation);
            } else if (curYPos > navY) {
                BackDist(navY,curOrientation);
            }


            if (curXPos > navX) {
                    RightDist(navX, curOrientation);
                } else if (curXPos < navX) {
                    LeftDist(navX, curOrientation);
                }


               curXPos = navX; //RightSonar.getDistanceSync();
               curYPos = navY; //BackSonar.getDistanceSync();
               System.out.println("get to target");

               System.out.println("currX " + curXPos);
                System.out.println("currY " + curYPos);

                System.out.println("targetX " + targetXPos);
                System.out.println("targetY " + targetYPos);


        if (targetYPos > curYPos) {
            ForwardDist(targetYPos, curOrientation);
        } else if (targetYPos < curYPos){
            BackDist(targetYPos,curOrientation);
        }

        if (curXPos > targetXPos)
        {
            RightDist(targetXPos, curOrientation);
        } else if (curXPos < targetXPos){
            LeftDist(targetXPos, curOrientation);
        }
        }

    public void LeftNavigate(int curOrientation, double targetXPos, double targetYPos) {

        double curXPos = LeftSonar.getDistanceSync();
        double curYPos = BackSonar.getDistanceSync();

        navX = getPathPos(curXPos, XCoord);
        navY = getPathPos(curYPos,YCoord);

        System.out.println("get to path");
        System.out.println("currXPos " + curXPos);
        System.out.println("currYPos " + curYPos);

        System.out.println("navX " + navX);
        System.out.println("navY " + navY);

        if (curYPos < navY) {
            ForwardDist(navY, curOrientation);
        } else if (curYPos > navY) {
            BackDist(navY,curOrientation);
        }


        if (curXPos < navX) {
            RightLDist(navX, curOrientation);
        } else if (curXPos > navX) {
            LeftLDist(navX, curOrientation);
        }


        curXPos = navX; //RightSonar.getDistanceSync();
        curYPos = navY; //BackSonar.getDistanceSync();
        System.out.println("get to target");

        System.out.println("currX " + curXPos);
        System.out.println("currY " + curYPos);

        System.out.println("targetX " + targetXPos);
        System.out.println("targetY " + targetYPos);


        if (targetYPos > curYPos) {
            ForwardDist(targetYPos, curOrientation);
        } else if (targetYPos < curYPos){
            BackDist(targetYPos,curOrientation);
        }

        if (curXPos < targetXPos)
        {
            RightLDist(targetXPos, curOrientation);
        } else if (curXPos > targetXPos){
            LeftLDist(targetXPos, curOrientation);
        }
    }

}
