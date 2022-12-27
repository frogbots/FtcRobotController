package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import net.frogbots.skystone.control.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import net.frogbots.skystone.drivers.MaxSonarI2CXL;
import net.frogbots.skystone.control.GyroUtils;
import net.frogbots.skystone.meta.opmode.FrogOpMode;

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

    public double[] XCoord = {20, 71, 130};
    public double[] YCoord = {20, 71, 130};

    public double curXPos;
    public double curYPos;
    public int curOrientation = 0;

    public double targetXPos = Junction_3_1_X;
    public double targetYPos = Junction_3_1_Y;
    public double navX = XCoord[0];
    public double navY = YCoord[0];

    public FrogOpMode op;

    public GyroUtils gyroUtils;

    public MaxSonarI2CXL RightSonar;
    public MaxSonarI2CXL BackSonar;
    public MaxSonarI2CXL LeftSonar;
    public DcMotorEx RR;
    public DcMotorEx RL;
    public DcMotorEx FR;
    public DcMotorEx FL;
    //public MaxSonarI2CXL FrontSonar;

    AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;

    public void ForwardtoCone(double t, double power, double Orientation)
    {   double err;
        for(int i = 0; i < t; i++)
            err = gyroUtils.gyroStraight(acclCtrl, power, Orientation, .017);
    }

    public void ForwardDist(double dist, double pow ,double target) {
        double bcm = 0;
        bcm = BackSonar.getDistanceSync(35);
        while (bcm < dist && op.opModeIsActive()) {
            System.out.println("bcm f = "+bcm);
            System.out.println("FR = " +FR.getCurrentPosition());
            System.out.println("FL = " +FL.getCurrentPosition());
            System.out.println("RR = " +RR.getCurrentPosition());
            System.out.println("RL = " +RL.getCurrentPosition());
            double err = gyroUtils.gyroStraight(acclCtrl, pow, target, .017);
            bcm = BackSonar.getDistanceSync();
        }

        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();

    }

    public void BackDist(double dist, double pow, double target) {
        double bcm = 0;
        bcm = BackSonar.getDistanceSync(35);
        while (bcm > dist && op.opModeIsActive()) {
            System.out.println("bcm b = "+bcm);
            double err = gyroUtils.gyroStraight(acclCtrl, pow, target, .017);
            bcm = BackSonar.getDistanceSync(35);
        }
        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();
    }

    public void LeftLDist(double dist, double pow, double target) {
        double lcm = 0;
        lcm = LeftSonar.getDistanceSync(35);

        while (dist < lcm && op.opModeIsActive()) {
            double err = gyroUtils.gyroStrafe(acclCtrl, pow, 0, .013);
            lcm = LeftSonar.getDistanceSync(35);
        }
        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();
    }

    public void RightLDist(double dist, double pow, double target) {
        double lcm = 0;
        lcm = LeftSonar.getDistanceSync(35);


        while (dist > lcm && op.opModeIsActive()) {
            double err = gyroUtils.gyroStrafe(acclCtrl, pow, target, .013);
            lcm = LeftSonar.getDistanceSync(35);
        }
        //robot.driveTrain.stopMotors();
        //acclCtrl.clr();
    }

    public void LeftDist(double dist, double pow, double target) {
        double rcm = 0;
        rcm = RightSonar.getDistanceSync(35);

        while (rcm < dist && op.opModeIsActive())  {
            System.out.println("rcm l = "+rcm);
            double err = gyroUtils.gyroStrafe(acclCtrl, pow, target, .013);
            rcm = RightSonar.getDistanceSync(35);
        }

        // robot.driveTrain.stopMotors();
        // acclCtrl.clr();
    }

    public void RightDist(double dist, double pow, double target) {
        double rcm = 0;
        rcm = RightSonar.getDistanceSync(35);
        while (rcm > dist && op.opModeIsActive()) {
            System.out.println("rcm r = "+rcm);
            double err = gyroUtils.gyroStrafe(acclCtrl, pow, target, .013);
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
                ForwardDist(navY, 2, curOrientation);
            } else if (curYPos > navY) {
                BackDist(navY, -.2, curOrientation);
            }


            if (curXPos > navX) {
                    RightDist(navX, .2, curOrientation);
                } else if (curXPos < navX) {
                    LeftDist(navX, .2, curOrientation);
                }


               curXPos = navX; //RightSonar.getDistanceSync();
               curYPos = navY; //BackSonar.getDistanceSync();
               System.out.println("get to target");

               System.out.println("currX " + curXPos);
                System.out.println("currY " + curYPos);

                System.out.println("targetX " + targetXPos);
                System.out.println("targetY " + targetYPos);


        if (targetYPos > curYPos) {
            ForwardDist(targetYPos, .2, curOrientation);
        } else if (targetYPos < curYPos){
            BackDist(targetYPos, -.2, curOrientation);
        }

        if (curXPos > targetXPos)
        {
            RightDist(targetXPos, .2, curOrientation);
        } else if (curXPos < targetXPos){
            LeftDist(targetXPos, -.2, curOrientation);
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
            ForwardDist(navY, .2,curOrientation);
        } else if (curYPos > navY) {
            BackDist(navY, -.2, curOrientation);
        }


        if (curXPos < navX) {
            RightLDist(navX, -.2, curOrientation);
        } else if (curXPos > navX) {
            LeftLDist(navX, .2, curOrientation);
        }


        curXPos = navX; //RightSonar.getDistanceSync();
        curYPos = navY; //BackSonar.getDistanceSync();
        System.out.println("get to target");

        System.out.println("currX " + curXPos);
        System.out.println("currY " + curYPos);

        System.out.println("targetX " + targetXPos);
        System.out.println("targetY " + targetYPos);


        if (targetYPos > curYPos) {
            ForwardDist(targetYPos, .2, curOrientation);
        } else if (targetYPos < curYPos){
            BackDist(targetYPos, -.2, curOrientation);
        }

        if (curXPos < targetXPos)
        {
            RightLDist(targetXPos, -.2, curOrientation);
        } else if (curXPos > targetXPos){
            LeftLDist(targetXPos, .2, curOrientation);
        }
    }

}
