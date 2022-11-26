//package net.frogbots.skystone.opmodes.tele;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import net.frogbots.skystone.hardware.Robot;
//import net.frogbots.skystone.meta.misc.Toggler;
//import net.frogbots.skystone.meta.misc.Vector;
//
//public class ExtController
//{
//    private Robot robot;
//    private Gamepad gamepad1;
//    private Gamepad gamepad2;
//
//    private Toggler extBimpOut_Increment = new Toggler();
//    private Toggler extBimpIn_Decrement = new Toggler();
//
//    private int bimp = 0;
//
//    private final int bimp_min = -5;
//    private final int bimp_max = 5;
//
//    private Vector bimpVector = new Vector();
//
//    private double bimp_gain = .01;
//
//    public ExtController(Robot robot, Gamepad gamepad1, Gamepad gamepad2)
//    {
//        this.robot = robot;
//        this.gamepad1 = gamepad1;
//        this.gamepad2 = gamepad2;
//    }
//
//    public void run(boolean shouldSetServoPos)
//    {
//        bimpVector.clear();
//        bimpVector.addCartesian(gamepad2.right_stick_x, gamepad2.right_stick_y);
//
//        //System.out.println(String.format("X:%.1f, Y:%.1f, Dir: %.1f Mag:%.1f", bimpVector.getX(), bimpVector.getY(), bimpVector.getDir(), bimpVector.getMag()));
//
//        double dir = bimpVector.getDir();
//
//        boolean fortyFiveTo135 = dir > 45.0 && dir < 135.0;
//        boolean up = fortyFiveTo135 && bimpVector.getMag() > .5;
//
//        boolean twoTwentyFiveTo315 = dir > 225.0 && dir < 315.0;
//        boolean down = twoTwentyFiveTo315 && bimpVector.getMag() > .5;
//
//        if(extBimpOut_Increment.shouldToggle(up))
//        {
//            bimp++;
//
//            bimp = Math.min(bimp, bimp_max);
//        }
//        else if(extBimpIn_Decrement.shouldToggle(down))
//        {
//            bimp--;
//
//            bimp = Math.max(bimp, bimp_min);
//        }
//    }
//
//    public double getBimpAdjustedPostion()
//    {
//        return Crane.EXT_POS_FIRST + (bimp * bimp_gain);
//    }
//
//    public int getBimp()
//    {
//        return bimp;
//    }
//}
