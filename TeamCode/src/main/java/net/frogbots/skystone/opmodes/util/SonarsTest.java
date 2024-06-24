//package net.frogbots.skystone.opmodes.util;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import net.frogbots.skystone.meta.opmode.FrogOpMode;
//
//@TeleOp
//public class SonarsTest extends FrogOpMode
//{
//    @Override
//    protected void frog_run()
//    {
//        while (opModeIsActive())
//        {
//            telemetry.addData("Rear sonar", robot.sensors.rearSonar.getDistanceSync());
//            telemetry.addData("Left sonar", robot.sensors.leftSonar.getDistanceSync());
//            telemetry.addData("Right sonar", robot.sensors.rightSonar.getDistanceSync());
//            telemetry.addData("Front (left) sonar", robot.sensors.frontSonarLeftSide.getDistanceSync());
//            telemetry.addData("Front (right) sonar", robot.sensors.frontSonarRightSide.getDistanceSync());
//            telemetry.update();
//        }
//    }
//
//    @Override
//    protected void frog_init()
//    {
//
//    }
//}
