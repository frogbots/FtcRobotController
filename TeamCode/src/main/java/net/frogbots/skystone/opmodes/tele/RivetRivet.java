//package net.frogbots.skystone.opmodes.tele;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.RunShellCommand;
//
//import net.frogbots.skystone.control.AccerlationControlledDrivetrainPowerGeneratorForAuto;
//import net.frogbots.skystone.control.MecanumDrive;
//import net.frogbots.skystone.meta.misc.Toggler;
//import net.frogbots.skystone.meta.misc.Vector;
//import net.frogbots.skystone.meta.opmode.FrogOpMode;
//import net.frogbots.skystone.opmodes.auto.StoneKeeperStateMachine;
//
//@TeleOp(name = "A - RivetRivet")
//public class RivetRivet extends FrogOpMode
//{
//    Toggler booperToggler = new Toggler();
//    private boolean booperIsBooping = false;
//
//    Toggler intakeForwardToggle = new Toggler();
//    private boolean intakeComingOutOfReverse = false;
//    private boolean intakeOn = false;
//
//    Toggler clawReleaseToggler = new Toggler();
//
//    Toggler rstToggle = new Toggler();
//
//    LiftControlStateMachine liftControlStateMachine;
//
//    StoneKeeperStateMachine stoneKeeperStateMachine;
//
//    Vector stoneOrientationVector = new Vector();
//
//    ExtController extController;
//
//    AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;
//
//    double powerScalar = 1;
//
//    boolean extShouldSet = true;
//
//    @Override
//    public void frog_run()
//    {
//        while (opModeIsActive())
//        {
//            liftControlStateMachine.runIteration();
//
//            /*
//             * DT layer takes care of making sure this swap doesn't happen every time
//             */
//            if(liftControlStateMachine.state == LiftControlStateMachine.State.LIFTING)
//            {
//                powerScalar = .5;
//                robot.driveTrain.enablePID();
//            }
//            else
//            {
//                powerScalar = 1;
//                robot.driveTrain.disablePID();
//            }
//
//            if(robot.crane.getCommandedExtPos() != Crane.ExtPos.NORMAL)
//            {
//                stoneRotationControls();
//            }
//
//            stoneKeeperStateMachine.runIteration();
//
//            MecanumDrive.cartesian(robot.driveTrain,
//                    -gamepad1.left_stick_y*powerScalar, //Main
//                    gamepad1.left_stick_x*powerScalar, //Strafe
//                    gamepad1.right_stick_x*powerScalar*.75); //Turn - we'd don't need to negate the turning
//
////            if(robot.foundationGrippers.getCommandedPosition() == FoundationGrippers.CommandedPosition.UP)
////            {
////                /*
////                 * Driving
////                 */
////                MecanumDrive.cartesian(robot.driveTrain,
////                        -gamepad1.left_stick_y*powerScalar, //Main
////                        gamepad1.left_stick_x*powerScalar, //Strafe
////                        gamepad1.right_stick_x*.75*powerScalar); //Turn - we'd don't need to negate the turning
////
////                acclCtrl.clr();
////            }
////            else
////            {
////                robot.driveTrain.setMotorPowers(acclCtrl.getAccelerationControlledPowers(
////                        MecanumDrive.calcCartesian(-gamepad1.left_stick_y*.75, gamepad1.left_stick_x*.75, gamepad1.right_stick_x*.75)));
////            }
//
//
//            /*
//             * Intake
//             */
//            if(gamepad1.b)
//            {
//                robot.intake.runEject();
//                intakeComingOutOfReverse = true;
//            }
//            else if(intakeForwardToggle.shouldToggle(gamepad1.a))
//            {
//                if(intakeOn)
//                {
//                    robot.intake.stop();
//                    intakeOn = false;
//
//                }
//                else
//                {
//                    robot.intake.runNormal();
//                    robot.crane.clawRelease();
//                    robot.booper.stow();
//                    intakeOn = true;
//                }
//            }
//            else
//            {
//                if(intakeComingOutOfReverse)
//                {
//                    intakeComingOutOfReverse = false;
//                    robot.intake.stop();
//                }
//            }
//
//            /*
//             * Booper
//             */
//            if(booperToggler.shouldToggle(gamepad2.start))
//            {
//                booperIsBooping = !booperIsBooping;
//
//                if(booperIsBooping)
//                {
//                    robot.booper.boop();
//                }
//                else
//                {
//                    robot.booper.stow();
//                }
//            }
//
//            /*
//             * Capstone
//             */
//            if(gamepad2.left_stick_button && gamepad2.right_stick_button)
//            {
//                robot.capstone.deploy();
//            }
//            else
//            {
//                robot.capstone.retain();
//            }
//
//            if(clawReleaseToggler.shouldToggle(gamepad2.x))
//            {
//                robot.crane.clawRelease();
//
//                if(robot.crane.getCommandedExtPos() != Crane.ExtPos.NORMAL)
//                {
//                    sleep(500);
//                    extShouldSet = false;
//                    robot.crane.setExtPosNormalRelease();
//                }
//            }
//
//            /*
//             * Extension
//             */
//            /*if(robot.crane.getCommandedRotatePosition() == Crane.RotatePosition.NORMAL)
//            {
//                if(gamepad2.dpad_up)
//                {
//                    robot.crane.setExtPosNormal();
//                }
//                else if(gamepad2.dpad_down)
//                {
//                    robot.crane.setExtPosFirst();
//                }
//            }*/
//
//            extController.run(extShouldSet);
//
//            if(gamepad2.dpad_up)
//            {
//                extShouldSet = false;
//                robot.crane.setExtPosNormal();
//            }
//            else if(gamepad2.dpad_down || extShouldSet)
//            {
//                extShouldSet = true;
//                robot.crane.custom(extController.getBimpAdjustedPostion());
//            }
//
//            /*
//             * Foundation grippers
//             */
//            if(gamepad2.a)
//            {
//                robot.foundationGrippers.release();
//            }
//            else if(gamepad2.b)
//            {
//                robot.foundationGrippers.grip();
//            }
//
//            if(rstToggle.shouldToggle(gamepad2.back))
//            {
//                System.out.println("Invoking zombie apocalypse");
//                new RunShellCommand().runAsRoot("echo zombie > /sys/kernel/debug/msm_dwc3/frog_reset");
//            }
//
//            /*
//             * Telemetry
//             */
//            telemetry.addData("Lift idx", liftControlStateMachine.getIdx());
//            telemetry.addData("Lift enc", robot.lift.liftMotor.getCurrentPosition());
//            telemetry.addData("Ext Bimp", extController.getBimp());
//            telemetry.update();
//        }
//    }
//
//    private void stoneRotationControls()
//    {
//        /*
//         * Stone Orientation
//         */
//        stoneOrientationVector.clear();
//        stoneOrientationVector.addCartesian(gamepad2.left_stick_x, -gamepad2.left_stick_y);
//
//        if(stoneOrientationVector.getMag() > .5)
//        {
//            double dir = stoneOrientationVector.getDir();
//
//            boolean zeroTo45 = dir < 45.0 && dir > 0.0;
//            boolean zeroTo315 = dir < 360.0 && dir > 315.0;
//            boolean exact = (dir == 0.0) | (dir == 360.0);
//            boolean right = zeroTo45 | zeroTo315 | exact;
//
//            boolean fortyFiveTo135 = dir > 45.0 && dir < 135.0;
//            boolean up = fortyFiveTo135;
//
//            boolean oneThirtyFiveTo225 = dir > 135.0 && dir < 225.0;
//            boolean left = oneThirtyFiveTo225;
//
//            boolean twoTwentyFiveTo315 = dir > 225.0 && dir < 315.0;
//            boolean down = twoTwentyFiveTo315;
//
//            if(down)
//            {
//                robot.crane.setRotatePos180();
//            }
//            else if(up)
//            {
//                robot.crane.setRotatePosNormal();
//            }
//            else if(right)
//            {
//                robot.crane.setRotatePos90();
//            }
//        }
//    }
//
//    @Override
//    protected void frog_init()
//    {
//        extController = new ExtController(robot, gamepad1, gamepad2);
//
//        acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.04, .04, 0);
//
//        liftControlStateMachine = new LiftControlStateMachine(robot, gyroUtils, gamepad1, gamepad2);
//        stoneKeeperStateMachine = new StoneKeeperStateMachine(robot, gyroUtils, gamepad1, gamepad2);
//
//        gamepad1.setJoystickDeadzone(0.2f);
//        gamepad2.setJoystickDeadzone(0.2f);
//
//        //robot.crane.setRotatePosNormal();
//
//        //robot.crane.setExtPosNormal();
//
//        robot.capstone.retain();
//        robot.crane.setRotatePosNormal();
//        robot.crane.setExtPosFirst();
//    }
//}
