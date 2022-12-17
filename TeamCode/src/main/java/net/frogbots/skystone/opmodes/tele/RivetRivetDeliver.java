//package net.frogbots.skystone.opmodes.tele;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.RunShellCommand;
//
//import net.frogbots.skystone.control.MecanumDrive;
//import net.frogbots.skystone.meta.misc.Toggler;
//import net.frogbots.skystone.meta.opmode.FrogOpMode;
//
//@TeleOp(name = "A - DELIVER RivetRivet")
//public class RivetRivetDeliver extends FrogOpMode
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
//    ExtController extController;
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
//            MecanumDrive.cartesian(robot.driveTrain,
//                    -gamepad1.left_stick_y*powerScalar, //Main
//                    gamepad1.left_stick_x*powerScalar, //Strafe
//                    gamepad1.right_stick_x*powerScalar*.75); //Turn - we'd don't need to negate the turning
//
//            /*
//             * Intake
//             */
//            if(gamepad1.b)
//            {
//                robot.intake.runEjectDeliver();
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
//                    robot.intake.runNormalDeliver();
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
//    @Override
//    protected void frog_init()
//    {
//
//        extController = new ExtController(robot, gamepad1, gamepad2);
//        liftControlStateMachine = new LiftControlStateMachine(robot, gyroUtils, gamepad1, gamepad2);
//
//        gamepad1.setJoystickDeadzone(0.2f);
//        gamepad2.setJoystickDeadzone(0.2f);
//
//        //robot.crane.setRotatePosNormal();
//
//        //robot.crane.setExtPosNormal();
//
//        robot.booper.boop();
//
//        robot.capstone.retain();
//        robot.crane.setRotatePosNormal();
//        robot.crane.setExtPosFirst();
//    }
//}
