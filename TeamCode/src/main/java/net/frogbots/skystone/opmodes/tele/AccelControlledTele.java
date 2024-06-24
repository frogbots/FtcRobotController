package net.frogbots.skystone.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.skystone.hardware.components.drivebase.MotorPowers;
import net.frogbots.skystone.hardware.components.drivebase.SkyStoneDriveBase;

import net.frogbots.skystone.control.AccelerationControlledDrivetrain;
@Disabled
@TeleOp
public class AccelControlledTele extends LinearOpMode
{
    SkyStoneDriveBase skyStoneDriveBase = new SkyStoneDriveBase();
    AccelerationControlledDrivetrain accelerationControlledDrivetrain;

    double LG_DOWN = 0.8357;
    double RG_DOWN = 0.2907;

    double LG_UP = 0.6892;
    double RG_UP = 0.4508;

    Servo leftGripper;
    Servo rightGripper;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftGripper = hardwareMap.servo.get("leftGripper");
        rightGripper = hardwareMap.servo.get("rightGripper");

        skyStoneDriveBase.init(hardwareMap);
        skyStoneDriveBase.enablePID();

        accelerationControlledDrivetrain = new AccelerationControlledDrivetrain(skyStoneDriveBase, .02, .02,.05);
        waitForStart();

        telemetry.setMsTransmissionInterval(20);


        while (opModeIsActive())
        {
            cartesianSimple(-gamepad1.left_stick_y, //Main

                    gamepad1.left_stick_x, //Strafe

                    gamepad1.right_stick_x); //Turn - we'd don't need to negate the turning);

            if(gamepad1.left_trigger > 0.5)
            {
                leftGripper.setPosition(LG_UP);
                rightGripper.setPosition(RG_UP);
            }
            else if(gamepad1.right_trigger > 0.5)
            {
                leftGripper.setPosition(LG_DOWN);
                rightGripper.setPosition(RG_DOWN);
            }
        }
    }

    public void cartesianSimple(double mainSpeed, double strafeSpeed, double turnSpeed)
    {
        MotorPowers motorPowers = new MotorPowers();

        double FL_power_raw;
        double FR_power_raw;
        double RL_power_raw;
        double RR_power_raw;

        //Run the holonomic formulas for each wheel
        FL_power_raw = mainSpeed + strafeSpeed + turnSpeed;
        FR_power_raw = mainSpeed - strafeSpeed - turnSpeed;
        RL_power_raw = mainSpeed - strafeSpeed + turnSpeed;
        RR_power_raw = mainSpeed + strafeSpeed - turnSpeed;

        /*
         * Are any of the computed wheel powers greater than 1?
         */
        if(Math.abs(FL_power_raw) > 1
                || Math.abs(FR_power_raw) > 1
                || Math.abs(RL_power_raw) > 1
                || Math.abs(RR_power_raw) > 1)
        {
            /*
             * Yeah, figure out which one
             */
            double maxLeft = Math.max(Math.abs(FL_power_raw), Math.abs(RL_power_raw));
            double maxRight = Math.max(Math.abs(FR_power_raw), Math.abs(RR_power_raw));
            double max = Math.max(maxLeft, maxRight);

            double ratio = 1 / max; //Create a ratio to normalize them all

            /*
             * Multiply each one by the ratio
             */
            motorPowers.frontLeft  = FL_power_raw * ratio;
            motorPowers.frontRight = FR_power_raw * ratio;
            motorPowers.rearLeft   = RL_power_raw * ratio;
            motorPowers.rearRight  = RR_power_raw * ratio;
        }

        /*
         * Nothing we need to do to the raw powers
         */
        else
        {
            motorPowers.frontLeft = FL_power_raw;
            motorPowers.frontRight = FR_power_raw;
            motorPowers.rearLeft = RL_power_raw;
            motorPowers.rearRight = RR_power_raw;
        }

        accelerationControlledDrivetrain.setMotorPowers(motorPowers.frontLeft, motorPowers.frontRight, motorPowers.rearLeft, motorPowers.rearRight);
    }
}
