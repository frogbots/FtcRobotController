package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.skystone.control.VariableSpeedServoController;
@Disabled
@TeleOp
public class VariableSpeedServoTest extends LinearOpMode
{
    Servo testServo;
    VariableSpeedServoController variableSpeedServoController;

    @Override
    public void runOpMode() throws InterruptedException
    {
        testServo = hardwareMap.servo.get("testServo");
        variableSpeedServoController = new VariableSpeedServoController(testServo);
        variableSpeedServoController.notifyCurrentPosition(.5);
        variableSpeedServoController.setIncrementRate(.02);

        testServo.setPosition(.5);

        waitForStart();

        while (opModeIsActive())
        {
            sleep(20);

            variableSpeedServoController.setPosition(gamepad1.left_stick_y);
            variableSpeedServoController.update();
        }
    }
}
