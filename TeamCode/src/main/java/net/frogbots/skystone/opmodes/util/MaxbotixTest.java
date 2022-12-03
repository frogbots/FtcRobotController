package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.frogbots.skystone.drivers.MaxSonarI2CXL;

@TeleOp
public class MaxbotixTest extends LinearOpMode
{
            MaxSonarI2CXL RightSonar;
            MaxSonarI2CXL BackSonar;
            MaxSonarI2CXL LeftSonar;
            MaxSonarI2CXL FrontSonar;
            double RcmToInch = 0;
            double LcmToInch = 0;
            double BcmToInch = 0;
            double FcmToInch = 0;


@Override
public void runOpMode() throws InterruptedException
        {
        RightSonar = hardwareMap.get(MaxSonarI2CXL.class, "RightSonar");
        BackSonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");
        //FrontSonar = hardwareMap.get(MaxSonarI2CXL.class, "FrontDistance");
        LeftSonar = hardwareMap.get(MaxSonarI2CXL.class, "LeftSonar");

        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        while (opModeIsActive())
        {
            RcmToInch = RightSonar.getDistanceSync();
            BcmToInch = BackSonar.getDistanceSync();
            LcmToInch = LeftSonar.getDistanceSync();

            telemetry.addData("Right Dist inch", RcmToInch);
            telemetry.addData("Back Dist inch", BcmToInch);
            telemetry.addData("Left Dist inch", LcmToInch);
           telemetry.update();
       // FcmToInch = FrontSonar.getDistanceSync();
       // telemetry.addData("Front Dist inch", Finch+1);
        }
        }
        }
