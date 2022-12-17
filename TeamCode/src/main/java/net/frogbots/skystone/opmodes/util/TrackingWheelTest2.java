package net.frogbots.skystone.opmodes.util;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.MovingStatistics;

import net.frogbots.skystone.control.PositionLogger;
import net.frogbots.skystone.control.TrackingWheelIntegrator;
import net.frogbots.skystone.control.TrackingWheelIntegratorArc;
import net.frogbots.skystone.control.TrackingWheelIntegratorRR_VC;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;
@Disabled
@TeleOp
public class TrackingWheelTest2 extends LinearOpMode
{
    MovingStatistics movingStatistics = new MovingStatistics(300);

    long startLoop = 0;

    TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

    TrackingWheelIntegratorRR_VC rr_vc = new TrackingWheelIntegratorRR_VC();

    PositionLogger positionLogger = new PositionLogger();

    @Override
    public void runOpMode() throws InterruptedException
    {
        LynxModule module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");

        LynxDcMotorController ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");

        ctrl.setMotorMode(0, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ctrl.setMotorMode(1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ctrl.setMotorMode(2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        ctrl.setMotorMode(0, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctrl.setMotorMode(1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctrl.setMotorMode(2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        startLoop = System.currentTimeMillis();

        while (!this.isStopRequested() && this.isStarted())
        {
            startLoop = System.currentTimeMillis();

            LynxModule.BulkData bulkData = module.getBulkData();

            int left = bulkData.getMotorCurrentPosition(0);
            int right = bulkData.getMotorCurrentPosition(1);
            int aux = bulkData.getMotorCurrentPosition(2);

            trackingWheelIntegrator.update(left, right, aux);
            rr_vc.update(left, right, aux);

            telemetry.addData("X", trackingWheelIntegrator.getX());
            telemetry.addData("Y", trackingWheelIntegrator.getY());
            telemetry.addData("R_X", rr_vc.getX());
            telemetry.addData("R_Y", rr_vc.getY());
            telemetry.addData("H", trackingWheelIntegrator.getHeading());
            telemetry.addData("L", left*-1);
            telemetry.addData("R", right*-1);
            telemetry.addData("Aux", aux);
            telemetry.addData("U", 1000/movingStatistics.getMean());

            telemetry.update();

            //positionLogger.add(trackingWheelIntegrator.getX(), trackingWheelIntegrator.getY());

            long delta = System.currentTimeMillis() - startLoop;
            movingStatistics.add(delta);
        }

        /*new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                positionLogger.dump();

                AppUtil.getInstance().showToast(UILocation.BOTH, "Dump done");
            }
        }).start();*/
    }
}
