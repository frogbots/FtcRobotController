package net.frogbots.skystone.opmodes.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.MovingStatistics;

import net.frogbots.skystone.control.PositionLogger;
import net.frogbots.skystone.control.TrackingWheelIntegrator;
import net.frogbots.skystone.control.TrackingWheelIntegratorArc;
import net.frogbots.skystone.drivers.FrogBNO055;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

@TeleOp
public class TrackingWheelTest extends LinearOpMode
{
    MovingStatistics movingStatistics = new MovingStatistics(300);

    long startLoop = 0;

    TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

    TrackingWheelIntegratorArc trackingWheelIntegratorArc = new TrackingWheelIntegratorArc();

    PositionLogger positionLogger = new PositionLogger();

    FrogBNO055 bno055;

    @Override
    public void runOpMode() throws InterruptedException
    {
        LynxModule module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");

        LynxDcMotorController ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");

        bno055 = new FrogBNO055(hardwareMap.get(BNO055IMU.class, "external_IMU"));

        ctrl.setMotorMode(0, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ctrl.setMotorMode(1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ctrl.setMotorMode(2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        ctrl.setMotorMode(0, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctrl.setMotorMode(1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctrl.setMotorMode(2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.setMsTransmissionInterval(50);

        bno055.init();

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
            //trackingWheelIntegratorArc.update(left, right, aux, telemetry);

            telemetry.addData("X", trackingWheelIntegrator.getX());
            telemetry.addData("Y", trackingWheelIntegrator.getY());
            //telemetry.addData("A_X", trackingWheelIntegratorArc.getX());
            //telemetry.addData("A_Y", trackingWheelIntegratorArc.getY());
            telemetry.addData("wheelH", trackingWheelIntegrator.getHeading());
            telemetry.addData("L", left*-1);
            telemetry.addData("R", right*-1);
            telemetry.addData("Aux", aux);
            telemetry.addData("S", trackingWheelIntegrator.speed());
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
