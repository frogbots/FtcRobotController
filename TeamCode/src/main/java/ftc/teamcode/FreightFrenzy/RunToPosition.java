package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RunToPosition {

    public DcMotorEx liftMotor;

    public void RunToPosition() {
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RunToPos(int targetPos, double pow) {
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(pow);
    }












}
