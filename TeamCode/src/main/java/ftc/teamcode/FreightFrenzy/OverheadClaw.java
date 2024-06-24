package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.hardware.Servo;

public class OverheadClaw {


    public Servo clawServo;

    public void ClawOpen() {
       if (clawServo.getPosition() <= .4)
          clawServo.setPosition(.4);
        }

        public void ClawClose() {
         if (clawServo.getPosition() >= -0.2)
           clawServo.setPosition(-0.2);
        }



    }



