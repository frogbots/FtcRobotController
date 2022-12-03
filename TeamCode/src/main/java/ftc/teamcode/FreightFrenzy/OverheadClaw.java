package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.hardware.Servo;

public class OverheadClaw {


    public Servo clawServo;

    public void ClawOpen() {
        while (clawServo.getPosition() < .8)
          clawServo.setPosition(.8);
        }

        public void ClawClose() {
         while (clawServo.getPosition() > -0.2)
           clawServo.setPosition(-0.2);
        }



    }



