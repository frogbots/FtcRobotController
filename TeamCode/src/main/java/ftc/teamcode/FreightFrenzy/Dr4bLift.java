package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.hardware.Servo;

public class Dr4bLift {

    public Servo servo1;
    public Servo servo2;

    public double lPos = 0.2;
    public double rPos = 0.81;

    public double coneLVL = .01;
    public double rconeLVL = 0.99;

    public double lowJunc = 0.54;
    public double rlowJunc = 0.47;

    public double midJunc = 0.68;
    public double rmidJunc = 0.33;

    public double highJunc = 0.28;
    public double rhighJunc =  0.738;

    public void ServoPos() {
        while (servo2.getPosition() < rPos) {
            servo2.setPosition(rPos);
            servo1.setPosition(lPos);
        }
    }

    public void ConeLVL1() {
        lPos = coneLVL;
        rPos = rconeLVL;
        while (servo2.getPosition() < rPos) {
            servo2.setPosition(rPos);
            servo1.setPosition(lPos);
        }
    }
    public void LowJunction() {
        lPos = lowJunc;
        rPos = rlowJunc;
        while (servo2.getPosition() < rPos) {
            servo2.setPosition(rPos);
            servo1.setPosition(lPos);
        }
    }
    public void MediumJunction() {
        lPos = midJunc;
        rPos = rmidJunc;
        while (servo2.getPosition() < rPos) {
            servo2.setPosition(rPos);
            servo1.setPosition(lPos);
        }
    }
    public void HighJunction() {
        lPos = highJunc;
        rPos = rhighJunc;
        while (servo2.getPosition() < rPos) {
            servo2.setPosition(rPos);
            servo1.setPosition(lPos);
        }
    }

}


