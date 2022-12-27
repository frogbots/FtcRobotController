package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.hardware.Servo;

public class Dr4bLift {

    public Servo servo1;
    public Servo servo2;

    public double lPos = 0.2;
    public double rPos = 0.81;

    public double coneLVL = .18;
    public double rconeLVL = 0.838;

    public double lowJunc = 0.45;
    public double rlowJunc = 0.56;

    public double midJunc = 0.7;
    public double rmidJunc = 0.31;

    public double highJunc = 0.28;
    public double rhighJunc =  0.738;

    public double[] cone_levels_1 = {.18,.16,.12,.08,.02};
    public double[] cone_levels_2 = {.838,.858,.898,.938,.99};

    public void ServoPos() {
        //while (servo2.getPosition() < rPos) {
            servo2.setPosition(rPos);
            servo1.setPosition(lPos);
        //}
    }

    public void ConeLVL1() {
        lPos = coneLVL;
        rPos = rconeLVL;
        //while (servo2.getPosition() < rPos) {
            servo2.setPosition(rPos);
            servo1.setPosition(lPos);
        //}
    }
    public void LowJunction() {
        lPos = lowJunc;
        rPos = rlowJunc;
        //while (servo2.getPosition() < rPos) {
            servo2.setPosition(rPos);
            servo1.setPosition(lPos);
        //}
    }
    public void MediumJunction() {
        lPos = midJunc;
        rPos = rmidJunc;
        //while (servo2.getPosition() < rPos) {
            servo2.setPosition(rPos);
            servo1.setPosition(lPos);
        //}
    }
    public void HighJunction() {
        lPos = highJunc;
        rPos = rhighJunc;
        //while (servo2.getPosition() < rPos) {
            servo2.setPosition(rPos);
            servo1.setPosition(lPos);
        //}
    }

}


