package net.frogbots.skystone.control;

import net.frogbots.skystone.hardware.components.drivebase.DriveTrain;

public class JerkControlledDrivetrain
{
    private DriveTrain driveTrain;
    private JerkControlledMotorPowerGenerator jerkFl, jerkFr, jerkRl, jerkRr;

    public JerkControlledDrivetrain(DriveTrain driveTrain)
    {
        this.driveTrain = driveTrain;

        jerkFl = new JerkControlledMotorPowerGenerator(.0002, .0035, .01);
        jerkFr = new JerkControlledMotorPowerGenerator(.0002, .0035, .01);
        jerkRl = new JerkControlledMotorPowerGenerator(.0002, .0035, .01);
        jerkRr = new JerkControlledMotorPowerGenerator(.0002, .0035, .01);
    }

    public void setMotorPowers(double fl, double fr, double rl, double rr)
    {
        jerkFl.setTargetPower(fl);
        jerkFr.setTargetPower(fr);
        jerkRl.setTargetPower(rl);
        jerkRr.setTargetPower(rr);

        jerkFl.update();
        jerkFr.update();
        jerkRl.update();
        jerkRr.update();

        double setFl = jerkFl.getJerkControlledPower();
        double setFr = jerkFr.getJerkControlledPower();
        double setRl = jerkRl.getJerkControlledPower();
        double setRr = jerkRr.getJerkControlledPower();

        driveTrain.setMotorPowers(setFl, setFr, setRl, setRr);
    }

    public double aPow()
    {
        return jerkFl.getJerkControlledPower();
    }
}
