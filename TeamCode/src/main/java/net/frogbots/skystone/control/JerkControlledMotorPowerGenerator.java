package net.frogbots.skystone.control;

public class JerkControlledMotorPowerGenerator
{
    private AcceleratedMotorPowerGeneratorForJerkController acceleratedMotorPowerGenerator;
    private double currentAcceleration = 0;
    private double jerkRate = 0;
    private double maxAccel;

    public JerkControlledMotorPowerGenerator(double jerkRate, double maxAccel, double minPow)
    {
        this.jerkRate = jerkRate;
        this.maxAccel = maxAccel;

        acceleratedMotorPowerGenerator = new AcceleratedMotorPowerGeneratorForJerkController(minPow);
    }

    public void setTargetPower(double targetPower)
    {
        acceleratedMotorPowerGenerator.setTargetPower(targetPower);
    }

    public void update()
    {
        if(acceleratedMotorPowerGenerator.getAccelerationControlledPower() < acceleratedMotorPowerGenerator.getTargetPower())
        {
            /*
             * Positive jerk
             */
            currentAcceleration += jerkRate;
            currentAcceleration = Math.min(currentAcceleration, maxAccel);

        }
        else if(acceleratedMotorPowerGenerator.getAccelerationControlledPower() > acceleratedMotorPowerGenerator.getTargetPower())
        {
            currentAcceleration -= jerkRate;
            currentAcceleration = Math.max(currentAcceleration, -maxAccel);
        }

        acceleratedMotorPowerGenerator.setAccelerationRate(currentAcceleration);
        acceleratedMotorPowerGenerator.update();
    }

    private class AcceleratedMotorPowerGeneratorForJerkController
    {
        private double accelerationRate;
        private double minPow;
        private double targetPower;
        private double currentPower;

        public AcceleratedMotorPowerGeneratorForJerkController(double minPow)
        {
            this.minPow = minPow;
        }

        public void setAccelerationRate(double accelerationRate)
        {
            this.accelerationRate = Math.abs(accelerationRate);
        }

        public void setTargetPower(double targetPower)
        {
            this.targetPower = targetPower;
        }

        public double getTargetPower()
        {
            return targetPower;
        }

        public void update()
        {
            if(currentPower > 0)
            {
                /*
                 * Accelerating
                 */
                if(currentPower < targetPower)
                {
                    currentPower += accelerationRate;
                    currentPower = Math.min(currentPower, targetPower);
                }

                /*
                 * Decelerating
                 */
                else if(currentPower > targetPower)
                {
                    currentPower -= accelerationRate;
                    currentPower = Math.max(currentPower, 0);
                }
            }
            else if (currentPower < 0)
            {
                /*
                 * Decelerating
                 */
                if(currentPower < targetPower)
                {
                    currentPower += accelerationRate;
                    currentPower = Math.min(currentPower, 0);
                }

                /*
                 * Accelerating
                 */
                else if(currentPower > targetPower)
                {
                    currentPower -= accelerationRate;
                    currentPower = Math.max(currentPower, targetPower);
                }
            }
            else
            {
                if(targetPower > 0.0)
                {
                    currentPower += Math.max(minPow, accelerationRate);
                }
                else if(targetPower < 0.0)
                {
                    currentPower -= Math.max(minPow, accelerationRate);
                }
            }
        }

        public double getAccelerationControlledPower()
        {
            return currentPower;
        }
    }

    public double getJerkControlledPower()
    {
        return acceleratedMotorPowerGenerator.getAccelerationControlledPower();
    }

}
