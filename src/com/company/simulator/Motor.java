package com.company.simulator;

public class Motor {
    public final double maxTorque;
    public final double maxOmega;
    protected final LowPassFilter lowPassFilter;
    protected double wheelVelocityFraction;
    protected double currentPower;
    protected double torque;
    protected double torqueFraction;

    /**
     *This class creates a motor object.
     * @param maxTorque - Maximum torque of the motor in Newton meters.
     * @param maxRotationalSpeed - Maximum rotational velocity of the motor in radians per second.
     * */
    public Motor(final double maxTorque, final double maxRotationalSpeed, final LowPassFilter lowPassFilter) {
        this.maxTorque = maxTorque;
        this.maxOmega = maxRotationalSpeed;
        this.lowPassFilter = lowPassFilter;
        this.wheelVelocityFraction = 0;
        currentPower = 0;
        torque = 0;
    }

    /**
     * Calculate the torque applied by the motor to the wheel.
     * @param newPowerSetting - New power setting of the motor, from -1 to 1, proportional to voltage.
     * @param wheelVelocity - The angular velocity of the wheel, radians per second.
     * @param dt - Difference in time, in seconds.
     * */
    public void update(final double newPowerSetting, final double wheelVelocity, final double dt) {
        currentPower = -lowPassFilter.iterate(dt, newPowerSetting);
        wheelVelocityFraction = wheelVelocity/maxOmega;
        torqueFraction = (currentPower - wheelVelocityFraction);
        torque = maxTorque * torqueFraction;
    }

    /**
     * Low-pass filter to simulate the fact a motor will not be perfectly responsive.
     * */
    public static class LowPassFilter {
        public final double pole;
        public final double startState;
        protected double dxdt;
        protected double dx;
        protected double x;
        protected double u;
        public double get_x() { return x;}
        public double get_dxdt() { return dxdt; }
        public double get_u() { return u;}

        /**
         * Low-pass filter constructor.
         * @param pole - The frequency of the sine wave at which the motor becomes unstable.
         * @param startState - Let's just say it starts at 0 and leave it at that.
         * */
        public LowPassFilter(final double pole, final double startState) {
            this.pole = pole;
            this.startState = startState;
            x = startState;
        }

        /**
         * Iterate the filter.
         * @param dt - difference in time.
         * @param input - Input power setting.
         * */
        public double iterate(final double dt, final double input) {
            u = input;
            dxdt = -pole*x + u;
            dx = dxdt*dt;
            x += dx;
            return -pole*x;
        }

        /**
         * Returns an identical object.
         * */
        public LowPassFilter clone() { return new LowPassFilter(pole, startState); }
    }

    /**
     * Returns an identical object.
     * */
    public Motor clone() { return new Motor(maxTorque, maxOmega, lowPassFilter.clone()); }

    public double getCurrentPower() { return currentPower; }

    public double getTorque() { return torque; }
}
