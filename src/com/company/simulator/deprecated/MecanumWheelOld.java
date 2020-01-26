package com.company.simulator.deprecated;

public class MecanumWheelOld {
    protected final MotorOld motorOld;
    protected final double mmPerTick;
    public final double X;
    public final double Y;
    public final double R;
    public final double phi;
    protected double wheelVelocity;
    public MecanumWheelOld(final double X, final double Y, final double phi, final MotorOld motorOld, final double mmPerTick) {
        this.X = X;
        this.Y = Y;
        this.phi = phi;
        this.motorOld = motorOld;
        this.mmPerTick = mmPerTick;
        R = Math.hypot(X, Y);
    }

    public MecanumWheelOld(final double X, final double Y, final double phi, final double mu_delta, final double mu_fr, final double mmPerTick) {
        this.X = X;
        this.Y = Y;
        this.phi = phi;
        motorOld = new MotorOld(mu_fr, mu_delta, 0);
        this.mmPerTick = mmPerTick;
        R = Math.hypot(X, Y);
    }

    public void iterate(final double newPowerSetting, final double chassisVelocity, final double dt) {
        motorOld.update(newPowerSetting, chassisVelocity, dt);
        wheelVelocity = motorOld.getWheelVelocity()*mmPerTick;
    }

    public double getWheelVelocity() {
        return wheelVelocity;
    }
}
