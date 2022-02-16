package org.quixilver8404.breakout.feedforward;

public interface VelocityPoint {
    public void setConfigVelocity(final double newConfigVelocity);

    public void setMaxVelocity(final double newMinVelocity);

    public double getConfigVelocity();

    public double getMaxVelocity();

    public boolean isZeroVelocityPoint();

    public double getS();
}
