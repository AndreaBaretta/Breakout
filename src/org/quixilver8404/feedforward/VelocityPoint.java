package org.quixilver8404.feedforward;

public interface VelocityPoint {
    public void setConfigVelocity(final double newConfigVelocity);

    public void setMinVelocity(final double newMinVelocity);

    public double getConfigVelocity();

    public double getMinVelocity();

    public double getS();
}
