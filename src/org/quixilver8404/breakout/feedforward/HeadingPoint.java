package org.quixilver8404.breakout.feedforward;

public interface HeadingPoint {
    public enum Heading {
        FRONT, BACK, CUSTOM, NONE
    }
    public Heading getHeadingState();

    public double getHeading();

    public double getS();
}
