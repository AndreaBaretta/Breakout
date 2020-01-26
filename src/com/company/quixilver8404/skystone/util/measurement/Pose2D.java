package com.company.quixilver8404.skystone.util.measurement;

/**
 * Represents a 2D position with a direction.
 */
public class Pose2D extends Position2D {

    public final Angle heading;

    public Pose2D(Distance x, Distance y, Angle heading) {
        super(x, y);
        this.heading = heading;
    }
}
