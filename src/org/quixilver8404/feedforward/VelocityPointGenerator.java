package org.quixilver8404.feedforward;

import java.util.List;

public class VelocityPointGenerator {
    public final List<VelocityPoint> velocityPoints;
    protected int i = 0;
    public final int maxIndex;
    VelocityPointGenerator(final List<VelocityPoint> velocityPoints) {
        this.velocityPoints = velocityPoints;
        maxIndex = velocityPoints.size();
    }

    public VelocityPoint getNext() {
        final VelocityPoint point = velocityPoints.get(i);
        if (i == velocityPoints.size() - 1) {
            i = 0;
        } else {
            i++;
        }
        return point;
    }
}
