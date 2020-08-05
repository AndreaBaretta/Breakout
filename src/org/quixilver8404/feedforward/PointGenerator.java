package org.quixilver8404.feedforward;

import java.util.List;

public class PointGenerator<E> {
    public final List<E> velocityPoints;
    protected int i = 0;
    public final int maxIndex;
    protected boolean finished = false;

    PointGenerator(final List<E> points) {
        this.velocityPoints = points;
        maxIndex = velocityPoints.size();
        if (maxIndex == 0) {
            finished = true;
        }
    }

    public E getNext() {
        System.out.println("Getting index: " + i);
        final E point = velocityPoints.get(i);
        i++;
        if (i == velocityPoints.size()) {
            finished = true;
        }
        return point;
    }

    public boolean getFinished() {
        return finished;
    }
}
