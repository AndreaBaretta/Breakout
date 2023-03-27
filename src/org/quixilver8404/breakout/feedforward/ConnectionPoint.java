package org.quixilver8404.breakout.feedforward;

import java.util.Arrays;
import java.util.List;
import java.util.Set;

public class ConnectionPoint extends Point2D
        implements VelocityPoint, HeadingPoint, ActionPoint {
    protected double configVelocity;
    protected double maxVelocity;
    protected double s;
    public int index;
    public final Heading headingState;
    public final double heading;
    public final List<ActionEventListener> actionEventListeners;
    public final Set<Integer> actions;
    public final boolean isOnAnchor;

    public ConnectionPoint(final double x, final double y,
                           final Heading headingState, final double heading,
                           final double configVelocity,
                           final List<ActionEventListener> actionEventListeners,
                           final Set<Integer> actions,
                           final boolean isOnAnchor) {
        super(x, y);
        this.heading = heading;
        this.headingState = headingState;
        this.configVelocity = configVelocity;
        maxVelocity = Double.NaN;
        this.actionEventListeners = actionEventListeners;
        this.actions = actions;
        this.isOnAnchor = isOnAnchor;
    }

    public ConnectionPoint(final Point2D point, final Heading headingState,
                           final double heading, final double configVelocity,
                           final List<ActionEventListener> actionEventListeners,
                           final Set<Integer> actions,
                           final boolean isOnAnchor) {
        super(point.x, point.y);
        this.heading = heading;
        this.headingState = headingState;
        this.configVelocity = configVelocity;
        maxVelocity = Double.NaN;
        this.actionEventListeners = actionEventListeners;
        this.actions = actions;
        this.isOnAnchor = isOnAnchor;
    }


    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getConfigVelocity() {
        return configVelocity;
    }

    public void setConfigVelocity(final double newConfigVelocity) {
        configVelocity = newConfigVelocity;
    }

    public void setMaxVelocity(final double newMaxVelocity) {
        if (Double.isNaN(maxVelocity)) {
            maxVelocity = newMaxVelocity;
        } else {
            maxVelocity = Math.min(newMaxVelocity, maxVelocity);
        }
    }

    public double getS() {
        return s;
    }

    public void setS(final double s) {
        this.s = s;
    }

    public double getHeading() {
        return heading;
    }

    public Heading getHeadingState() {
        return headingState;
    }

    public List<ActionEventListener> getActionEventListeners() {
        return actionEventListeners;
    }

    public Set<Integer> getActions() {
        return actions;
    }

    public void runActions() {
        for (final ActionEventListener eventListener : actionEventListeners) {
            eventListener.run();
        }
    }

    public String toString() {
        final Object[] actionArray;
        if (actions == null) {
            actionArray = new Integer[]{};
        } else {
            actionArray = actions.toArray();
        }
        return "(" + x + ", " + y + ", s=" + s + ", configVelocity=" +
                configVelocity + ", minVelocity=" + maxVelocity +
                ", headingState=" + getHeadingState() + ", heading=" +
                getHeading() + ", actions=" + Arrays.toString(actionArray) +
                (isOnAnchor ? ", isOnAnchor" : ", isConnection") + ")";
    }
}
