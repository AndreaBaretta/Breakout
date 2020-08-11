package org.quixilver8404.feedforward;

import org.quixilver8404.util.Config;

import java.util.Arrays;
import java.util.List;
import java.util.Set;

public class ConnectionPoint extends Point2D implements VelocityPoint, HeadingPoint, ActionPoint {
    public MinorSegment prevSegment = null;
    public MinorSegment nextSegment = null;
    public boolean complete;
    protected double minVelocity;
    public double configVelocity;
    protected double s;
    public double index;
    public final AnchorPoint.Heading headingState;
    public final double heading;
    public final List<ActionEventListener> actionEventListeners;
    public final Set<Integer> actions;

    ConnectionPoint(final double x, final double y, final AnchorPoint.Heading headingState, final double heading, final double configVelocity,
                    final List<ActionEventListener> actionEventListeners, final Set<Integer> actions) {
        super(x, y);
        this.heading = heading;
        this.headingState = headingState;
        complete = false;
        minVelocity = configVelocity;
        this.configVelocity = configVelocity;
        this.actionEventListeners = actionEventListeners;
        this.actions = actions;
    }

    ConnectionPoint(final Point2D point, final AnchorPoint.Heading headingState, final double heading, final double configVelocity,
                    final List<ActionEventListener> actionEventListeners, final Set<Integer> actions) {
        super(point.x, point.y);
        this.heading = heading;
        this.headingState = headingState;
        complete = false;
        minVelocity = configVelocity;
        this.configVelocity = configVelocity;
        this.actionEventListeners = actionEventListeners;
        this.actions = actions;
    }

    public void setPrevSegment(final MinorSegment segment) {
        if (complete) {
            throw new Error("Error parsing: Connector point already complete when setting prevSegment at index: " + index);
        }
        prevSegment = segment;
        if (prevSegment != null && nextSegment != null) {
            complete = true;
        }
//        System.out.println("Set prevSegment at index: " + index + "  s = " + segment.getEndS());
        s = segment.getEndS();
    }

    public void setNextSegment(final MinorSegment segment) {
        if (complete) {
            throw new Error("Error parsing: Connector point already complete when setting nextSegment at index: " + index);
        }
        nextSegment = segment;
        if (prevSegment != null && nextSegment != null) {
            complete = true;
        }
        minVelocity = Math.min(minVelocity, nextSegment.getMinVelocity());
//        System.out.println("Set nextSegment at index: " + index + "  s = " + segment.s0);
        s = segment.s0;
    }

    public double getMinVelocity() {
        return minVelocity;
    }

    public double getConfigVelocity() {
        return configVelocity;
    }

    public void setConfigVelocity(final double newConfigVelocity) {
        configVelocity = newConfigVelocity;
    }

    public void setMinVelocity(final double newMinVelocity) {
        minVelocity = newMinVelocity;
    }

    public double getS() {
        return s;
    }

    public double getHeading() {
        return heading;
    }

    public AnchorPoint.Heading getHeadingState() {
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
        return "(" + x/Config.INCHES_TO_METERS + ", " + y/Config.INCHES_TO_METERS + ", s=" + s + ", configVelocity=" + configVelocity + ", minVelocity=" + minVelocity + ", actions=" + Arrays.toString(actionArray) + ")";
    }

}
