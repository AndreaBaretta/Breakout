package org.quixilver8404.feedforward;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.quixilver8404.util.Config;

import java.util.*;

public class SegmentPoint implements VelocityPoint, HeadingPoint, ActionPoint {

    public final int anchorIndex;
    public final double velP;
    public final String headingStateString;
    public final AnchorPoint.Heading headingState;
    public final double heading;
    public final int config;
    public final double tFromAnchor;
    public final Set<Integer> actions;
    public final List<ActionEventListener> actionEventListeners;

    protected double s;
    protected double minSegmentVelocity;

    SegmentPoint(final JSONObject segmentPoint) {
        anchorIndex = ((Long)segmentPoint.get("anchorIndex")).intValue();
        velP = Math.min((double)segmentPoint.get("velP"), Config.MAX_SAFE_VELOCITY)*Config.MAX_VELOCITY;
        headingStateString = (String)segmentPoint.get("headingState");
        if (headingStateString.equals("FRONT")) {
            headingState = AnchorPoint.Heading.FRONT;
        } else if (headingStateString.equals("BACK")) {
            headingState = AnchorPoint.Heading.BACK;
        } else if (headingStateString.equals("CUSTOM")) {
            headingState = AnchorPoint.Heading.CUSTOM;
        } else {
            headingState = AnchorPoint.Heading.NONE;
        }
        heading = (double)segmentPoint.get("heading");
        config = ((Long)segmentPoint.get("config")).intValue();
        tFromAnchor = (double)segmentPoint.get("tFromAnchor");
        final JSONArray actionJson = (JSONArray) segmentPoint.get("actions");
        actions = new HashSet<Integer>();
        for (final Object actionObj : actionJson) {
            actions.add((int) ((long) actionObj));
        }
        actionEventListeners = new ArrayList<ActionEventListener>();
        if (!actionEventListeners.isEmpty() && !actions.isEmpty()) {
            for (final Integer action : actions) {
                for (final ActionEventListener eventListener : Config.actionEventListeners) {
                    if (eventListener.action == action) {
                        actionEventListeners.add(eventListener);
                    }
                }
            }
        }
    }

    public double getMinVelocity() {
        return minSegmentVelocity;
    }

    public double getConfigVelocity() {
        return velP;
    }

    public void setMinVelocity(final double minSegmentVelocity) {
        this.minSegmentVelocity = minSegmentVelocity;
    }

    public void setS(final double s0, final double totalS) {
        s = s0 + tFromAnchor*totalS;
    }

    public double getS() {
        return s;
    }

    public AnchorPoint.Heading getHeadingState() {
        return headingState;
    }

    public double getHeading() {
        return heading;
    }

    public Set<Integer> getActions() {
        return actions;
    }

    public List<ActionEventListener> getActionEventListeners() {
        return actionEventListeners;
    }

    public void runActions() {
        for (final ActionEventListener eventListener : actionEventListeners) {
            eventListener.run();
        }
    }

    public String toString() {
        return "(s=" + s + ", t=" + tFromAnchor + ", i=" + anchorIndex + ", velP=" + velP + ", headingState=" + headingStateString + ", heading=" + heading + ", actions=" + Arrays.toString(actions.toArray()) + ")";
    }
}
