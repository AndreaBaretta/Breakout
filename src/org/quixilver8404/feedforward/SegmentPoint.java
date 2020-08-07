package org.quixilver8404.feedforward;

import org.json.simple.JSONObject;
import org.quixilver8404.util.Config;

import java.util.List;

public class SegmentPoint implements VelocityPoint, HeadingPoint {

    public final int anchorIndex;
    public final double velP;
    public final String headingStateString;
    public final AnchorPoint.Heading headingState;
    public final double heading;
    public final int config;
    public final double tFromAnchor;

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

    public String toString() {
        return "(s=" + s + ", t=" + tFromAnchor + ", i=" + anchorIndex + ", velP=" + velP + ", headingState=" + headingStateString + ", heading=" + heading + ")";
    }
}
