package org.quixilver8404.feedforward;

import org.json.simple.JSONObject;
import org.quixilver8404.util.Config;

import java.util.List;

public class SegmentPoint implements VelocityPoint {

    public final int anchorIndex;
    public final double velP;
    public final String headingState;
    public final double heading;
    public final int config;
    public final double tFromAnchor;

    protected double s;
    protected double minSegmentVelocity;

    SegmentPoint(final JSONObject segmentPoint) {
        anchorIndex = ((Long)segmentPoint.get("anchorIndex")).intValue();
        velP = Math.min((double)segmentPoint.get("velP"), Config.MAX_SAFE_VELOCITY)*Config.MAX_VELOCITY;
        headingState = (String)segmentPoint.get("headingState");
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

    public String toString() {
        return "(s=" + s + ", t=" + tFromAnchor + ", i=" + anchorIndex + ", velP=" + velP + ", headingState=" + headingState + ", heading=" + heading + ")";
    }
}
