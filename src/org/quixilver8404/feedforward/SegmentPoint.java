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

    public final double s;

    SegmentPoint(final JSONObject segmentPoint, final List<MainSegment> segments) {
        anchorIndex = ((Long)segmentPoint.get("anchorIndex")).intValue();
        velP = Math.min((double)segmentPoint.get("velP"), Config.MAX_SAFE_VELOCITY)*Config.MAX_VELOCITY;
        headingState = (String)segmentPoint.get("headingState");
        heading = (double)segmentPoint.get("heading");
        config = ((Long)segmentPoint.get("config")).intValue();
        tFromAnchor = (double)segmentPoint.get("tFromAnchor");
        final MainSegment segment = segments.get(anchorIndex);
        s = tFromAnchor*segment.getTotalS() + segment.s0;
    }

    public double getMinVelocity() {
        return velP;
    }

    public double getS() {
        return s;
    }

    public String toString() {
        return "(s=" + s + ", t=" + tFromAnchor + ", i=" + anchorIndex + ", velP=" + velP + ", headingState=" + headingState + ", heading=" + heading + ")";
    }
}
