package org.quixilver8404.breakout.feedforward;

import org.quixilver8404.breakout.util.Vector3;

import java.util.ArrayList;
import java.util.List;

public class MainSegment {
    public final CircleSegment circleSegment0;
    public final LinearSegment linearSegment;
    public final CircleSegment circleSegment1;

    public final int index;

    protected Segment currentSegment;

    public final AnchorPoint anchorPoint0;
    public final AnchorPoint anchorPoint1;
    public final double alpha0;
    public final double alpha1;
    public final AnchorPoint.Heading heading;
    public final boolean counterClockwise;
    public final double alpha0_;

    public final double s0;

    MainSegment(final CircleSegment circleSegment0, final LinearSegment linearSegment, final CircleSegment circleSegment1, final int index,
                final AnchorPoint anchorPoint0, final AnchorPoint anchorPoint1, final List<SegmentPoint> segmentPoints) {
        this.circleSegment0 = circleSegment0;
        this.linearSegment = linearSegment;
        this.circleSegment1 = circleSegment1;
        this.index = index;
//        currentSegment = circleSegment0;
        if (!circleSegment0.zeroSegment) {
            currentSegment = circleSegment0;
        } else if (!linearSegment.zeroSegment) {
            currentSegment = linearSegment;
        } else if (!circleSegment1.zeroSegment) {
            currentSegment = circleSegment1;
        } else {
            throw new Error("Zero mainsegment");
        }
        this.anchorPoint0 = anchorPoint0;
        this.anchorPoint1 = anchorPoint1;
        if (anchorPoint0.heading == AnchorPoint.Heading.FRONT) {
            alpha0 = circleSegment0.getPosition(circleSegment0.s0).theta;
        } else if (anchorPoint0.heading == AnchorPoint.Heading.BACK) {
            alpha0 = Vector3.normalizeAlpha(Math.PI + circleSegment0.getPosition(circleSegment0.s0).theta);
        } else {
            alpha0 = anchorPoint0.customHeading;
        }
        if (anchorPoint1.heading == AnchorPoint.Heading.FRONT) {
            alpha1 = circleSegment1.getPosition(circleSegment1.getEndS()).theta;
        } else if (anchorPoint1.heading == AnchorPoint.Heading.BACK) {
            alpha1 = Vector3.normalizeAlpha(Math.PI + circleSegment1.getPosition(circleSegment1.getEndS()).theta);
        } else {
            alpha1 = anchorPoint1.customHeading;
        }
        if (anchorPoint0.heading == AnchorPoint.Heading.FRONT && anchorPoint1.heading == AnchorPoint.Heading.FRONT) {
            heading = AnchorPoint.Heading.FRONT;
        } else if (anchorPoint0.heading == AnchorPoint.Heading.BACK && anchorPoint1.heading == AnchorPoint.Heading.BACK) {
            heading = AnchorPoint.Heading.BACK;
        } else {
            heading = AnchorPoint.Heading.CUSTOM;
        }
        if (alpha1 > alpha0) {
            if (alpha1 - alpha0 >= Math.PI) {
                counterClockwise = false; //TODO: Remove this useless variable
                alpha0_ = alpha0 + 2*Math.PI;
            } else {
                counterClockwise = true;
                alpha0_ = alpha0;
            }
        } else {
            if (alpha0 - alpha1 >= Math.PI) {
                counterClockwise = true;
                alpha0_ = alpha0 - 2*Math.PI;
            } else {
                counterClockwise = false;
                alpha0_ = alpha0;
            }
        }

        circleSegment1.firstPoint.setConfigVelocity(circleSegment0.firstPoint.configVelocity);

        final List<SegmentPoint> circle0SegmentPoints = new ArrayList<SegmentPoint>();
        final List<SegmentPoint> linearSegmentPoints = new ArrayList<SegmentPoint>();
        final List<SegmentPoint> circle1SegmentPoints = new ArrayList<SegmentPoint>();

        segmentPoints.forEach((final SegmentPoint p) -> {
            p.setS(circleSegment0.s0, getTotalS());
            System.out.println("segment point s: " + p.getS());
            if (circleSegment0.inRange(p.getS())) {
                circle0SegmentPoints.add(p);
            } else if (linearSegment.inRange(p.getS())) {
                linearSegmentPoints.add(p);
            } else {
                circle1SegmentPoints.add(p);
            }
        });

        s0 = circleSegment0.s0;
    }

    public double getTotalS() {
        return circleSegment1.getEndS() - s0;
    }

}
