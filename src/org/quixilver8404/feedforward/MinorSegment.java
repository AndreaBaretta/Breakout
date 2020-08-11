package org.quixilver8404.feedforward;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class MinorSegment extends Segment {
    public final ConnectionPoint firstPoint;
    public final ConnectionPoint lastPoint;
    public final List<VelocitySegment> velocitySegments;

    MinorSegment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint, final double s0, final double configVelocity) {
        super(firstPoint, lastPoint, s0, configVelocity);
        this.firstPoint = firstPoint;
        this.lastPoint = lastPoint;
        velocitySegments = new ArrayList<VelocitySegment>();
        System.out.println("At start: Firstpoint: " + firstPoint.toString() + "  Lastpoint: " + lastPoint.toString());
    }

    public void setPointSegment() {
        firstPoint.setNextSegment(this);
        lastPoint.setPrevSegment(this);
    }

    public void setVelocitySegments(final List<SegmentPoint> segmentPoints) {
        if (segmentPoints.size() != 0) {
            segmentPoints.forEach((final SegmentPoint p) -> {
                p.setMinVelocity(getMinVelocity());
            });
            VelocityPoint prevPoint = firstPoint;
            for (int i = 0; i < segmentPoints.size(); i++) {
                final VelocityPoint point = segmentPoints.get(i);
                final VelocitySegment segment = new VelocitySegment(prevPoint.getS(), point.getS(), getMinVelocity(), prevPoint, point);
                if (segment.zeroSegment) {
                    prevPoint.setConfigVelocity(point.getConfigVelocity());
                    prevPoint.setMinVelocity(point.getMinVelocity());
                }
                velocitySegments.add(segment);
                prevPoint = point;
            }
            final VelocitySegment finalSegment = new VelocitySegment(prevPoint.getS(), lastPoint.getS(), getMinVelocity(), prevPoint, lastPoint);
            velocitySegments.add(finalSegment);
            lastPoint.setConfigVelocity(prevPoint.getConfigVelocity());
        } else {
            final VelocitySegment segment = new VelocitySegment(firstPoint.getS(), lastPoint.getS(), getMinVelocity(), firstPoint, lastPoint);
            if (segment.zeroSegment) {
                firstPoint.setConfigVelocity(lastPoint.getConfigVelocity());
                firstPoint.setMinVelocity(lastPoint.getMinVelocity());
            }
            velocitySegments.add(segment);
        }
        System.out.print("Minor segment: ");
        System.out.print(Arrays.toString(velocitySegments.toArray()));
        System.out.println(" Segment minVelocity: " + getMinVelocity());
        System.out.println("Firstpoint: " + firstPoint.toString() + "  Lastpoint: " + lastPoint.toString());
        System.out.println("Segment points: " + Arrays.toString(segmentPoints.toArray()));
        System.out.println();
    }

    public abstract double getMinVelocity();

    public NextVCurVDistS getNextVelocity(final double s) {
        for (int i = 0; i < velocitySegments.size(); i++) {
            final VelocitySegment segment = velocitySegments.get(i);
            if (segment.inRange(s)) {
//                System.out.println();
//                System.out.println("Current VelocitySegment: " + segment.toString());
                return new NextVCurVDistS(segment.getNextVelocity(), segment.getSegmentVelocity(), segment.sToNextVelocity(s));
            }
        }
//        System.out.println("Should only happen at beginning -----------------------------------");
        return new NextVCurVDistS(velocitySegments.get(0).getNextVelocity(), velocitySegments.get(0).getSegmentVelocity(), velocitySegments.get(0).sToNextVelocity(s));
    }

    public class NextVCurVDistS {
        final double nextV;
        final double curV;
        final double s;
        public NextVCurVDistS(final double nextV, final double curV, final double s) {
            this.nextV = nextV;
            this.curV = curV;
            this.s = s;
        }
    }

}
