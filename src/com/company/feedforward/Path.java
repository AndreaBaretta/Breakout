package com.company.feedforward;

import com.company.simulator.Vector3;

import java.util.ArrayList;
import java.util.List;

public class Path {
    public final List<AnchorPoint> anchorPoints;
    public final List<MainSegment> mainSegments;

    public Path(final List<AnchorPoint> anchorPoints) {
        this.anchorPoints = anchorPoints;
        this.mainSegments = new ArrayList<MainSegment>();

        AnchorPoint curPoint = null;
        AnchorPoint prevPoint = curPoint;

        double s0 = 0;
        for (int i = 0; i < anchorPoints.size(); i++) {
            System.out.println(i);
            curPoint = anchorPoints.get(i);
            if (curPoint.first) {
                prevPoint = curPoint;
            } else {
                System.out.println("ah");
                final ConnectionPoint connection0 = prevPoint.middlePoint;
                final ConnectionPoint connection1 = prevPoint.nextPoint;
                final ConnectionPoint connection2 = curPoint.prevPoint;
                final ConnectionPoint connection3 = curPoint.middlePoint;

                final double prevAnchorTheta = Math.atan2(prevPoint.middlePoint.y-prevPoint.center1.y, prevPoint.middlePoint.x-prevPoint.center1.x);
                final double curAnchorTheta = Math.atan2(curPoint.middlePoint.y-curPoint.center0.y, curPoint.middlePoint.x-curPoint.center0.x);

                final CircleSegment segment0 = new CircleSegment(connection0, connection1, s0, prevPoint.center1, prevPoint.r1, prevAnchorTheta, prevPoint.theta1, prevPoint.counterClockwise1);
                s0 = segment0.getEndS();
                final LinearSegment segment1 = new LinearSegment(connection1, connection2, s0);
                s0 = segment1.getEndS();
                final CircleSegment segment2 = new CircleSegment(connection2, connection3, s0, curPoint.center0, curPoint.r0, curPoint.theta0, curAnchorTheta, curPoint.counterClockwise0);
                s0 = segment2.getEndS();

                final MainSegment mainSegment = new MainSegment(segment0, segment1, segment2);

                mainSegments.add(mainSegment);

                prevPoint = curPoint;
            }
        }
    }

    public State evaluate(final double s, final double s_dot, final  double s_dot_dot) {
        for (final MainSegment segment : mainSegments) {
            if (segment.inRange(s)) {
                return new State(
                        segment.getPosition(s),
                        segment.getVelocity(s, s_dot),
                        segment.getAcceleration(s, s_dot, s_dot_dot)
                );
            }
        }
        System.out.println("End");
        final MainSegment lastSegment = mainSegments.get(mainSegments.size() - 1);
        return new State(
                lastSegment.getPosition(lastSegment.getEndS()),
                new Vector3(0,0,0),
                new Vector3(0,0,0)
        );
    }
}