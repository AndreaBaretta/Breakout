package com.company.feedforward;

import java.util.ArrayList;
import java.util.List;

public class Path {
    final List<AnchorPoint> anchorPoints;
    final List<MainSegment> mainSegments;

    Path(final List<AnchorPoint> anchorPoints) {
        this.anchorPoints = anchorPoints;
        this.mainSegments = new ArrayList<MainSegment>();

        AnchorPoint curPoint = null;
        AnchorPoint prevPoint = curPoint;

        double s0 = 0;
        for (int i = 0; i < anchorPoints.size(); i++) {
            curPoint = anchorPoints.get(i);
            if (curPoint.first) {
                prevPoint = curPoint;
            } else {
                final ConnectionPoint connection0 = prevPoint.middlePoint;
                final ConnectionPoint connection1 = prevPoint.nextPoint;
                final ConnectionPoint connection2 = curPoint.prevPoint;
                final ConnectionPoint connection3 = curPoint.middlePoint;

                final double prevAnchorTheta = Math.atan2(prevPoint.middlePoint.y-prevPoint.center1.y, prevPoint.middlePoint.x-prevPoint.center1.x);
                final double curAnchorTheta = Math.atan2(curPoint.middlePoint.y-curPoint.center1.y, curPoint.middlePoint.x-curPoint.center1.x);

                final CircleSegment segment0 = new CircleSegment(connection0, connection1, s0, prevPoint.center1, prevPoint.r1, prevPoint.theta1, prevAnchorTheta, prevPoint.counterClockwise1);
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

    }
}
