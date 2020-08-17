package org.quixilver8404.feedforward;

import org.quixilver8404.controller.PowerProfile;
import org.quixilver8404.util.Config;
import org.quixilver8404.util.Vector3;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class ContinuousSegment {

    public final List<AnchorPoint> anchorPoints;
    public final List<SegmentPoint> segmentPoints;
    public final int index;
    public final double s0;
    public final double s1;
    public final List<MainSegment> mainSegments;
    public final List<ConnectionPoint> connectionPoints;
    public final List<HeadingSegment> headingSegments;
    public final List<ActionPoint> actionPoints;

    protected MainSegment currentMainSegment;
    protected HeadingSegment currentHeadingSegment;
    protected boolean finished;

    public ContinuousSegment(final List<AnchorPoint> anchorPointsList, final List<SegmentPoint> segmentPointsList, final double s0, final int index) {
        this.index = index;
        this.s0 = s0;
        this.anchorPoints = anchorPointsList;
        this.segmentPoints = segmentPointsList;
        mainSegments = new ArrayList<MainSegment>();
        connectionPoints = new ArrayList<ConnectionPoint>();
        headingSegments = new ArrayList<HeadingSegment>();
        actionPoints = new ArrayList<ActionPoint>();

        AnchorPoint curPoint = null;
        AnchorPoint prevPoint = curPoint;

        if (anchorPointsList.size() == 0) {
            throw new Error("Ya done goofed");
        }

        anchorPointsList.get(0).first = true;
        anchorPointsList.get(anchorPointsList.size() - 1).last = true;

        double s0_ = s0;
        double connectionPointCounter = 0;
        for (int i = 0; i < anchorPoints.size(); i++) {
            curPoint = anchorPoints.get(i);
            if (curPoint.first) {
                prevPoint = curPoint;
                connectionPoints.add(curPoint.middlePoint);
            } else {
                final ConnectionPoint connection0 = prevPoint.middlePoint;
                final ConnectionPoint connection1 = prevPoint.nextPoint;
                final ConnectionPoint connection2 = curPoint.prevPoint;
                final ConnectionPoint connection3 = curPoint.middlePoint;

//                System.out.println("Current connections: " + prevPoint.nextPoint.toString() + curPoint.prevPoint.toString() + curPoint.middlePoint.toString());

                connectionPoints.add(prevPoint.nextPoint);
                connectionPoints.add(curPoint.prevPoint);
                connectionPoints.add(curPoint.middlePoint);

                connection0.index = connectionPointCounter;
                connectionPointCounter++;
                connection1.index = connectionPointCounter;
                connectionPointCounter++;
                connection2.index = connectionPointCounter;
                connectionPointCounter++;
                connection3.index = connectionPointCounter;
                connectionPointCounter++;



//                System.out.println("toString: (" + curPoint.middlePoint.x/Config.INCHES_TO_METERS + ", " + curPoint.middlePoint.y/Config.INCHES_TO_METERS + ")");
                final double prevAnchorTheta = MainSegment.normalizeAlpha(Math.atan2(prevPoint.middlePoint.y-prevPoint.center1.y, prevPoint.middlePoint.x-prevPoint.center1.x));
                final double curAnchorTheta = MainSegment.normalizeAlpha(Math.atan2(curPoint.middlePoint.y-curPoint.center0.y, curPoint.middlePoint.x-curPoint.center0.x));

                final CircleSegment segment0 = new CircleSegment(connection0, connection1, s0_, prevPoint.configVelocity, prevPoint.center1, prevPoint.r1, prevAnchorTheta, prevPoint.theta1, prevPoint.counterClockwise1);
                s0_ = segment0.getEndS();
                final LinearSegment segment1 = new LinearSegment(connection1, connection2, s0_, prevPoint.configVelocity);
                s0_ = segment1.getEndS();
                final CircleSegment segment2 = new CircleSegment(connection2, connection3, s0_, prevPoint.configVelocity, curPoint.center0, curPoint.r0, curPoint.theta0, curAnchorTheta, curPoint.counterClockwise0);
                s0_ = segment2.getEndS();

                System.out.println("zeroSegment circle1: " + segment0.zeroSegment + " firstpoint: " + segment0.firstPoint.toString() + "  " + "lastpoint: " + segment0.lastPoint.toString() + "  counterclockwise: " + segment0.counterClockwise + "  center: " + segment0.center.toString() + " s0: " + segment0.s0 + " s1:" + segment0.getEndS());
                System.out.println("zeroSegment linear: " + segment1.zeroSegment + " firstpoint: " + segment1.firstPoint.toString() + "  " + "lastpoint: " + segment1.lastPoint.toString() + " s0: " + segment1.s0 + " s1:" + segment1.getEndS());
                System.out.println("zeroSegment circle2: " + segment2.zeroSegment + " firstpoint: " + segment2.firstPoint.toString() + "  " + "lastpoint: " + segment2.lastPoint.toString() + "  counterclockwise: " + segment2.counterClockwise + "  center: " + segment2.center.toString() + " s0: " + segment2.s0 + " s1:" + segment2.getEndS());
                System.out.println();

//                System.out.println(co);
                final List<SegmentPoint> curSegmentPoints = new ArrayList<SegmentPoint>();
                segmentPoints.forEach((final SegmentPoint p) -> {
                    if (p.anchorIndex == mainSegments.size()) {
                        curSegmentPoints.add(p);
                    }
                });
                curSegmentPoints.sort(new Comparator<SegmentPoint>() {
                    @Override
                    public int compare(final SegmentPoint t0, final SegmentPoint t1) {
                        if (t0.tFromAnchor - t1.tFromAnchor <= 0) {
                            return -1;
                        } else {
                            return 1;
                        }
                    }
                });
                System.out.println("curSegmentPoints: " + Arrays.toString(curSegmentPoints.toArray()));
                final MainSegment mainSegment = new MainSegment(segment0, segment1, segment2, i - 1, prevPoint, curPoint, curSegmentPoints);

                mainSegments.add(mainSegment);
//                System.out.println("Domain mainsegment: " + mainSegment.getEndS());
//                System.out.println("Domain circlesegment1: " + mainSegment.circleSegment1.getEndS());

                prevPoint = curPoint;
            }
        }

        s1 = s0_;

        currentMainSegment = mainSegments.get(0);
        finished = false;

        //        segmentPoints = parseSegmentPoints(foxtrotFile, config);
//
        final List<HeadingPoint> headingPoints = new ArrayList<HeadingPoint>();

        segmentPoints.forEach((final SegmentPoint p) -> {
            if (p.getActionEventListeners() != null) {
                if (!p.getActions().isEmpty()) {
                    actionPoints.add(p);
                }
            }
            if (p.headingState == AnchorPoint.Heading.CUSTOM) {
                headingPoints.add(p);
            } else if (p.headingState == AnchorPoint.Heading.FRONT) {
                headingPoints.add(new HeadingPoint() {
                    @Override
                    public AnchorPoint.Heading getHeadingState() {
                        return AnchorPoint.Heading.FRONT;
                    }

                    @Override
                    public double getHeading() {
                        return mainSegments.get(p.anchorIndex).getPosition(p.getS()).theta;
                    }

                    @Override
                    public double getS() {
                        return p.getS();
                    }
                });
            } else if (p.headingState == AnchorPoint.Heading.BACK) {
                headingPoints.add(new HeadingPoint() {
                    @Override
                    public AnchorPoint.Heading getHeadingState() {
                        return AnchorPoint.Heading.BACK;
                    }

                    @Override
                    public double getHeading() {
                        return MainSegment.normalizeAlpha(mainSegments.get(p.anchorIndex).getPosition(p.getS()).theta);
                    }

                    @Override
                    public double getS() {
                        return p.getS();
                    }
                });
            }
        });

        connectionPoints.forEach((final ConnectionPoint p) -> {
            if (p.getActionEventListeners() != null) {
                if (!p.getActions().isEmpty()) {
                    actionPoints.add(p);
                }
            }
            if (p.headingState != AnchorPoint.Heading.NONE) {
                headingPoints.add(p);
            }
        });

        headingPoints.sort(new Comparator<HeadingPoint>() {
            @Override
            public int compare(final HeadingPoint t0, final HeadingPoint t1) {
                if (t0.getS() - t1.getS() <= 0) {
                    return -1;
                } else {
                    return 1;
                }
            }
        });

        System.out.println("headingPoints: " + Arrays.toString(headingPoints.toArray()));

        actionPoints.sort(new Comparator<ActionPoint>() {
            @Override
            public int compare(final ActionPoint t0, final ActionPoint t1) {
                if (t0.getS() - t1.getS() <= 0) {
                    return -1;
                } else {
                    return 1;
                }
            }
        });


        HeadingPoint prevHeadingPoint = null;
        for (int i = 0; i < headingPoints.size(); i++) {
            if (i == 0) {
                prevHeadingPoint = headingPoints.get(i);
            } else {
                final HeadingPoint headingPoint = headingPoints.get(i);
                final HeadingSegment headingSegment = new HeadingSegment(prevHeadingPoint, headingPoint, i - 1);
                if (!headingSegment.zeroSegment) {
                    headingSegments.add(headingSegment);
                }
                prevHeadingPoint = headingPoint;
            }
        }

        currentHeadingSegment = headingSegments.get(0);

//        VelocityPoint prevVelocityPoint = null;
//        int velocitySegmentCounter = 0;
//        for (int i = 0; i < velocityPoints.size(); i++) {
//            if (i == 0) {
//                prevVelocityPoint = velocityPoints.get(i);
//            } else {
//                final VelocityPoint velocityPoint = velocityPoints.get(i);
//                final VelocitySegment velocitySegment = new VelocitySegment(prevVelocityPoint.getS(), velocityPoint.getS(), Math.min(velocityPoint.getMinVelocity(), prevVelocityPoint.getMinVelocity()), velocitySegmentCounter, prevVelocityPoint, velocityPoint);
//                if (!velocitySegment.zeroSegment) {
//                    velocitySegments.add(velocitySegment);
//                    velocitySegmentCounter++;
//                }
//                prevVelocityPoint = velocityPoint;
//            }
//        }
//
//        currentVelocitySegment = velocitySegments.get(0);

        System.out.println("End S: " + mainSegments.get(mainSegments.size() - 1).getEndS());

        System.out.println("Connection points: " + Arrays.toString(connectionPoints.toArray()));
        System.out.println("Segment points: " + Arrays.toString(segmentPoints.toArray()));
        System.out.println("Heading segments: " + Arrays.toString(headingSegments.toArray()));
        System.out.println();
//        System.out.println("Velocity points: " + Arrays.toString(velocityPoints.toArray()));
//        System.out.println("Velocity segments: " + Arrays.toString(velocitySegments.toArray()));
    }

    public RobotState evaluate(final double s, final double s_dot, final  double s_dot_dot) {
        if (!currentHeadingSegment.inRange(s)) {
            if (s < currentHeadingSegment.s0) {
//                preiousHeadingSegment();
            } else {
                nextHeadingSegment();
            }
        }
        final double endS = currentMainSegment.getEndS();
        if (finished) { //If at end, stay there
            return new RobotState(
                    currentHeadingSegment.calcAlpha(endS, currentMainSegment.getPosition(endS)),
                    currentHeadingSegment.calcAlphaDot(0, currentMainSegment.getVelocity(endS, 0)),
                    currentHeadingSegment.calcAlphaDotDot(0, currentMainSegment.getAcceleration(endS, 0, 0))
            );
        }
        if (currentMainSegment.index == mainSegments.size() - 1 && s >= endS - 0.001) { //Condition to finish
            finish();
            System.out.println("<---------------------------------------- Finished ---------------------------------------->");
            return new RobotState(
                    currentHeadingSegment.calcAlpha(endS, currentMainSegment.getPosition(endS)),
                    currentHeadingSegment.calcAlphaDot(0, currentMainSegment.getVelocity(endS, 0)),
                    currentHeadingSegment.calcAlphaDotDot(0, currentMainSegment.getAcceleration(endS, 0, 0))
            );
        } else if (s < s0) {
            return new RobotState(
                    currentHeadingSegment.calcAlpha(s0, currentMainSegment.getPosition(currentMainSegment.s0)),
                    currentHeadingSegment.calcAlphaDot(0, currentMainSegment.getVelocity(currentMainSegment.s0, s_dot)),
                    currentHeadingSegment.calcAlphaDotDot(0, currentMainSegment.getAcceleration(currentMainSegment.s0, s_dot, s_dot_dot))
            );
        } else { //Else, do the normal thing
//            System.out.println("Normal: " + currentMainSegment.getPosition(s).toString() + "  calcAlpha: " + currentHeadingSegment.calcAlpha(s, currentMainSegment.getPosition(s)));
//            System.out.println("Normal: " + currentMainSegment.getVelocity(s, s_dot).toString() + "  calcAlphaDot: " + currentHeadingSegment.calcAlphaDot(s_dot, currentMainSegment.getVelocity(s, s_dot)));
//            System.out.println("Normal: " + currentMainSegment.getAcceleration(s, s_dot, s_dot_dot).toString() + "  calcAlphaDotDot: " + currentHeadingSegment.calcAlphaDotDot(s_dot_dot, currentMainSegment.getAcceleration(s, s_dot, s_dot_dot)));

            return new RobotState(
                    currentHeadingSegment.calcAlpha(s, currentMainSegment.getPosition(s)),
                    currentHeadingSegment.calcAlphaDot(s_dot, currentMainSegment.getVelocity(s, s_dot)),
                    currentHeadingSegment.calcAlphaDotDot(s_dot_dot, currentMainSegment.getAcceleration(s, s_dot, s_dot_dot))
            );
        }
    }

    public double calcS(final double x, final double y) {
//        System.out.println("About to CalcS in Path");
        if (Double.isNaN(x)) System.out.println("Nan values");
        final double curSegmentS = currentMainSegment.calcS(x, y);
        if (curSegmentS >= currentMainSegment.getEndS()) {
            nextMainSegment();
            return currentMainSegment.calcS(x, y);
        } else if (curSegmentS < currentMainSegment.s0) {
//            previousMainSegment();
            return currentMainSegment.calcS(x, y);
        } else {
            return curSegmentS;
        }
    }

    protected void nextMainSegment() {
        if (currentMainSegment.index == mainSegments.size() - 1) {
            ;
        } else {
            System.out.println("Next segment");
            currentMainSegment = mainSegments.get(currentMainSegment.index + 1);
        }
    }

    protected void previousMainSegment() {
        if (currentMainSegment.index == 0) {
            ;
        } else {
            System.out.println("Previous segment");
            currentMainSegment = mainSegments.get(currentMainSegment.index - 1);
        }
    }

    protected void nextHeadingSegment() {
        if (currentHeadingSegment.index == headingSegments.size() - 1) {
            ;
        } else {
            System.out.println("Next HeadingSegment");
            currentHeadingSegment = headingSegments.get(currentHeadingSegment.index + 1);
        }
    }

    protected void previousHeadingSegment() {
        if (currentHeadingSegment.index == 0) {
            ;
        } else {
            System.out.println("Previous HeadingSegment");
            currentHeadingSegment = headingSegments.get(currentHeadingSegment.index - 1);
        }
    }

    protected void finish() {
        finished = true;
    }

    public boolean getFinished() {
        return finished;
    }

    public double calcAccelerationCorrection(final double s, final double s_dot) {
//        double s_dot_dot = Config.MAX_ACCELERATION;
        final MinorSegment.NextVCurVDistS nextVCurVDistS = getNextVelocity(s);
        System.out.println("Next velocity: " + nextVCurVDistS.nextV);
        final double d_s = nextVCurVDistS.s;
        final double v_f = nextVCurVDistS.nextV;
        final double accToVel = (1/d_s)*(0.5 * Math.pow(v_f - s_dot, 2) + s_dot * (v_f - s_dot));

//        System.out.println("Distance to next velocity: " + d_s);

        if (accToVel <= Config.MAX_SAFE_ACCELERATION*Config.MAX_DECELERATION) {
//            System.out.println("Return accToVel: " + accToVel);
            return accToVel;
        }

        final double maxAcc = findMaxPossibleAcc(s, s_dot);
        final double acc = Math.tan(Config.ACCELERATION_CORRECTION)*(nextVCurVDistS.curV - s_dot);
        System.out.println("maxAcc: " + maxAcc + "  acc: " + acc);
        if (acc > maxAcc) {
            System.out.println("Returned maxAcc: " + maxAcc);
            return maxAcc;
        } else if (acc < Config.MAX_DECELERATION) {
            System.out.println("Returned MAX_DECELERATION: " + Config.MAX_DECELERATION);
            return Config.MAX_DECELERATION;
        } else {
            System.out.println("delta v: "+ (nextVCurVDistS.curV - s_dot));
            System.out.println("Returned acc: " + acc + "  curV: " + nextVCurVDistS.curV);
            return acc;
        }
//        return maxAcc;

//        final MinorSegment.NextVCurVDistS nextVCurVDistS = currentMainSegment.getNextVelocity(s);


//        if (accToVel >= 0) {
//            final double maxPossibleAcc = findMaxPossibleAcc(s, s_dot);
//            if (accToVel >= Config.MAX_SAFE_ACCELERATION * maxPossibleAcc) {
//                return accToVel;
//            } else {
//                return maxPossibleAcc;
//            }
//        } else {
//            return
//        }
//
//
//        while (true) {
//            final RobotState state = evaluate(s, s_dot, s_dot_dot);
//            final Vector3 vel = state.vel;
//            final Vector3 acc = state.acc;
//            final double[] powerSettings = PowerProfile.toRawPowerSettings(acc, vel, state.pos.theta);
//            boolean optimized = true;
//            for (double p : powerSettings) {
//                if (Math.abs(p) >= 1) {
//                    optimized = false;
//                }
//            }
//            if (optimized) {
//                return s_dot_dot;
//            }
//            if (s_dot_dot < Config.MAX_DECELERATION) {
//                return 0;
//            }
//            s_dot_dot -= Config.ACCELERATION_CORRECTION_STEP;
//        }
    }

    public double findMaxPossibleAcc(final double s, final double s_dot) {
        double s_dot_dot = Config.MAX_ACCELERATION;
        while (true) {
            final RobotState state = evaluate(s, s_dot, s_dot_dot);
            final Vector3 vel = state.vel;
            final Vector3 acc = state.acc;
            final double[] powerSettings = PowerProfile.toRawPowerSettings(acc, vel, state.pos.theta);
            boolean optimized = true;
            for (double p : powerSettings) {
                if (Math.abs(p) >= 1) {
                    optimized = false;
                }
            }
            if (optimized) {
                return s_dot_dot;
            }
            if (s_dot_dot < Config.MAX_DECELERATION) {
                return 0;
            }
            s_dot_dot -= Config.ACCELERATION_CORRECTION_STEP;
        }
    }

    public MinorSegment.NextVCurVDistS getNextVelocity(final double s) {
        return currentMainSegment.getNextVelocity(s);
    }

}
