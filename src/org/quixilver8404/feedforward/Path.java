package org.quixilver8404.feedforward;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.quixilver8404.controller.PowerProfile;
import org.quixilver8404.util.Config;
import org.quixilver8404.util.Vector3;

import java.io.File;
import java.io.FileReader;
import java.util.*;


public class Path {
    public final List<ActionEventListener> configActionEventListeners;

    public final List<AnchorPoint> anchorPoints;
    public final List<ConnectionPoint> connectionPoints;
    public final List<SegmentPoint> segmentPoints;
    public final List<HeadingSegment> headingSegments;
    public final List<ActionPoint> actionPoints;
    public final List<Segment> segments;
    public final List<VelocitySegment> velocitySegments;

    protected Segment currentSegment;
    protected HeadingSegment currentHeadingSegment;
    protected VelocitySegment currentVelocitySegment;
    protected boolean finished;

    public Path(final File foxtrotFile, final int config, final List<ActionEventListener> configActionEventListeners) {
        this.configActionEventListeners = configActionEventListeners;
        anchorPoints = parseAnchorPoints(foxtrotFile, config);
        connectionPoints = new ArrayList<ConnectionPoint>();
        segmentPoints = parseSegmentPoints(foxtrotFile, config);
        actionPoints = new ArrayList<ActionPoint>();
        segments = new ArrayList<Segment>();
        velocitySegments = new ArrayList<VelocitySegment>();

        AnchorPoint curPoint = null;
        AnchorPoint prevPoint = curPoint;

        double s0 = 0;
        int connectionPointCounter = 0;
        int segmentCounter = 0;
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

                final CircleSegment segment0 = new CircleSegment(connection0, connection1, s0, segmentCounter, prevPoint.center1, prevPoint.r1, prevAnchorTheta, prevPoint.theta1, prevPoint.counterClockwise1);
                s0 = segment0.getEndS();
                if (!segment0.zeroSegment) { segmentCounter++; }
                final LinearSegment segment1 = new LinearSegment(connection1, connection2, s0, segmentCounter);
                s0 = segment1.getEndS();
                if (!segment1.zeroSegment) { segmentCounter++; }
                final CircleSegment segment2 = new CircleSegment(connection2, connection3, s0, segmentCounter, curPoint.center0, curPoint.r0, curPoint.theta0, curAnchorTheta, curPoint.counterClockwise0);
                s0 = segment2.getEndS();
                if (!segment2.zeroSegment) { segmentCounter++; }

                if (!segment0.zeroSegment) { segments.add(segment0); }
                if (!segment1.zeroSegment) { segments.add(segment1); }
                if (!segment2.zeroSegment) { segments.add(segment2); }

                System.out.println("zeroSegment circle1: " + segment0.zeroSegment + " firstpoint: " + segment0.firstPoint.toString() + "  " + "lastpoint: " + segment0.lastPoint.toString() + "  counterclockwise: " + segment0.counterClockwise + "  center: " + segment0.center.toString() + " s0: " + segment0.s0 + " s1:" + segment0.getEndS());
                System.out.println("zeroSegment linear: " + segment1.zeroSegment + " firstpoint: " + segment1.firstPoint.toString() + "  " + "lastpoint: " + segment1.lastPoint.toString() + " s0: " + segment1.s0 + " s1:" + segment1.getEndS());
                System.out.println("zeroSegment circle2: " + segment2.zeroSegment + " firstpoint: " + segment2.firstPoint.toString() + "  " + "lastpoint: " + segment2.lastPoint.toString() + "  counterclockwise: " + segment2.counterClockwise + "  center: " + segment2.center.toString() + " s0: " + segment2.s0 + " s1:" + segment2.getEndS());
                System.out.println("Test: s=0.8790616921214678 : " + segment2.getPosition(0.8790616921214678).toString());
                System.out.println("Test: s=1 : " + segment2.getPosition(1).toString());
                System.out.println();

                final double anchorIndex = i;
                segmentPoints.forEach(p -> {
                    if (p.anchorIndex == anchorIndex - 1) {
                        System.out.println("setS of segmentPoint");
                        p.setS(segment0.s0, segment2.getEndS() - segment0.s0);
                        if (p.getHeadingState() != AnchorPoint.Heading.CUSTOM && p.getHeadingState() != AnchorPoint.Heading.NONE) {
                            final double heading;
                            if (segment0.inRange(p.getS())) {
                                heading = segment0.getPosition(p.getS()).theta;
                            } else if (segment1.inRange(p.getS())) {
                                heading = segment1.getPosition(p.getS()).theta;
                            } else if (segment2.inRange(p.getS())) {
                                heading = segment2.getPosition(p.getS()).theta;
                            } else { throw new Error("Out of bounds error that REALLY REALLY shouldn't be happening"); }
                            if (p.getHeadingState() == AnchorPoint.Heading.FRONT) {
                                p.setHeading(heading);
                            } else {
                                p.setHeading(MainSegment.normalizeAlpha(heading + Math.PI));
                            }
                        }
                    }
                });

//                System.out.println(co);
//                final List<SegmentPoint> curSegmentPoints = new ArrayList<SegmentPoint>();
//                segmentPoints.forEach((final SegmentPoint p) -> {
//                    if (p.anchorIndex == mainSegments.size()) {
//                        curSegmentPoints.add(p);
//                    }
//                });
//                curSegmentPoints.sort(new Comparator<SegmentPoint>() {
//                    @Override
//                    public int compare(final SegmentPoint t0, final SegmentPoint t1) {
//                        if (t0.tFromAnchor - t1.tFromAnchor <= 0) {
//                            return -1;
//                        } else {
//                            return 1;
//                        }
//                    }
//                });
//                System.out.println("curSegmentPoints: " + Arrays.toString(curSegmentPoints.toArray()));
//                final MainSegment mainSegment = new MainSegment(segment0, segment1, segment2, i - 1, prevPoint, curPoint, curSegmentPoints);

//                mainSegments.add(mainSegment);
//                System.out.println("Domain mainsegment: " + mainSegment.getEndS());
//                System.out.println("Domain circlesegment1: " + mainSegment.circleSegment1.getEndS());

                prevPoint = curPoint;
            }
        }

//        connectionPoints.get(connectionPoints.size() - 1).setConfigVelocity(0);

        System.out.println("ConnectionPoints: " + Arrays.toString(connectionPoints.toArray()));
        System.out.println("Testing the thing: " + (connectionPoints.get(2).getS() == connectionPoints.get(3).getS()));

        currentSegment = segments.get(0);
        finished = false;

//        segmentPoints = parseSegmentPoints(foxtrotFile, config);
//
        final List<HeadingPoint> headingPoints = new ArrayList<HeadingPoint>();
        final List<VelocityPoint> velocityPoints_ = new ArrayList<VelocityPoint>();

        segmentPoints.forEach((final SegmentPoint p) -> {
            if (p.getActionEventListeners() != null) {
                if (!p.getActions().isEmpty()) {
                    actionPoints.add(p);
                }
            }
            if (p.getHeadingState() != AnchorPoint.Heading.NONE) {
                headingPoints.add(p);
            }
//            if (segments.size() != p.)
            velocityPoints_.add(p);
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
//            if (connectionPoints.size() != p.index + 1) {
//                if (connectionPoints.get(p.index + 1).getS() - p.getS() >= 1e-12) {
//                    velocityPoints_.add(p);
//                }
//            } else {
//                velocityPoints_.add(p);
//            }
            velocityPoints_.add(p);
        });

        System.out.println("velocityPoints_: " + Arrays.toString(velocityPoints_.toArray()));


        headingPoints.sort(new Comparator<HeadingPoint>() {
            @Override
            public int compare(final HeadingPoint t0, final HeadingPoint t1) {
                if (t0.getS() - t1.getS() < 0) {
                    return -1;
                } else if (t0.getS() - t1.getS() > 0) {
                    return 1;
                } else {
                    return 0;
                }
            }
        });

        velocityPoints_.sort(new Comparator<VelocityPoint>() {
            @Override
            public int compare(final VelocityPoint t0, final VelocityPoint t1) {
                if (t0.getS() - t1.getS() < 0) {
                    return -1;
                } else if (t0.getS() - t1.getS() > 0) {
                    return 1;
                } else {
                    return 0;
                }
            }
        });
        System.out.println("velocityPoints_ after sort: " + Arrays.toString(velocityPoints_.toArray()));

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

        //-------------------------------------------------------------Velocity

        for (int i = 0; i < velocityPoints_.size(); i++) {
            if (i != 0) {
                final VelocityPoint velocityPoint = velocityPoints_.get(i);
//                System.out.println("Cur velocity point: " + velocityPoint.toString());
                if (Double.isNaN(velocityPoint.getMaxVelocity())) {
                    velocityPoint.setMaxVelocity(velocityPoints_.get(i - 1).getMaxVelocity());
//                    System.out.println("Set max velocity");
                }
                if (Double.isNaN(velocityPoint.getConfigVelocity())) {
                    velocityPoint.setConfigVelocity(velocityPoints_.get(i - 1).getMaxVelocity());
//                    System.out.println("Set config velocity");
                }
            }
        }

        System.out.println("VelocityPoints_ after thing: " + Arrays.toString(velocityPoints_.toArray()));


        final List<VelocityPoint> velocityPoints = new ArrayList<>();
        for (int i = 0; i < velocityPoints_.size(); i++) {
            if (i != velocityPoints_.size() - 1) {
                if (velocityPoints_.get(i + 1).getS() - velocityPoints_.get(i).getS() >= 1e-12) {
                    velocityPoints.add(velocityPoints_.get(i));
                }
            } else {
                System.out.println("This is being done, right? I'm not crazy, RIGHT?Q??DWA?E?F?F?");
                System.out.println("Last point added: " + velocityPoints_.get(i).toString());
                velocityPoints.add(velocityPoints_.get(i));
            }
        }

        VelocityPoint prevVelocityPoint = null;
        for (int i = 0; i < velocityPoints.size(); i++) {
            if (i == 0) {
                prevVelocityPoint = velocityPoints.get(i);
            } else {
                final VelocityPoint velocityPoint = velocityPoints.get(i);
                final VelocitySegment velocitySegment = new VelocitySegment(prevVelocityPoint, velocityPoint, i - 1);
                velocitySegments.add(velocitySegment);
            }
        }

        currentVelocitySegment = velocitySegments.get(0);

        System.out.println("VelocityPoints_: " + Arrays.toString(velocityPoints_.toArray()));
        System.out.println("VelocityPoints: " + Arrays.toString(velocityPoints.toArray()));
        System.out.println("Velocity segments: " + Arrays.toString(velocitySegments.toArray()));

        //-------------------------------------------------------------Heading


        headingSegments = new ArrayList<HeadingSegment>();
        System.out.println("Headingsegment points: " + Arrays.toString(headingPoints.toArray()));
        HeadingPoint prevHeadingPoint = null;
        int headingSegmentIndex = 0;
        for (int i = 0; i < headingPoints.size(); i++) {
            if (i == 0) {
                prevHeadingPoint = headingPoints.get(i);
            } else {
                final HeadingPoint headingPoint = headingPoints.get(i);
                final HeadingSegment headingSegment = new HeadingSegment(prevHeadingPoint, headingPoint, headingSegmentIndex);
                if (!headingSegment.zeroSegment) {
                    headingSegments.add(headingSegment);
                    headingSegmentIndex++;
                }
                prevHeadingPoint = headingPoint;
            }
        }
        System.out.println("headingpoints length: " + headingPoints.size());
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

        System.out.println();

        System.out.println("End S: " + segments.get(segments.size() - 1).getEndS());

        connectionPoints.forEach(p -> System.out.println("Connection point: " + p.toString()));
        segmentPoints.forEach(p -> System.out.println("Segment point: " + p.toString()));
        headingPoints.forEach(p -> System.out.println("Heading point: " + p.toString()));
        headingSegments.forEach(s -> System.out.println("Heading segment: " + s.toString()));

        System.out.println("Segment points: " + Arrays.toString(segmentPoints.toArray()));
        System.out.println("Heading points: " + Arrays.toString(headingPoints.toArray()));
        System.out.println("Heading segments: " + Arrays.toString(headingSegments.toArray()));
//        System.out.println("Velocity segments: " + Arrays.toString(velocitySegments.toArray()));
//        System.out.println("Segments: " + Arrays.toString(segments.toArray()));
        System.out.println();
//        System.out.println("Velocity points: " + Arrays.toString(velocityPoints.toArray()));
//        System.out.println("Velocity segments: " + Arrays.toString(velocitySegments.toArray()));
//        System.out.println("ActionPoints: " + Arrays.toString(actionPoints.toArray()));
//        System.out.println("ActionEventListeners:");
//        actionPoints.forEach(p -> System.out.println(Arrays.toString(p.getActionEventListeners().toArray())));
    }

    public RobotState evaluate(final double s, final double s_dot, final  double s_dot_dot) {
//        System.out.println("index: " + currentSegment.index);

        while (true) {
            if (actionPoints.size() == 0) {
                break;
            }
            final ActionPoint actionPoint = actionPoints.get(0);
            if (s >= actionPoint.getS() - 1e-12) {
                if (actionPoint.getActionEventListeners() != null) {
                    actionPoint.runActions();
                }
                actionPoints.remove(0);
            } else {
                break;
            }
        }
        if (!currentHeadingSegment.inRange(s)) {
            if (s < currentHeadingSegment.s0) {
//                previousHeadingSegment();
            } else {
                nextHeadingSegment();
            }
        }
        final double endS = segments.get(segments.size() - 1).getEndS();

        if (finished) { //If at end, stay there
            return new RobotState(
                    currentHeadingSegment.calcAlpha(endS, currentSegment.getPosition(endS)),
                    currentHeadingSegment.calcAlphaDot(0, currentSegment.getVelocity(endS, 0)),
                    currentHeadingSegment.calcAlphaDotDot(0, currentSegment.getAcceleration(endS, 0, 0))
            );
        }
        if (currentSegment.index == segments.size() - 1 && s >= endS - 0.001) { //Condition to finish
            finish();
            System.out.println("<---------------------------------------- Finished ---------------------------------------->");
            return new RobotState(
                    currentHeadingSegment.calcAlpha(endS, currentSegment.getPosition(endS)),
                    currentHeadingSegment.calcAlphaDot(0, currentSegment.getVelocity(endS, 0)),
                    currentHeadingSegment.calcAlphaDotDot(0, currentSegment.getAcceleration(endS, 0, 0))
            );
        } else if (s < currentSegment.s0) {
            return new RobotState(
                    currentHeadingSegment.calcAlpha(0, currentSegment.getPosition(currentSegment.s0)),
                    currentHeadingSegment.calcAlphaDot(0, currentSegment.getVelocity(currentSegment.s0, s_dot)),
                    currentHeadingSegment.calcAlphaDotDot(0, currentSegment.getAcceleration(currentSegment.s0, s_dot, s_dot_dot))
            );
        } else { //Else, do the normal thing
//            System.out.println("Normal: " + currentMainSegment.getPosition(s).toString() + "  calcAlpha: " + currentHeadingSegment.calcAlpha(s, currentMainSegment.getPosition(s)));
//            System.out.println("Normal: " + currentMainSegment.getVelocity(s, s_dot).toString() + "  calcAlphaDot: " + currentHeadingSegment.calcAlphaDot(s_dot, currentMainSegment.getVelocity(s, s_dot)));
//            System.out.println("Normal: " + currentMainSegment.getAcceleration(s, s_dot, s_dot_dot).toString() + "  calcAlphaDotDot: " + currentHeadingSegment.calcAlphaDotDot(s_dot_dot, currentMainSegment.getAcceleration(s, s_dot, s_dot_dot)));
//            System.out.println("Normal thing");
//            System.out.println("s = " + s + " endS = " + endS + " index = " + currentSegment.index + " end index = " + (segments.size() - 1));
            return new RobotState(
                    currentHeadingSegment.calcAlpha(s, currentSegment.getPosition(s)),
                    currentHeadingSegment.calcAlphaDot(s_dot, currentSegment.getVelocity(s, s_dot)),
                    currentHeadingSegment.calcAlphaDotDot(s_dot_dot, currentSegment.getAcceleration(s, s_dot, s_dot_dot))
                );
        }
    }

    public double calcS(final double x, final double y) {
//        System.out.println("About to CalcS in Path");
        if (Double.isNaN(x)) System.out.println("Nan values");
        final double curSegmentS = currentSegment.calcS(x, y);
        final double s;
        if (curSegmentS >= currentSegment.getEndS()) {
//            System.out.println("Before nextSegment: CurSegmentS: " + curSegmentS + "  currentSegment.getEndS(): " + currentSegment.getEndS());
            nextSegment();
//            System.out.println("After  nextSegment: CurSegmentS: " + curSegmentS + "  currentSegment.getEndS(): " + currentSegment.getEndS());
            s = currentSegment.calcS(x, y);
        } else if (curSegmentS < currentSegment.s0) {
            //Do nothing
            s = currentSegment.calcS(x, y);
        } else {
            s = curSegmentS;
        }
        return s;
    }

    protected void nextSegment() {
        if (currentSegment.index == segments.size() - 1) {
            ;
        } else {
            currentSegment = segments.get(currentSegment.index + 1);
            System.out.println("Next segment: " + currentSegment.toString());
        }
    }

    protected void nextHeadingSegment() {
        if (currentHeadingSegment.index == headingSegments.size() - 1) {
            ;
        } else {
//            System.out.println("Next HeadingSegment");
            currentHeadingSegment = headingSegments.get(currentHeadingSegment.index + 1);
        }
    }

    protected void nextVelocitySegment() {
        if (currentVelocitySegment.index == velocitySegments.size() - 1) {
            ;
        } else {
//            System.out.println("Next VelocitySegment");
            currentVelocitySegment = velocitySegments.get(currentVelocitySegment.index + 1);
        }
    }

    protected void finish() {
//        System.out.println("Finished -------------------------------------------------------------------------------------------->");
        finished = true;
    }

    public double calcAccelerationCorrection(final double s, final double s_dot) {
//        double s_dot_dot = Config.MAX_ACCELERATION;
//        if (currentVelocitySegment.)

        if (s > currentVelocitySegment.s1) {
            nextVelocitySegment();
        }
//        System.out.println("currentVelocitySegment.index=" + currentVelocitySegment.index);
        final VelocitySegment.NextVCurVDistS nextVCurVDistS = getNextVelocity(s);

        if (nextVCurVDistS.nextV <= 1e-12 && currentVelocitySegment.index != velocitySegments.size() - 1 && currentVelocitySegment.s1 - s <= 1e-12) {
            currentVelocitySegment.p1.setConfigVelocity(Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY);
            currentVelocitySegment.p1.setMaxVelocity(Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY);
        }

        final double d_s = nextVCurVDistS.distS;
        final double v_f = nextVCurVDistS.nextV;
        final double accToVel = (1/d_s)*(0.5 * Math.pow(v_f - s_dot, 2) + s_dot * (v_f - s_dot));

//        System.out.println("Distance to next velocity: " + d_s);

        if (accToVel <= Config.MAX_SAFE_ACCELERATION*Config.MAX_DECELERATION) {
//            System.out.println("Return accToVel: " + accToVel);
            return accToVel;
        }

        final double maxAcc = findMaxPossibleAcc(s, s_dot);
        final double acc = Math.tan(Config.ACCELERATION_CORRECTION)*(nextVCurVDistS.curV - s_dot);
//        System.out.println("maxAcc: " + maxAcc + "  acc: " + acc);
        if (acc > maxAcc) {
//            System.out.println("Returned maxAcc: " + maxAcc);
            return maxAcc;
        } else if (acc < Config.MAX_DECELERATION) {
//            System.out.println("Returned MAX_DECELERATION: " + Config.MAX_DECELERATION);
            return Config.MAX_DECELERATION;
        } else {
//            System.out.println("Returned acc: " + acc + "  curV: " + nextVCurVDistS.curV);
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

    public VelocitySegment.NextVCurVDistS getNextVelocity(final double s) {
        return currentVelocitySegment.getNextVelocity(s);
    }

    public List<AnchorPoint> parseAnchorPoints(final File file, final int config) {
        final List<AnchorPoint> anchorPointsList = new ArrayList<AnchorPoint>();
        final JSONParser jsonParser = new JSONParser();
        try (final FileReader fileReader = new FileReader(file)) {
            final JSONObject obj = (JSONObject) jsonParser.parse(fileReader);
            final JSONArray anchorPoints = (JSONArray) obj.get("anchors");

            final JSONObject output = (JSONObject) obj.get("output");
            final JSONObject configObj = (JSONObject) output.get(Integer.toString(config));
            final JSONArray curves = (JSONArray) configObj.get("curves");

//            System.out.println("Read file");

            CurveParameters curParams = null;
            CurveParameters prevParams = null;

            for (int i = 0; i < anchorPoints.size(); i++) {
                final JSONObject anchorObj = (JSONObject) anchorPoints.get(i);

//                System.out.println("config: " + anchorObj.get("config") + "  type: " + anchorObj.get("config").getClass());
                if (((Long)anchorObj.get("config")).intValue() != config && ((Long)anchorObj.get("config")).intValue() != 0) {
                    continue;
                }

                final boolean first = i == 0;
                final boolean last = i == anchorPoints.size() - 1;

                final double x = ((double)anchorObj.get("x"))*Config.INCHES_TO_METERS;
                final double y = ((double)anchorObj.get("y"))*Config.INCHES_TO_METERS;
                final double tan = (double)anchorObj.get("tangent");
                final AnchorPoint.Heading heading;
                final double customHeading;
                if (((String) anchorObj.get("headingState")).equals("FRONT")) {
                    heading = AnchorPoint.Heading.FRONT;
                    customHeading = 0d;
                } else if (((String) anchorObj.get("headingState")).equals("BACK")) {
                    heading = AnchorPoint.Heading.BACK;
                    customHeading = 0d;
                } else {
                    heading = AnchorPoint.Heading.CUSTOM;
                    customHeading = (double)anchorObj.get("heading");
                }
                final double configVelocity = Math.min((double)anchorObj.get("velP"), Config.MAX_SAFE_VELOCITY)*Config.MAX_VELOCITY;

                final JSONArray actionJson = (JSONArray) anchorObj.get("actions");
                final Set<Integer> actions = new HashSet<Integer>();
                for (final Object actionObj : actionJson) {
                    actions.add((int) ((long) actionObj));
                }
                final List<ActionEventListener> actionEventListeners = new ArrayList<ActionEventListener>();
                if (!configActionEventListeners.isEmpty() && !actions.isEmpty()) {
                    for (final Integer action : actions) {
                        for (final ActionEventListener eventListener : configActionEventListeners) {
                            if (eventListener.action == action) {
                                actionEventListeners.add(eventListener);
                            }
                        }
                    }
                }

//                System.out.println("AnchorPoint Actions: " + Arrays.toString(actions.toArray()));;

                if (first) {
                    curParams = new CurveParameters((JSONObject)curves.get(0));
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, Double.NaN, null, Double.NaN,
                            curParams.circle1Radius, curParams.circle1Center, curParams.endTheta1, null, curParams.p1, configVelocity, actionEventListeners,
                            actions, first, last);
                    anchorPointsList.add(anchorPoint);
                    System.out.println("New AnchorPoint: " + anchorPoint.toString());
                    prevParams = curParams.copy();
                } else if (last) {
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, prevParams.circle2Radius, prevParams.circle2Center,
                            prevParams.endTheta2, Double.NaN, null, Double.NaN, prevParams.p2, null, configVelocity, actionEventListeners,
                            actions,first, last);
                    System.out.println("New AnchorPoint: " + anchorPoint.toString());
                    anchorPointsList.add(anchorPoint);
                } else {
                    curParams = new CurveParameters((JSONObject)curves.get(anchorPointsList.size()));
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, prevParams.circle2Radius, prevParams.circle2Center,
                            prevParams.endTheta2, curParams.circle1Radius, curParams.circle1Center, curParams.endTheta1, prevParams.p2, curParams.p1, configVelocity, actionEventListeners,
                            actions, first, last);
                    anchorPointsList.add(anchorPoint);
                    System.out.println("New AnchorPoint: " + anchorPoint.toString());
                    prevParams = curParams.copy();
                }
            }
            anchorPointsList.get(anchorPointsList.size() - 1).middlePoint.setConfigVelocity(0);
        } catch (final Exception e) {
            System.out.println("Failed to open " + file.getPath() + ". The file is not in the right format or is corrupted.");
            System.out.println(e);
            e.printStackTrace();
        }

        return anchorPointsList;
    }

    public List<SegmentPoint> parseSegmentPoints(final File file, final int config) {
        final List<SegmentPoint> segmentPoints = new ArrayList<SegmentPoint>();
        final JSONParser jsonParser = new JSONParser();
        try (final FileReader fileReader = new FileReader(file)) {
            final JSONObject obj = (JSONObject) jsonParser.parse(fileReader);
            final JSONArray segments = (JSONArray) obj.get("segments");
            for (int i = 0; i < segments.size(); i++) {
                final SegmentPoint segment = new SegmentPoint((JSONObject)segments.get(i), configActionEventListeners);
                if (segment.config == config || segment.config == 0) {
                    segmentPoints.add(segment);
                }
            }
        } catch (Exception e) {
            System.out.println("Failed to open " + file.getPath() + ". The file is not in the right format or is corrupted.");
            System.out.println(e);
            e.printStackTrace();
        }

        return segmentPoints;
    }
}
