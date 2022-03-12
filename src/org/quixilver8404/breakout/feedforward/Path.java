package org.quixilver8404.breakout.feedforward;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.quixilver8404.breakout.controller.Breakout;
import org.quixilver8404.breakout.controller.PowerProfile;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.io.*;
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
    public final List<VelocityCurve> velocityCurves;

    protected Segment currentSegment;
    protected HeadingSegment currentHeadingSegment;
    protected VelocityCurve currentVelocityCurve;
    protected boolean finished;
    protected double lastKnownS;

    public final double startX;
    public final double startY;
    public final double startHeading;

    public Vector3 holdPos;

    public static Path fromFile(final File foxtrotFile, final int config, final List<ActionEventListener> configActionEventListeners) {
        try {
            System.out.println("File exists: " + (foxtrotFile!=null));
            return new Path(new FileInputStream(foxtrotFile), config, configActionEventListeners);
        } catch (final FileNotFoundException e) {
            System.out.println("EEEEEEEEEEEEEEEEEEEEEEEEEEE");
            return null;
        }
    }

    public Path(final InputStream foxtrotFileReader, final int config, final List<ActionEventListener> configActionEventListeners) {
        this.configActionEventListeners = configActionEventListeners;
        final JSONObject obj = parseJSON(foxtrotFileReader);
        anchorPoints = parseAnchorPoints(obj, config);
        connectionPoints = new ArrayList<ConnectionPoint>();
        segmentPoints = parseSegmentPoints(obj, config);
        actionPoints = new ArrayList<ActionPoint>();
        segments = new ArrayList<Segment>();
        velocitySegments = new ArrayList<VelocitySegment>();
        velocityCurves = new ArrayList<VelocityCurve>();
        lastKnownS = 0;

        AnchorPoint curPoint = null;
        AnchorPoint prevPoint = curPoint;

        double s0 = 0;
        int connectionPointCounter = 0;
        int segmentCounter = 0;
        int velocitySegmentCounter = 0;
        for (int i = 0; i < anchorPoints.size(); i++) {
            curPoint = anchorPoints.get(i);
            if (curPoint.first) {
                prevPoint = curPoint;
                curPoint.middlePoint.setMaxVelocity(Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY);
                connectionPoints.add(curPoint.middlePoint);
            } else {
                final ConnectionPoint connection0 = prevPoint.middlePoint;
                final ConnectionPoint connection1 = prevPoint.nextPoint;
                final ConnectionPoint connection2 = curPoint.prevPoint;
                final ConnectionPoint connection3 = curPoint.middlePoint;

                final double prevAnchorTheta = Vector3.normalizeAlpha(Math.atan2(prevPoint.middlePoint.y-prevPoint.center1.y, prevPoint.middlePoint.x-prevPoint.center1.x));
                final double curAnchorTheta = Vector3.normalizeAlpha(Math.atan2(curPoint.middlePoint.y-curPoint.center0.y, curPoint.middlePoint.x-curPoint.center0.x));

                final CircleSegment segment0 = new CircleSegment(connection0, connection1, s0, prevPoint.center1, prevPoint.r1, prevAnchorTheta, prevPoint.theta1, prevPoint.counterClockwise1);
                s0 = segment0.getEndS();
                final LinearSegment segment1 = new LinearSegment(connection1, connection2, s0);
                s0 = segment1.getEndS();
                final CircleSegment segment2 = new CircleSegment(connection2, connection3, s0, curPoint.center0, curPoint.r0, curPoint.theta0, curAnchorTheta, curPoint.counterClockwise0);
                s0 = segment2.getEndS();

                System.out.println("circle segment 0: " + segment0);
                System.out.println("linear segment 1: " + segment1);
                System.out.println("circle segment 2: " + segment2);

                ConnectionPoint p0 = connection0;
                ConnectionPoint p1 = connection1;

                if (!segment0.zeroSegment) { // Normal scenario
                    segment0.configurePoints(p0, p1);

                    System.out.println("connection point 0: " + connection0.toString()); //Only print after configuring it

                    p1.index = connectionPointCounter;
                    connectionPointCounter++;
                    connectionPoints.add(p1);
                    System.out.println("Added connection point: " + p1);

                    segments.add(segment0);
                    segment0.setIndex(segmentCounter);
                    segmentCounter++;
                    p0 = connection1;
                } else if (segment0.getMaxVelocity() <= 1e-10) { // Zero velocity, zero segment --> Skip, but make sure velocity is configured
                    segment0.configurePoints(p0, p1);

                    System.out.println("connection point 0: " + connection0.toString()); //Only print after configuring it
                } else { // If radius is not 0 but length is, then path is contiguous
                    System.out.println("connection point 0: " + connection0.toString()); //Only print after configuring it
                }

                if (!segment2.zeroSegment) {
                    p1 = connection2;
                } else {
                    p1 = connection3;
                }

                if (!segment1.zeroSegment) { // Normal scenario
                    segment1.configurePoints(p0, p1);

                    p1.index = connectionPointCounter;
                    connectionPointCounter++;
                    connectionPoints.add(p1);
                    System.out.println("Added connection point: " + p1);

                    segments.add(segment1);
                    segment1.setIndex(segmentCounter);
                    segmentCounter++;
                    p0 = p1;
                    p1 = connection3;
                } else { // Line segment can never be 0 velocity, so I only consider the case when it is of length 0.
                    p1 = connection3;
                }

                if (!segment2.zeroSegment) { // Normal scenario
                    segment2.configurePoints(p0, p1);

                    p1.index = connectionPointCounter;
                    connectionPointCounter++;
                    connectionPoints.add(p1);
                    System.out.println("Added connection point: " + p1);

                    segments.add(segment2);
                    segment2.setIndex(segmentCounter);
                    segmentCounter++;
                } else if (segment2.getMaxVelocity() <= 1e-10) { // Zero velocity, zero segment --> Skip, but make sure velocity is configured
                    segment2.configurePoints(p0, p1);
                    System.out.println("seg2 config v on p1: " + p1);
                } // If radius is not 0 but length is, then path is contiguous, so no need to configure velocities

                final List<SegmentPoint> currentSegPoints = new ArrayList<>();

                final double anchorIndex = i;
                segmentPoints.forEach(p -> {
                    if (p.anchorIndex == anchorIndex - 1) {
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
                                p.setHeading(Vector3.normalizeAlpha(heading + Math.PI));
                            }
                        }

                        currentSegPoints.add(p);
                    }
                });

                currentSegPoints.sort(new Comparator<SegmentPoint>() {
                    @Override
                    public int compare(final SegmentPoint t0, final SegmentPoint t1) {
                        if (t0.getS() - t1.getS() < 0) {
                            return -1;
                        } else if (t0.getS() - t1.getS() > 0) {
                            return 1;
                        } else {
                            return 0;
                        }
                    }
                });

                for (final SegmentPoint segPoint : currentSegPoints) {
                    if (segment0.inRange(segPoint.getS())) {
                        segPoint.setMaxVelocity(segment0.getMaxVelocity());
                    } else if (segment1.inRange(segPoint.getS())) {
                        segPoint.setMaxVelocity(segment1.getMaxVelocity());
                    } else {
                        segPoint.setMaxVelocity(segment2.getMaxVelocity());
                    }
                }

                System.out.println("Current segmentPoints");
                currentSegPoints.forEach(p -> System.out.println(p.toString()));
                System.out.println();

                VelocityPoint lastPoint = connection0;
                VelocitySegment curVSegment = null;
                for (final VelocityPoint velocityPoint : currentSegPoints) {
                    curVSegment = new VelocitySegment(lastPoint, velocityPoint, connection1, connection2, velocitySegmentCounter);
                    velocitySegments.add(curVSegment);
                    velocitySegmentCounter++;
                    lastPoint = velocityPoint;
                }
                curVSegment = new VelocitySegment(lastPoint, connection3, connection1, connection2, velocitySegmentCounter);
                velocitySegments.add(curVSegment);
                velocitySegmentCounter++;

                prevPoint = curPoint;

                System.out.println();
            }
        }

        velocitySegments.forEach(s -> s.set());
        int[] velocityCurveCounter = new int[]{0};
        velocitySegments.forEach(s -> s.velocityCurves.forEach(c -> {
            c.setIndex(velocityCurveCounter[0]);
            velocityCurves.add(c);
            velocityCurveCounter[0]++;
        }));

        currentSegment = segments.get(0);
        finished = false;

        final List<HeadingPoint> headingPoints = new ArrayList<HeadingPoint>();

        segmentPoints.forEach((final SegmentPoint p) -> {
            if (p.getActionEventListeners() != null) {
                if (!p.getActions().isEmpty()) {
                    actionPoints.add(p);
                }
            }
            if (p.getHeadingState() != AnchorPoint.Heading.NONE) {
                headingPoints.add(p);
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
                if (t0.getS() - t1.getS() < 0) {
                    return -1;
                } else if (t0.getS() - t1.getS() > 0) {
                    return 1;
                } else {
                    return 0;
                }
            }
        });

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

        currentVelocityCurve = velocityCurves.get(0);

        headingSegments = new ArrayList<HeadingSegment>();
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
        currentHeadingSegment = headingSegments.get(0);

        final AnchorPoint firstPoint = anchorPoints.get(0);
        startX = firstPoint.x;
        startY = firstPoint.y;
        startHeading = currentHeadingSegment.calcAlpha(0, currentSegment.getPosition(0)).theta;

        holdPos = new Vector3(startX, startY, startHeading);

        System.out.println();

        System.out.println("End S: " + segments.get(segments.size() - 1).getEndS());

        System.out.println("Segments");
        segments.forEach(p -> System.out.println(p.toString()));
//        velocitySegments.forEach(p -> System.out.println("Velocity segment: " + p.toString()));

//        connectionPoints.forEach(p -> System.out.println("Connection point: " + p.toString()));
        System.out.println("Segment points");
        segmentPoints.forEach(p -> System.out.println(p.toString()));
        System.out.println();
        headingPoints.forEach(p -> System.out.println("Heading point: " + p.toString()));
        headingSegments.forEach(s -> System.out.println("Heading segment: " + s.toString()));

//        System.out.println("Segments");
//        segments.forEach(p -> System.out.println(p.toString()));

        System.out.println("Velocity curves");
        velocityCurves.forEach(s -> System.out.println(s.toString()));
    }

    public RobotState evaluate(final double s, final double s_dot, final  double s_dot_dot) {
        while (true) {
            if (actionPoints.size() == 0) {
                break;
            }
            final ActionPoint actionPoint = actionPoints.get(0);
            if (s >= actionPoint.getS() - 1e-12) {
                if (actionPoint.getActionEventListeners() != null) {
//                    System.out.println("About to run an action");
                    final Vector3 holdPos_ = currentHeadingSegment.calcAlpha(s, currentSegment.getPosition(actionPoint.getS()));
//                  holdPos = new Vector3(holdPos_.x, holdPos_.y, holdPos_.theta + Math.PI/2);
                    holdPos = holdPos_;
                    System.out.println("Running an action: " + actionPoint.toString());
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
//            throw new Error("Yo");
            return new RobotState(
                    currentHeadingSegment.calcAlpha(endS, currentSegment.getPosition(endS)),
                    currentHeadingSegment.calcAlphaDot(0, currentSegment.getVelocity(endS, 0)),
                    currentHeadingSegment.calcAlphaDotDot(0, currentSegment.getAcceleration(endS, 0, 0))
            );
        }
        if (s >= endS) { //Condition to finish
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
            return new RobotState(
                    currentHeadingSegment.calcAlpha(s, currentSegment.getPosition(s)),
                    currentHeadingSegment.calcAlphaDot(s_dot, currentSegment.getVelocity(s, s_dot)),
                    currentHeadingSegment.calcAlphaDotDot(s_dot_dot, currentSegment.getAcceleration(s, s_dot, s_dot_dot))
            );
        }
    }

    public double calcS(final double x, final double y) {
        if (Double.isNaN(x)) System.out.println("Nan values");
        final double curSegmentS = currentSegment.calcS(x, y);
        final double s;
        if (curSegmentS >= currentSegment.getEndS()) {
            nextSegment();
            s = currentSegment.calcS(x, y);
        } else if (curSegmentS < currentSegment.s0) {
            //Do nothing
            s = currentSegment.calcS(x, y);
        } else {
            s = curSegmentS;
        }
        lastKnownS = s;
        return s;
    }

    protected void nextSegment() {
        if (currentSegment.index == segments.size() - 1) {
            ;
        } else {
            currentSegment = segments.get(currentSegment.index + 1);
        }
    }

    protected void nextHeadingSegment() {
        if (currentHeadingSegment.index == headingSegments.size() - 1) {
            ;
        } else {
            currentHeadingSegment = headingSegments.get(currentHeadingSegment.index + 1);
        }
    }

    protected void nextVelocitySegment() {
        if (currentVelocityCurve.getIndex() == velocityCurves.size() - 1) {
            ;
        } else {
            currentVelocityCurve = velocityCurves.get(currentVelocityCurve.getIndex() + 1);
            System.out.println("New vel curve: " + currentVelocityCurve.toString());
        }
    }

    protected void finish() {
        finished = true;
    }

    public boolean isFinished() {
        return finished;
    }

    public double getLastKnownS() {
        return lastKnownS;
    }

    public double[] calcAccelerationCorrection(final double s_, final double s_dot) {

        final double s = Math.max(s_, currentVelocityCurve.s0);

//        if (currentVelocitySegment.distS(s) <= 0.01 && currentVelocitySegment.index != velocitySegments.size() - 1 && currentVelocitySegment.v1 == 0) {
//            currentVelocitySegment.p1.setConfigVelocity(Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY);
//            currentVelocitySegment.p1.setMaxVelocity(Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY);
////            System.out.println("Crazy bullshit: " + currentVelocitySegment.toString());
//        }

//        System.out.println("s=" + s);

        if (currentVelocityCurve.v1 <= 1e-3 && s_dot <= 1e-3) {
//            System.out.println("New velocity segment");
            nextVelocitySegment();
        }

        if (currentVelocityCurve.s1 < s) {
//            System.out.println("New velocity segment");
            nextVelocitySegment();
        }

        final double targetVelocity = currentVelocityCurve.getVelocity(s);

        final double targetAcc = currentVelocityCurve.acceleration;

        final double acc = Math.tan(Config.ACCELERATION_CORRECTION)*(targetVelocity - s_dot) + targetAcc;

        final double maxAcc = findMaxPossibleAcc(s, s_dot);

//        System.out.println("targetVel: " + targetVelocity + "  accToVel: " + accToVel + "  overall acc: " + acc + "  distS: " + nextVCurVDistS.distS);

        if (acc > maxAcc) {
            return new double[]{targetVelocity, maxAcc};
        } else if (acc < Config.MAX_DECELERATION) {
            return new double[]{targetVelocity, Config.MAX_DECELERATION};
        } else {
            return new double[]{targetVelocity, acc};
        }
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

    public JSONObject parseJSON(final InputStream stream) {
        final JSONParser jsonParser = new JSONParser();
        try {
//            System.out.println("Is stream ready to rumble: " + streamReader.ready());
            final JSONObject obj = (JSONObject) jsonParser.parse(new InputStreamReader(stream));
            return obj;
        } catch (final Exception e) {
            System.out.println("Failed to open file. It is not in the right format or is corrupted.");
            System.out.println(e);
            e.printStackTrace();
            return null;
        }
    }

    public List<AnchorPoint> parseAnchorPoints(final JSONObject obj, final int config) {
        final List<AnchorPoint> anchorPointsList = new ArrayList<AnchorPoint>();
        final JSONArray anchorPoints = (JSONArray) obj.get("anchors");

        final JSONObject output = (JSONObject) obj.get("output");
        final JSONObject configObj = (JSONObject) output.get(Integer.toString(config));
        final JSONArray curves = (JSONArray) configObj.get("curves");

//            System.out.println("Read file");

        CurveParameters curParams = null;
        CurveParameters prevParams = null;

        System.out.println("Number of anchorPoints: " + anchorPoints.size());

        for (int i = 0; i < anchorPoints.size(); i++) {
            final JSONObject anchorObj = (JSONObject) anchorPoints.get(i);

            System.out.println("i = " + i);

            if (((Long)anchorObj.get("config")).intValue() != config && ((Long)anchorObj.get("config")).intValue() != 0) {
                continue;
            }

            final boolean first = (i == 0);
//            final boolean last = (i == anchorPoints.size() - 1);

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

            System.out.println("AnchorPoints list size: " + anchorPointsList.size());
            if (first) {
                curParams = new CurveParameters((JSONObject)curves.get(0));
                final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, Double.NaN, null, Double.NaN,
                        curParams.circle1Radius, curParams.circle1Center, curParams.endTheta1, null, curParams.p1, configVelocity, actionEventListeners,
                        actions, first, false);
                anchorPointsList.add(anchorPoint);
                prevParams = curParams.copy();
            } else {
                try {
                    curParams = new CurveParameters((JSONObject) curves.get(anchorPointsList.size()));
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, prevParams.circle2Radius, prevParams.circle2Center,
                            prevParams.endTheta2, curParams.circle1Radius, curParams.circle1Center, curParams.endTheta1, prevParams.p2, curParams.p1, configVelocity, actionEventListeners,
                            actions, first, false);
                    anchorPointsList.add(anchorPoint);
                    prevParams = curParams.copy();
                } catch (final IndexOutOfBoundsException e) { //Last
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, prevParams.circle2Radius, prevParams.circle2Center,
                            prevParams.endTheta2, Double.NaN, null, Double.NaN, prevParams.p2, null, configVelocity, actionEventListeners,
                            actions,first, true);
                    anchorPointsList.add(anchorPoint);
                }
            }
        }
        anchorPointsList.get(anchorPointsList.size() - 1).middlePoint.setConfigVelocity(0);
        return anchorPointsList;
    }

    public List<SegmentPoint> parseSegmentPoints(final JSONObject obj, final int config) {
        final List<SegmentPoint> segmentPoints = new ArrayList<SegmentPoint>();
        final JSONObject output = (JSONObject) obj.get("output");
        final JSONObject curveConfig = (JSONObject) output.get(Integer.toString(config));
        final JSONArray segments = (JSONArray) curveConfig.get("segments");

        for (int i = 0; i < segments.size(); i++) {
            final SegmentPoint segment = new SegmentPoint((JSONObject)segments.get(i), configActionEventListeners);
            segmentPoints.add(segment);
        }

        return segmentPoints;
    }
}
