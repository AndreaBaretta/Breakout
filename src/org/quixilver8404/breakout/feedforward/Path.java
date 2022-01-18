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

    protected Segment currentSegment;
    protected HeadingSegment currentHeadingSegment;
    protected VelocitySegment currentVelocitySegment;
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

    public Path(final InputStream foxtrotFile, final int config, final List<ActionEventListener> configActionEventListeners) {
        this.configActionEventListeners = configActionEventListeners;
        final JSONObject obj = parseJSON(foxtrotFile);
        anchorPoints = parseAnchorPoints(obj, config);
        connectionPoints = new ArrayList<ConnectionPoint>();
        segmentPoints = parseSegmentPoints(obj, config);
        actionPoints = new ArrayList<ActionPoint>();
        segments = new ArrayList<Segment>();
        velocitySegments = new ArrayList<VelocitySegment>();
        lastKnownS = 0;

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

                System.out.println("Connection Point: " + connection0.toString());
                System.out.println("Connection Point: " + connection1.toString());
                System.out.println("Connection Point: " + connection2.toString());
                System.out.println("Connection Point: " + connection3.toString());


                final double prevAnchorTheta = Vector3.normalizeAlpha(Math.atan2(prevPoint.middlePoint.y-prevPoint.center1.y, prevPoint.middlePoint.x-prevPoint.center1.x));
                final double curAnchorTheta = Vector3.normalizeAlpha(Math.atan2(curPoint.middlePoint.y-curPoint.center0.y, curPoint.middlePoint.x-curPoint.center0.x));

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
                    }
                });

                prevPoint = curPoint;
            }
        }

        System.out.println("ConnectionPoints: " + Arrays.toString(connectionPoints.toArray()));

        currentSegment = segments.get(0);
        finished = false;

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

        System.out.println("velocityPoints_ after sort: ");
        velocityPoints_.forEach(p -> System.out.println(p.toString()));

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

        System.out.println("---------------------------");
        System.out.println("VelocityPoint i=1: " + velocityPoints_.get(1).toString());

        for (int i = 0; i < velocityPoints_.size(); i++) {
            if (i != 0) {
                final VelocityPoint velocityPoint = velocityPoints_.get(i);
                if (Double.isNaN(velocityPoint.getMaxVelocity())) { // Segment points
                    final double prevMaxVelocity = velocityPoints_.get(i - 1).getMaxVelocity();
                    if (prevMaxVelocity == 0.0) {
                        velocityPoint.setMaxVelocity(velocityPoint.getConfigVelocity());
                    } else {
                        velocityPoint.setMaxVelocity(prevMaxVelocity);
                    }
                }
                if (Double.isNaN(velocityPoint.getConfigVelocity())) {// Non-anchor connection points
                    System.out.println("Reset config velocity at i=" + i);
                    if (velocityPoints_.get(i-1).getS() == velocityPoint.getS()) {
                        velocityPoint.setConfigVelocity(velocityPoints_.get(i - 1).getConfigVelocity());
                    } else if (velocityPoints_.get(i - 1).getConfigVelocity() != 0) {
                        velocityPoint.setConfigVelocity(velocityPoints_.get(i - 1).getConfigVelocity());
                    } else {
                        double nextVelocity = -1;
                        if (i == velocityPoints_.size() - 1) { // Either this doesn't happen or someone fucked up BIG TIME
                            nextVelocity = 0;
                        } else {
                            for (int j = 1; j + i < velocityPoints_.size(); j++) {
                                nextVelocity = velocityPoints_.get(i + j).getConfigVelocity();
                                System.out.println("Next velocity for i= " + i + ": " + nextVelocity);
                                if (!Double.isNaN(nextVelocity)) { // If it's 0, then fuck it. Not my problem
                                    break;
                                }
                            }
                            if (Double.isNaN(nextVelocity)) { // Same as before. Either this doesn't happen or someone is bound to face the wrath of God
                                nextVelocity = 0;
                            }
                        }
                        System.out.println("Set next velocity for i= " + i + ": " + nextVelocity);
                        velocityPoint.setConfigVelocity(nextVelocity);
//                    velocityPoint.setConfigVelocity(velocityPoints_.get(i - 1).getConfigVelocity());
                    }
                }
            }
        }

        System.out.println("velocityPoints_ after setting velocities: ");
        velocityPoints_.forEach(p -> System.out.println(p.toString()));

        final List<VelocityPoint> velocityPoints = new ArrayList<>();
        for (int i = 0; i < velocityPoints_.size(); i++) {
            if (i != velocityPoints_.size() - 1) {
                if (velocityPoints_.get(i + 1).getS() - velocityPoints_.get(i).getS() >= 1e-12) {
                    velocityPoints.add(velocityPoints_.get(i));
                }
            } else {
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
                prevVelocityPoint = velocityPoint;
            }
        }

        System.out.println("velocityPoints: ");
        velocityPoints.forEach(p -> System.out.println(p.toString()));

        currentVelocitySegment = velocitySegments.get(0);

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

        final ConnectionPoint firstPoint = segments.get(0).firstPoint;
        startX = firstPoint.x;
        startY = firstPoint.y;
        startHeading = currentHeadingSegment.calcAlpha(0, currentSegment.getPosition(0)).theta;

        holdPos = new Vector3(startX, startY, startHeading);

        System.out.println();

        System.out.println("End S: " + segments.get(segments.size() - 1).getEndS());

//        segments.forEach(p -> System.out.println("Segment: " + p.toString()));
//        velocitySegments.forEach(p -> System.out.println("Velocity segment: " + p.toString()));

//        connectionPoints.forEach(p -> System.out.println("Connection point: " + p.toString()));
//        segmentPoints.forEach(p -> System.out.println("Segment point: " + p.toString()));
        headingPoints.forEach(p -> System.out.println("Heading point: " + p.toString()));
        headingSegments.forEach(s -> System.out.println("Heading segment: " + s.toString()));

        System.out.println("Velocity segments");
        velocitySegments.forEach(s -> System.out.println(s.toString()));
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
        if (currentVelocitySegment.index == velocitySegments.size() - 1) {
            ;
        } else {
            currentVelocitySegment = velocitySegments.get(currentVelocitySegment.index + 1);
            System.out.println("New vel segment: " + currentVelocitySegment.toString());
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

        final double s = Math.max(s_, currentSegment.s0);

        VelocitySegment.NextVCurVDistS nextVCurVDistS = getNextVelocity(s);

        if (nextVCurVDistS.distS <= 0 && currentVelocitySegment.index != velocitySegments.size() - 1 && nextVCurVDistS.nextV == 0) {
            currentVelocitySegment.p1.setConfigVelocity(Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY);
            currentVelocitySegment.p1.setMaxVelocity(Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY);
//            System.out.println("Crazy bullshit: " + currentVelocitySegment.toString());
        }

        if (s > currentVelocitySegment.s1) {
            nextVelocitySegment();
        }

//        System.out.println("Current velocity segment (at s=" + s + ")");
//        System.out.println(currentVelocitySegment.toString());

        nextVCurVDistS = getNextVelocity(s);

//        if (nextVCurVDistS.distS <= 0 && currentVelocitySegment.index != velocitySegments.size() - 1 && nextVCurVDistS.nextV == 0) {
//            currentVelocitySegment.p1.setConfigVelocity(Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY);
//            currentVelocitySegment.p1.setMaxVelocity(Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY);
//            System.out.println("Crazy bullshit: " + currentVelocitySegment.toString());
//        }

        final double maxAcc = findMaxPossibleAcc(s, s_dot);

//        final double d_s = nextVCurVDistS.distS;
//        final double v_f = nextVCurVDistS.nextV;
//        final double accToVel = (1/d_s)*(0.5 * Math.pow(v_f - s_dot, 2) + s_dot * (v_f - s_dot));
        final double accToVel = (Math.pow(nextVCurVDistS.nextV, 2) - Math.pow(s_dot, 2))/(2 * nextVCurVDistS.distS);

        final double targetVelocity;
        if (currentVelocitySegment.hasStartedAcceleration()) {
            targetVelocity = nextVCurVDistS.curV + ((nextVCurVDistS.nextV-nextVCurVDistS.curV)/(currentVelocitySegment.s1-currentVelocitySegment.getAccPoint()))*(s - currentVelocitySegment.getAccPoint());
//            return new double[]{nextVCurVDistS.nextV, accToVel};
        } else if (accToVel < Config.MAX_DECELERATION*0.7) {
            currentVelocitySegment.startAcceleration(s);
            targetVelocity = nextVCurVDistS.curV + ((nextVCurVDistS.nextV-nextVCurVDistS.curV)/(currentVelocitySegment.s1-currentVelocitySegment.getAccPoint()))*(s - currentVelocitySegment.getAccPoint());
//            System.out.println("Start decelerating");
//            return new double[]{nextVCurVDistS.nextV, accToVel};
        } else if (accToVel > maxAcc*0.7) {
            currentVelocitySegment.startAcceleration(s);
            targetVelocity = nextVCurVDistS.curV + ((nextVCurVDistS.nextV-nextVCurVDistS.curV)/(currentVelocitySegment.s1-currentVelocitySegment.getAccPoint()))*(s - currentVelocitySegment.getAccPoint());
//            return new double[]{nextVCurVDistS.nextV, accToVel};
        } else {
            targetVelocity = nextVCurVDistS.curV;
        }

        final double acc = Math.tan(Config.ACCELERATION_CORRECTION)*(targetVelocity - s_dot) + ((currentVelocitySegment.hasStartedAcceleration()) ? accToVel : 0);

        System.out.println("targetVel: " + targetVelocity + "  accToVel: " + accToVel + "  overall acc: " + acc + "  distS: " + nextVCurVDistS.distS);

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

    public VelocitySegment.NextVCurVDistS getNextVelocity(final double s) {
        return currentVelocitySegment.getNextVelocity(s);
    }

    public JSONObject parseJSON(final InputStream stream) {
        final JSONParser jsonParser = new JSONParser();
        try {
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
        final JSONArray segments = (JSONArray) obj.get("segments");
        for (int i = 0; i < segments.size(); i++) {
            final SegmentPoint segment = new SegmentPoint((JSONObject)segments.get(i), configActionEventListeners);
            if (segment.config == config || segment.config == 0) {
                segmentPoints.add(segment);
            }
        }

        return segmentPoints;
    }
}
