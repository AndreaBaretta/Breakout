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
    public final List<AnchorPoint> anchorPoints;
    public final List<MainSegment> mainSegments;
    public final List<ConnectionPoint> connectionPoints;
    public final List<SegmentPoint> segmentPoints;
    public final List<HeadingSegment> headingSegments;

    protected MainSegment currentMainSegment;
    protected HeadingSegment currentHeadingSegment;
    protected boolean finished;

    public Path(final File foxtrotFile, final int config) {
        anchorPoints = parseAnchorPoints(foxtrotFile, config);
        mainSegments = new ArrayList<MainSegment>();
        connectionPoints = new ArrayList<ConnectionPoint>();
        segmentPoints = parseSegmentPoints(foxtrotFile, config);
//        velocitySegments = new ArrayList<VelocitySegment>();

        AnchorPoint curPoint = null;
        AnchorPoint prevPoint = curPoint;

        double s0 = 0;
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

                System.out.println("Current connections: " + prevPoint.nextPoint.toString() + curPoint.prevPoint.toString() + curPoint.middlePoint.toString());

                connectionPoints.add(prevPoint.nextPoint);
                connectionPoints.add(curPoint.prevPoint);
                connectionPoints.add(curPoint.middlePoint);

//                System.out.println("Connection point 0: " + connection0.toString());
//                System.out.println("Connection point 1: " + connection1.toString());
//                System.out.println("Connection point 2: " + connection2.toString());
//                System.out.println("Connection point 3: " + connection3.toString());

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

                final CircleSegment segment0 = new CircleSegment(connection0, connection1, s0, prevPoint.configVelocity, prevPoint.center1, prevPoint.r1, prevAnchorTheta, prevPoint.theta1, prevPoint.counterClockwise1);
                s0 = segment0.getEndS();
                final LinearSegment segment1 = new LinearSegment(connection1, connection2, s0, prevPoint.configVelocity);
                s0 = segment1.getEndS();
                final CircleSegment segment2 = new CircleSegment(connection2, connection3, s0, prevPoint.configVelocity, curPoint.center0, curPoint.r0, curPoint.theta0, curAnchorTheta, curPoint.counterClockwise0);
                s0 = segment2.getEndS();

//                System.out.println("zeroSegment circle1: " + segment0.zeroSegment + " firstpoint: " + segment0.firstPoint.toString() + "  " + "lastpoint: " + segment0.lastPoint.toString() + "  counterclockwise: " + segment0.counterClockwise + "  center: " + segment0.center.toString() + "  tangent: " + prevPoint.tan + "  length: " + segment0.getTotalS());
//                System.out.println("zeroSegment linear: " + segment1.zeroSegment + " firstpoint: " + segment1.firstPoint.toString() + "  " + "lastpoint: " + segment1.lastPoint.toString() + "  length: " + segment1.getTotalS());
//                System.out.println("zeroSegment circle2: " + segment2.zeroSegment + " firstpoint: " + segment2.firstPoint.toString() + "  " + "lastpoint: " + segment2.lastPoint.toString() + "  counterclockwise: " + segment2.counterClockwise + "  center: " + segment2.center.toString() + "  tangent: " + curPoint.tan + "  length: " + segment2.getTotalS()
//                 + "  theta0_: " + segment2.theta0_ + "  theta1_: " + segment2.theta1_ + "  theta0: " + segment2.theta0 + "  theta1: " + segment2.theta1);
//                System.out.println("At point, s = " + segment2.calcS(76.13/ Config.INCHES_TO_METERS, 68.24/Config.INCHES_TO_METERS));
//                System.out.println();

//                System.out.println(co);
                final List<SegmentPoint> curSegmentPoints = new ArrayList<SegmentPoint>();
                segmentPoints.forEach((final SegmentPoint p) -> {
                    if (p.anchorIndex == mainSegments.size()) {
                        curSegmentPoints.add(p);
                    }
                });
                final MainSegment mainSegment = new MainSegment(segment0, segment1, segment2, i - 1, prevPoint, curPoint, curSegmentPoints);

                mainSegments.add(mainSegment);
//                System.out.println("Domain mainsegment: " + mainSegment.getEndS());
//                System.out.println("Domain circlesegment1: " + mainSegment.circleSegment1.getEndS());

                prevPoint = curPoint;
            }
        }

        currentMainSegment = mainSegments.get(0);
        finished = false;

//        segmentPoints = parseSegmentPoints(foxtrotFile, config);
//
        final List<HeadingPoint> headingPoints = new ArrayList<HeadingPoint>();

        segmentPoints.forEach((final SegmentPoint p) -> {
            if (p.headingState != AnchorPoint.Heading.NONE) {
                headingPoints.add(p);
            }
        });

        connectionPoints.forEach((final ConnectionPoint p) -> {
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

        headingSegments = new ArrayList<HeadingSegment>();
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
                preiousHeadingSegment();
            } else {
                nextMainSegment();
            }
        }
        final double endS = currentMainSegment.getEndS();
        if (finished) { //If at end, stay there

            return new RobotState(
                    currentMainSegment.getPosition(endS),
                    currentMainSegment.getVelocity(endS, 0),
                    currentMainSegment.getAcceleration(endS, 0, 0)
            );
        }
        if (currentMainSegment.index == mainSegments.size() - 1 && s >= endS - 0.001) { //Condition to finish
            finish();
            System.out.println("Finished");
            return new RobotState(
                    currentMainSegment.getPosition(endS),
                    currentMainSegment.getVelocity(endS, 0),
                    currentMainSegment.getAcceleration(endS, 0, 0)
            );
        } else if (s < 0) {
            return new RobotState(
                    currentMainSegment.getPosition(currentMainSegment.s0),
                    currentMainSegment.getVelocity(currentMainSegment.s0, s_dot),
                    currentMainSegment.getAcceleration(currentMainSegment.s0, s_dot, s_dot_dot)
            );
        } else { //Else, do the normal thing
            return new RobotState(
                    currentMainSegment.getPosition(s),
                    currentMainSegment.getVelocity(s, s_dot),
                    currentMainSegment.getAcceleration(s, s_dot, s_dot_dot)
                );
        }
    }

    public double calcS(final double x, final double y) {
        final double curSegmentS = currentMainSegment.calcS(x, y);
        if (curSegmentS >= currentMainSegment.getEndS()) {
            nextMainSegment();
            return currentMainSegment.calcS(x, y);
        } else if (curSegmentS < currentMainSegment.s0) {
            previousMainSegment();
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

    protected void preiousHeadingSegment() {
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

    public double calcAccelerationCorrection(final double s, final double s_dot) {
//        double s_dot_dot = Config.MAX_ACCELERATION;
        final MinorSegment.NextVCurVDistS nextVCurVDistS = getNextVelocity(s);
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

    public MinorSegment.NextVCurVDistS getNextVelocity(final double s) {
        return currentMainSegment.getNextVelocity(s);
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

                if (first) {
                    curParams = new CurveParameters((JSONObject)curves.get(0));
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, Double.NaN, null, Double.NaN,
                            curParams.circle1Radius, curParams.circle1Center, curParams.endTheta1, null, curParams.p1, configVelocity, first, last);
                    anchorPointsList.add(anchorPoint);
                    prevParams = curParams.copy();
                } else if (last) {
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, prevParams.circle2Radius, prevParams.circle2Center,
                            prevParams.endTheta2, Double.NaN, null, Double.NaN, prevParams.p2, null, configVelocity, first, last);
                    anchorPointsList.add(anchorPoint);
                } else {
                    curParams = new CurveParameters((JSONObject)curves.get(anchorPointsList.size()));
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, prevParams.circle2Radius, prevParams.circle2Center,
                            prevParams.endTheta2, curParams.circle1Radius, curParams.circle1Center, curParams.endTheta1, prevParams.p2, curParams.p1, configVelocity, first, last);
                    anchorPointsList.add(anchorPoint);
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
                final SegmentPoint segment = new SegmentPoint((JSONObject)segments.get(i));
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
