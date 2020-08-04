package org.quixilver8404.feedforward;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.quixilver8404.simulator.PowerProfile;
import org.quixilver8404.simulator.Vector3;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;


public class Path {
    public final List<AnchorPoint> anchorPoints;
    public final List<MainSegment> mainSegments;

    protected MainSegment currentSegment;
    protected boolean finished;

    public Path(final List<AnchorPoint> anchorPoints) {
        this.anchorPoints = anchorPoints;
        this.mainSegments = new ArrayList<MainSegment>();

        AnchorPoint curPoint = null;
        AnchorPoint prevPoint = curPoint;

        double s0 = 0;
        double connectionPointCounter = 0;
        for (int i = 0; i < anchorPoints.size(); i++) {
            curPoint = anchorPoints.get(i);
            if (curPoint.first) {
                prevPoint = curPoint;
            } else {
                final ConnectionPoint connection0 = prevPoint.middlePoint;
                final ConnectionPoint connection1 = prevPoint.nextPoint;
                final ConnectionPoint connection2 = curPoint.prevPoint;
                final ConnectionPoint connection3 = curPoint.middlePoint;

                System.out.println("Connection point 0: " + connection0.toString());
                System.out.println("Connection point 1: " + connection1.toString());
                System.out.println("Connection point 2: " + connection2.toString());
                System.out.println("Connection point 3: " + connection3.toString());

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
                System.out.println("zeroSegment circle2: " + segment2.zeroSegment + " firstpoint: " + segment2.firstPoint.toString() + "  " + "lastpoint: " + segment2.lastPoint.toString() + "  counterclockwise: " + segment2.counterClockwise + "  center: " + segment2.center.toString() + "  tangent: " + curPoint.tan + "  length: " + segment2.getTotalS()
                 + "  theta0_: " + segment2.theta0_ + "  theta1_: " + segment2.theta1_ + "  theta0: " + segment2.theta0 + "  theta1: " + segment2.theta1);
                System.out.println("At point, s = " + segment2.calcS(76.13/Config.INCHES_TO_METERS, 68.24/Config.INCHES_TO_METERS));
//                System.out.println();

//                System.out.println(co);

                final MainSegment mainSegment = new MainSegment(segment0, segment1, segment2, i - 1, prevPoint, curPoint);

                mainSegments.add(mainSegment);

                prevPoint = curPoint;
            }
        }

        currentSegment = mainSegments.get(0);
        finished = false;
        System.out.println("End S: " + mainSegments.get(mainSegments.size() - 1).getEndS());
    }

    public RobotState evaluate(final double s, final double s_dot, final  double s_dot_dot) {
        final double endS = currentSegment.getEndS();
        if (finished) { //If at end, stay there
            return new RobotState(
                    currentSegment.getPosition(endS),
                    currentSegment.getVelocity(endS, 0),
                    currentSegment.getAcceleration(endS, 0, 0)
            );
        }
        if (currentSegment.index == mainSegments.size() - 1 && s >= endS - 0.01) { //Condition to finish
            finish();
            return new RobotState(
                    currentSegment.getPosition(endS),
                    currentSegment.getVelocity(endS, 0),
                    currentSegment.getAcceleration(endS, 0, 0)
            );
        } else if (s < 0) {
            return new RobotState(
                    currentSegment.getPosition(currentSegment.s0),
                    currentSegment.getVelocity(currentSegment.s0, s_dot),
                    currentSegment.getAcceleration(currentSegment.s0, s_dot, s_dot_dot)
            );
        } else { //Else, do the normal thing
            return new RobotState(
                    currentSegment.getPosition(s),
                    currentSegment.getVelocity(s, s_dot),
                    currentSegment.getAcceleration(s, s_dot, s_dot_dot)
                );
        }
    }

    public double calcS(final double x, final double y) {
        final double curSegmentS = currentSegment.calcS(x, y);
        if (curSegmentS >= currentSegment.getEndS()) {
            next();
            return currentSegment.calcS(x, y);
        } else if (curSegmentS < currentSegment.s0) {
            back();
            return currentSegment.calcS(x, y);
        } else {
            return curSegmentS;
        }
    }

    protected void next() {
        if (currentSegment.index == mainSegments.size() - 1) {
            ;
        } else {
            System.out.println("Next segment");
            currentSegment = mainSegments.get(currentSegment.index + 1);
        }
    }

    protected void back() {
        if (currentSegment.index == 0) {
            ;
        } else {
            System.out.println("Previous segment");
            currentSegment = mainSegments.get(currentSegment.index - 1);
        }
    }

    protected void finish() {
        finished = true;
    }

    public double calcAccelerationCorrection(final double s, final double s_dot) {
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

    public static Path foxtrotParser(final File file, final int config) {
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

                System.out.println();
                if (first) {
                    curParams = new CurveParameters((JSONObject)curves.get(0));
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, Double.NaN, null, Double.NaN,
                            curParams.circle1Radius, curParams.circle1Center, curParams.endTheta1, null, curParams.p1, configVelocity, first, last);
//                    System.out.println("In first");
//                    System.out.println(anchorPointsList.size());
                    anchorPointsList.add(anchorPoint);
                    prevParams = curParams.copy();
//                    System.out.println(anchorPointsList.size());
                } else if (last) {
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, prevParams.circle2Radius, prevParams.circle2Center,
                            prevParams.endTheta2, Double.NaN, null, Double.NaN, prevParams.p2, null, configVelocity, first, last);
                    anchorPointsList.add(anchorPoint);
                    System.out.println("endTheta2: " + prevParams.endTheta2);
                } else {
                    System.out.println("Index of curves: " + anchorPoints.size());
                    curParams = new CurveParameters((JSONObject)curves.get(anchorPointsList.size()));
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, prevParams.circle2Radius, prevParams.circle2Center,
                            prevParams.endTheta2, curParams.circle1Radius, curParams.circle1Center, curParams.endTheta1, prevParams.p2, curParams.p1, configVelocity, first, last);
//                    System.out.println(anchorPointsList.size());
                    anchorPointsList.add(anchorPoint);
//                    System.out.println(anchorPointsList.size());
                    prevParams = curParams.copy();
                }
                System.out.println("Parsed");
            }
        } catch (final Exception e) {
            System.out.println("Failed to open " + file.getPath() + ". The file is not in the right format or is corrupted.");
            System.out.println(e);
            e.printStackTrace();
        }

        return new Path(anchorPointsList);
    }
}
