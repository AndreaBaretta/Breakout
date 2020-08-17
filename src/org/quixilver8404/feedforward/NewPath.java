package org.quixilver8404.feedforward;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.quixilver8404.util.Config;

import java.io.File;
import java.io.FileReader;
import java.util.*;

public class NewPath {
    final List<AnchorPoint> anchorPoints;
    final List<SegmentPoint> segmentPoints;
    final List<ActionPoint> actionPoints;

    final List<ActionEventListener> configActionEventListeners;

    final List<ContinuousSegment> continuousSegments;

    protected ContinuousSegment curSegment;

    public NewPath(final File file, final int config, final List<ActionEventListener> configActionEventListeners) {
        this.configActionEventListeners = configActionEventListeners;
        anchorPoints = parseAnchorPoints(file, config);
        segmentPoints = parseSegmentPoints(file, config);
        final List<ActionPoint> actionPoints_ = new ArrayList<ActionPoint>();
        continuousSegments = new ArrayList<ContinuousSegment>();

        final List<AnchorPoint> curAnchorPoints = new ArrayList<AnchorPoint>();
        final List<SegmentPoint> curSegmentPoints = new ArrayList<SegmentPoint>();

        int counter = 0;
        double s0 = 0;
        for (int i = 0; i < anchorPoints.size(); i++) {
            final AnchorPoint anchorPoint = anchorPoints.get(i);
            curAnchorPoints.add(anchorPoint.copy());
            final int index = i;
            curSegmentPoints.forEach(p -> {
                if (p.anchorIndex == index) {
                    curSegmentPoints.add(p);
                }
            });
            if (anchorPoint.r0 <= 1e-12 || anchorPoint.r1 <= 1e-12 || anchorPoint.last) {
                System.out.println("Created continuousSegment of index: " + counter);
                final ContinuousSegment newSegment = new ContinuousSegment(curAnchorPoints, curSegmentPoints, s0, counter);
                continuousSegments.add(newSegment);
                curAnchorPoints.clear();
                curSegmentPoints.clear();
                counter++;
                s0 = newSegment.s1;
                final AnchorPoint newAnchorpoint = anchorPoint;
                newAnchorpoint.middlePoint.setConfigVelocity(Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY);
                newAnchorpoint.nextPoint.setConfigVelocity(Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY);
                curAnchorPoints.add(newAnchorpoint.copy());
            }
        }

        continuousSegments.forEach(p -> {
            actionPoints_.addAll(p.actionPoints);
        });
        actionPoints = new ArrayList<ActionPoint>(new HashSet<ActionPoint>(actionPoints_));

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

        curSegment = continuousSegments.get(0);
    }

    public final double calcS(final double x, final double y) {
        if (curSegment.getFinished()) {
            System.out.println("Finished continuous segment");
            nextContinuousSegment();
        }
        return curSegment.calcS(x, y);
    }

    public final RobotState evaluate(final double s, final double s_dot, final double s_dot_dot) {
//        System.out.println("EndS cursSegment: " + curSegment.s1);
        System.out.println("s=" + s + "  s_dot="+s_dot + "  s_dot_dot="+s_dot_dot + "  pos:" +  curSegment.evaluate(s, s_dot, s_dot_dot).pos.toString() + "  vel:" +  curSegment.evaluate(s, s_dot, s_dot_dot).vel.toString()  + "  acc:" +  curSegment.evaluate(s, s_dot, s_dot_dot).acc.toString() + " curSegment.index="+ curSegment.index);
        return curSegment.evaluate(s, s_dot, s_dot_dot);
    }

    protected void nextContinuousSegment() {
        if (curSegment.index == continuousSegments.size() - 1) {
            ;
        } else {
            System.out.println("Next continuous segment");
            curSegment = continuousSegments.get(curSegment.index + 1);
        }
    }

    public double calcAccelerationCorrection(final double s, final double s_dot) {
        System.out.println("acceleration: " + curSegment.calcAccelerationCorrection(s, s_dot));
        return curSegment.calcAccelerationCorrection(s, s_dot);
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

                final double x = ((double)anchorObj.get("x"))* Config.INCHES_TO_METERS;
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
                System.out.println(Arrays.toString(configActionEventListeners.toArray()));
                if (!configActionEventListeners.isEmpty() && !actions.isEmpty()) {
                    for (final Integer action : actions) {
                        for (final ActionEventListener eventListener : configActionEventListeners) {
                            if (eventListener.action == action) {
                                actionEventListeners.add(eventListener);
                            }
                        }
                    }
                }

                System.out.println("AnchorPoint Actions: " + Arrays.toString(actions.toArray()));

                if (first) {
                    curParams = new CurveParameters((JSONObject)curves.get(0));
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, Double.NaN, null, Double.NaN,
                            curParams.circle1Radius, curParams.circle1Center, curParams.endTheta1, null, curParams.p1, configVelocity, actionEventListeners,
                            actions, first, last);
                    anchorPointsList.add(anchorPoint);
                    prevParams = curParams.copy();
                } else if (last) {
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, prevParams.circle2Radius, prevParams.circle2Center,
                            prevParams.endTheta2, Double.NaN, null, Double.NaN, prevParams.p2, null, configVelocity, actionEventListeners,
                            actions,first, last);
                    anchorPointsList.add(anchorPoint);
                } else {
                    curParams = new CurveParameters((JSONObject)curves.get(anchorPointsList.size()));
                    final AnchorPoint anchorPoint = new AnchorPoint(x, y, tan, heading, customHeading, prevParams.circle2Radius, prevParams.circle2Center,
                            prevParams.endTheta2, curParams.circle1Radius, curParams.circle1Center, curParams.endTheta1, prevParams.p2, curParams.p1, configVelocity, actionEventListeners,
                            actions, first, last);
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
