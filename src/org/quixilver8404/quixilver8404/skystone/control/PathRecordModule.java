//package com.company.quixilver8404.skystone.control;
//
//import android.annotation.SuppressLint;
//import android.os.Environment;
//
//import com.company.quixilver8404.skystone.util.measurement.Angle;
//import com.company.quixilver8404.skystone.util.measurement.Distance;
//import com.company.quixilver8404.skystone.util.measurement.Pose2D;
//
//import java.io.File;
//import java.io.FileWriter;
//import java.io.IOException;
//import java.util.ArrayList;
//
//public class PathRecordModule {
//
//    public static final File recordFile = new File(Environment.getExternalStorageDirectory() + "/FIRST/recorded_path.csv");
//
//    public static final double SPACING_INCHES = PPPathBuilder.INJECT_SPACING_INCHES;
//
//    private boolean isRecording;
//
//    // all distances are in inches and all angles are in radians
//    private ArrayList<Double> xVals;
//    private ArrayList<Double> yVals;
//    private ArrayList<Double> headings;
//    private ArrayList<Double> leftIntakePowers;
//    private ArrayList<Double> rightIntakePowers;
//
//    private double lastRecX;
//    private double lastRecY;
//    private double lastOvershoot;
//
//    private double lastLeftIntakePower;
//    private double lastRightIntakePower;
//
//    public PathRecordModule() {
//        xVals = new ArrayList<>();
//        yVals = new ArrayList<>();
//        headings = new ArrayList<>();
//        leftIntakePowers = new ArrayList<>();
//        rightIntakePowers = new ArrayList<>();
//        reset();
//    }
//
//    public synchronized void update(BaseRobot robot) {
//        if (isRecording) {
//            Pose2D pose = robot.navModule.getPose();
//            double x = pose.x.getValue(Distance.Unit.INCHES);
//            double y = pose.y.getValue(Distance.Unit.INCHES);
//            double heading = pose.heading.getStandard(Angle.Unit.RADIANS);
//            double distFromLast = Math.hypot(x - lastRecX, y - lastRecY);
//            if (distFromLast + lastOvershoot >= SPACING_INCHES || lastOvershoot == -1) {
//                if (lastOvershoot == -1) {
//                    lastOvershoot = 0;
//                }
//                xVals.add(x);
//                yVals.add(y);
//                headings.add(heading);
//                lastRecX = x;
//                lastRecY = y;
//                lastOvershoot = distFromLast + lastOvershoot - SPACING_INCHES;
//
//                double[] intakePowers = robot.intakeModule.getTargetPowers();
//                double leftIntakePower = intakePowers[0];
//                double rightIntakePower = intakePowers[1];
//                if (leftIntakePower != lastLeftIntakePower) {
//                    leftIntakePowers.add(leftIntakePower);
//                    lastLeftIntakePower = leftIntakePower;
//                } else {
//                    leftIntakePowers.add(-3.0);
//                }
//                if (rightIntakePower != lastRightIntakePower) {
//                    rightIntakePowers.add(rightIntakePower);
//                    lastRightIntakePower = rightIntakePower;
//                } else {
//                    rightIntakePowers.add(-3.0);
//                }
//            }
//        }
//    }
//
//    public synchronized boolean isRecording() {
//        return isRecording;
//    }
//
//    public synchronized void record() {
//        isRecording = true;
//    }
//
//    public synchronized void stop() {
//        reset();
//    }
//
//    public synchronized void stopAndOutput() {
//        output();
//        stop();
//    }
//
//    @SuppressLint("DefaultLocale")
//    private void output() {
////        StringBuilder outputStringBuilder = new StringBuilder();
////        for (int i = 0; i < xVals.size(); i++) {
////            outputStringBuilder.append(String.format("%.3f", xVals.get(i)));
////            outputStringBuilder.append(",");
////            outputStringBuilder.append(String.format("%.3f", yVals.get(i)));
////            outputStringBuilder.append(",");
////            outputStringBuilder.append(String.format("%.3f", headings.get(i)));
////            outputStringBuilder.append(",");
////        }
////        Log.v("path_output", outputStringBuilder.toString());
//
//        try {
//            if (recordFile.exists()) {
//                //noinspection ResultOfMethodCallIgnored
//                recordFile.delete();
//            }
//            //noinspection ResultOfMethodCallIgnored
//            recordFile.createNewFile();
//            FileWriter csvWriter = new FileWriter(recordFile);
//            for (int i = 0; i < xVals.size(); i++) {
//                csvWriter.append(String.format("%.3f", xVals.get(i)));
//                csvWriter.append(",");
//                csvWriter.append(String.format("%.3f", yVals.get(i)));
//                csvWriter.append(",");
//                csvWriter.append(String.format("%.3f", headings.get(i)));
//                csvWriter.append(",");
//                csvWriter.append(String.format("%.2f", leftIntakePowers.get(i)));
//                csvWriter.append(",");
//                csvWriter.append(String.format("%.2f", rightIntakePowers.get(i)));
//                csvWriter.append("\n");
//            }
//            csvWriter.flush();
//            csvWriter.close();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//    }
//
//    private void reset() {
//        isRecording = false;
//        xVals.clear();
//        yVals.clear();
//        headings.clear();
//        leftIntakePowers.clear();
//        rightIntakePowers.clear();
//        lastRecX = 0;
//        lastRecY = 0;
//        lastOvershoot = -1;
//        lastLeftIntakePower = -3;
//        lastRightIntakePower = -3;
//    }
//
//}
