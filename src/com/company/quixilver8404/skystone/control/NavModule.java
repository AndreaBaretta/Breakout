package com.company.quixilver8404.skystone.control;

import com.company.quixilver8404.skystone.control.math.Vector3;
import com.company.quixilver8404.skystone.util.Tunable;
import com.company.quixilver8404.skystone.util.measurement.Angle;
import com.company.quixilver8404.skystone.util.measurement.Distance;
import com.company.quixilver8404.skystone.util.measurement.Pose2D;
import com.company.quixilver8404.skystone.util.measurement.Position2D;
import com.company.simulator.MecanumKinematics;

/**
 * Handles tracking the robot's location and heading
 */
public class NavModule {

    // max velocity of the robot driving forwards
    @Tunable
    public static final double MAX_VEL_IN_PER_SEC = 18.0;

    // max acceleration of the robot driving forwards
    @Tunable
    public static final double MAX_ACCEL_IN_PER_SEC_PER_SEC = MAX_VEL_IN_PER_SEC / 0.7;

    private int lastDriveEncoderFL = 0;
    private int lastDriveEncoderFR = 0;
    private int lastDriveEncoderBL = 0;
    private int lastDriveEncoderBR = 0;

    @Tunable
    public static final double DRIVE_ENCODER_MM_PER_COUNT = 35 * Math.PI / 4000.0 * (2476.0 / 2483.0); // TODO tune
    @Tunable
    public static final double LEFT_RIGHT_DRIVE_ENCODER_FROM_CENTER_MM = 181.0 * (3610.0 / 3600.0); //TODO tune
    @Tunable
    public static final double CENTER_DRIVE_ENCODER_FROM_CENTER_MM = 181.0; // TODO tune

    // the center of rotation for the robot is the reference point
    // the heading is relative to the driver's point of view, straight ahead is 90 degrees
    private double xMM;
    private double yMM;
    private Angle heading;

    @Tunable
    protected static final Vector3[] setup = new Vector3[]{
            new Vector3(41.72, 40.095, 3 * Math.PI / 4),
            new Vector3(-41.72, 40.095, Math.PI / 4) //TODO fix this
    };


    protected Pose2D deltaPos;

    public final MecanumKinematics kinematics;

    public NavModule(Pose2D curPose, MecanumKinematics kinematics) {
        this.kinematics = kinematics;
        setCurHeading(curPose.heading);
        setCurPosition(curPose);
    }

    public synchronized void update() {
        updatePose();

//        lastDriveEncoderFL = hwCollection.driveMotorFL.getEncoder().getEncoderPosition();
//        lastDriveEncoderFR = hwCollection.driveMotorFR.getEncoder().getEncoderPosition();
//        lastDriveEncoderBL = hwCollection.driveMotorBL.getEncoder().getEncoderPosition();
//        lastDriveEncoderBR = hwCollection.driveMotorBR.getEncoder().getEncoderPosition();

//        calculatePose(hwCollection);
    }

    /**
     * Adjusts the internal reference angle such that the field-oriented drive system reads the
     * robot's current heading as curHeading. This uses the last heading from update() for reference
     */
    public synchronized void setCurHeading(Angle curHeading) {
        heading = curHeading;
    }

    public synchronized void setCurPosition(Position2D position) {
        xMM = position.x.getValue(Distance.Unit.MILLIMETERS);
        yMM = position.y.getValue(Distance.Unit.MILLIMETERS);
    }

    public synchronized Pose2D getPose() {
        return new Pose2D(new Distance(xMM, Distance.Unit.MILLIMETERS), new Distance(yMM, Distance.Unit.MILLIMETERS), heading);
    }

    public synchronized Vector3 getPoseVector3() {
        return new Vector3(xMM, yMM, heading.getStandard(Angle.Unit.RADIANS));
    }

    public synchronized Angle getHeading() {
        return heading;
    }

    public synchronized Pose2D getDeltaPos() {
        return deltaPos;
    }

    public void reset(final Pose2D position) {
        setCurHeading(position.heading);
        setCurPosition(position);
    }

    private void updatePose() {
//        double leftEncMM = hwCollection.driveEncoderLeft.getDeltaPosition() * DRIVE_ENCODER_MM_PER_COUNT;
//        double rightEncMM = hwCollection.driveEncoderRight.getDeltaPosition() * DRIVE_ENCODER_MM_PER_COUNT;
//        double centerEncMM = hwCollection.driveEncoderCenter.getDeltaPosition() * DRIVE_ENCODER_MM_PER_COUNT;
//
//        // variables to simplify the expressions
//        double L = leftEncMM;
//        double R = rightEncMM;
//        double C = centerEncMM;
//        double P = LEFT_RIGHT_DRIVE_ENCODER_FROM_CENTER_MM;
//        double Q = CENTER_DRIVE_ENCODER_FROM_CENTER_MM;
//
//        // calculate the robot's new position and heading
//        double curHeadingRad = heading.getStandard(Angle.Unit.RADIANS);
//        double theta = (R - L) / (2 * P);
//
//        double dXIntrinsMM = R * sinOverX(theta) - P * Math.sin(theta) + C * cosMinus1OverX(theta) - Q * (Math.cos(theta) - 1);
//        double dYIntrinsMM = C * sinOverX(theta) - Q * Math.sin(theta) - R * cosMinus1OverX(theta) + P * (Math.cos(theta) - 1);
//
//        heading = new Angle(curHeadingRad + theta, Angle.Unit.RADIANS);
//
//        xMM += dXIntrinsMM * Math.cos(curHeadingRad) - dYIntrinsMM * Math.sin(curHeadingRad);
//        yMM += dYIntrinsMM * Math.cos(curHeadingRad) + dXIntrinsMM * Math.sin(curHeadingRad);

        xMM = kinematics.getFieldPos().x/1000d;
        yMM = kinematics.getFieldPos().y/1000d;
        heading = new Angle(kinematics.getFieldPos().theta, Angle.Unit.RADIANS);

        // TODO uncomment once calculatePose() is working
//        pose = calculatePose(pose, hwCollection);
    }

    /**
     * This will use a taylor series to compute sin(x)/x if x is small.
     * Otherwise, this will compute it using the built in trig functions.
     */
    private double sinOverX(double x) {
        double threshold = 2;
        if (Math.abs(x) < threshold) {
            int n = 9;
            double result = 0;
            double numer = 1;
            double denom = 1;
            for (int i = 0; i < n; i++) {
                result += numer / denom;
                numer *= -x * x;
                denom *= (2 * i + 2) * (2 * i + 3);
            }
            return result;
        } else {
            return Math.sin(x) / x;
        }
    }

    /**
     * This will use a taylor series to compute (cos(x)-1)/x if x is small.
     * Otherwise, this will compute it using the built in trig functions.
     */
    private double cosMinus1OverX(double x) {
        double threshold = 2;
        if (Math.abs(x) < threshold) {
            int n = 9;
            double result = 0;
            double numer = -x;
            double denom = 2;
            for (int i = 0; i < n; i++) {
                result += numer / denom;
                numer *= -x * x;
                denom *= (2 * i + 3) * (2 * i + 4);
            }
            return result;
        } else {
            return (Math.cos(x) - 1) / x;
        }
    }

//    private void calculatePose(final HardwareCollection hwCollection) {


//        double leftEncoderDelta = hwCollection.driveEncoderLeft.getDeltaPosition();
//        double rightEncoderDelta = hwCollection.driveEncoderRight.getDeltaPosition();
//        double centerEncoderDelta = hwCollection.driveEncoderCenter.getDeltaPosition();

//        double encoderDeltaBL = hwCollection.driveMotorBL.getEncoder().getDeltaPosition();
//        double encoderDeltaFL = hwCollection.driveMotorFL.getEncoder().getDeltaPosition();
//        double encoderDeltaBR = hwCollection.driveMotorBR.getEncoder().getDeltaPosition();
//        double encoderDeltaFR = hwCollection.driveMotorFR.getEncoder().getDeltaPosition();

//        double encoderDeltaLeft = (encoderDeltaBL + encoderDeltaFL) / 2;
//        double encoderDeltaRight = (encoderDeltaBR + encoderDeltaFR) / 2;


//        int deltaTimeMillis = hwCollection.clock.getDeltaTimeMillis();

//        imuReadTimeTrackerMillis += deltaTimeMillis;
//        if (imuReadTimeTrackerMillis >= IMU_READ_INTERVAL_MILLISEC) {
//            imuReadTimeTrackerMillis %= IMU_READ_INTERVAL_MILLISEC;
//            // here we can update the heading and do something with it
//            final Angle imuHeading = readRelativeIMUHeading(hwCollection);
//        }
//        final Vector3 prevPos = odometryMath.pos;
//        odometryMath = odometryMath.updateSimple(new double[]{encoderDeltaLeft, encoderDeltaRight});
////        odometryMath = odometryMath.update(new double[]{centerEncoderDelta, rightEncoderDelta, leftEncoderDelta});
//        pos = odometryMath.pos;
//        pose = pos.toPose2D();
//        deltaPos = Vector3.SubtractVector(pos, prevPos).toPose2D();
//    }

    public synchronized int getLastDriveEncoderFL() {
        return lastDriveEncoderFL;
    }

    public synchronized int getLastDriveEncoderFR() {
        return lastDriveEncoderFR;
    }

    public synchronized int getLastDriveEncoderBL() {
        return lastDriveEncoderBL;
    }

    public synchronized int getLastDriveEncoderBR() {
        return lastDriveEncoderBR;
    }
}
