package com.company.quixilver8404.skystone.control;

import com.company.quixilver8404.skystone.util.Tunable;
import com.company.quixilver8404.skystone.util.VectorUtil;
import com.company.quixilver8404.skystone.util.measurement.Angle;
import com.company.simulator.MecanumKinematics;

/**
 * Handles moving the base of the robot at a specified velocity
 */
public class DriveModule {

    @Tunable
    public static final int MAX_FORWARD_VEL_COUNTS_PER_SEC = 2750;

    // the "front" of the robot points along the direction vector <1, 0>
    private static final double[] mecanumFLDirection = VectorUtil.toUnitVector2D(1, -1);
    private static final double[] mecanumFRDirection = VectorUtil.toUnitVector2D(1, 1);
    private static final double[] mecanumBLDirection = VectorUtil.toUnitVector2D(1, 1);
    private static final double[] mecanumBRDirection = VectorUtil.toUnitVector2D(1, -1);

    private double targetPowerX = 0;
    private double targetPowerY = 0;

    private double targetRotatePower = 0;

    private boolean isPowerIntrinsic = false; // whether targetPowerX and targetPowerY are relative to the robot

    public DriveModule() {

    }

    public synchronized void update(BaseRobot baseRobot, MecanumKinematics kinematics) {
        System.out.println("Thread ID: " + Thread.currentThread().getId());
        double curHeadingRad = baseRobot.navModule.getHeading().getStandard(Angle.Unit.RADIANS);

        // these are relative powers
        double intrinsicPowerX;
        double intrinsicPowerY;


        if (isPowerIntrinsic) {
            intrinsicPowerX = targetPowerX;
            intrinsicPowerY = targetPowerY;
        } else {
            intrinsicPowerX = targetPowerX * Math.cos(-curHeadingRad) - targetPowerY * Math.sin(-curHeadingRad);
            intrinsicPowerY = targetPowerX * Math.sin(-curHeadingRad) + targetPowerY * Math.cos(-curHeadingRad);
        }

        // VERY VERY VERY CRUDE IMPLEMENTATION OF A MECANUM CONTROL
        // FOR PRELIMINARY TESTING ONLY!!!!!
        double powerFL = intrinsicPowerX - intrinsicPowerY - targetRotatePower;
        double powerFR = intrinsicPowerX + intrinsicPowerY + targetRotatePower;
        double powerBL = intrinsicPowerX + intrinsicPowerY - targetRotatePower;
        double powerBR = intrinsicPowerX - intrinsicPowerY + targetRotatePower;

        double maxPower = Math.max(Math.abs(powerFL), Math.abs(powerFR));
        maxPower = Math.max(maxPower, Math.abs(powerBL));
        maxPower = Math.max(maxPower, Math.abs(powerBR));

        if (maxPower > 1) {
            powerFL /= maxPower;
            powerFR /= maxPower;
            powerBL /= maxPower;
            powerBR /= maxPower;
        }

//        hwCollection.driveMotorFL.setVelocity(powerFL * MAX_FORWARD_VEL_COUNTS_PER_SEC);
//        hwCollection.driveMotorFR.setVelocity(powerFR * MAX_FORWARD_VEL_COUNTS_PER_SEC);
//        hwCollection.driveMotorBL.setVelocity(powerBL * MAX_FORWARD_VEL_COUNTS_PER_SEC);
//        hwCollection.driveMotorBR.setVelocity(powerBR * MAX_FORWARD_VEL_COUNTS_PER_SEC);
        kinematics.updatePresetFrequency(new double[]{
                powerFR, powerFL, powerBL, powerBR
        });
    }

    /**
     * Sets the power proportional to the robot's top speed in reference to the field orientation
     */
    public synchronized void setExtrinsicTargetPower(double targetPowerX, double targetPowerY) {
        this.targetPowerX = targetPowerX;
        this.targetPowerY = targetPowerY;
        isPowerIntrinsic = false;
    }

    public synchronized void setExtrinsicTargetPower(double targetPower, Angle direction) {
        double moveDirectionRad = direction.getStandard(Angle.Unit.RADIANS);
        double targetPowerX = targetPower * Math.cos(moveDirectionRad);
        double targetPowerY = targetPower * Math.sin(moveDirectionRad);
        setExtrinsicTargetPower(targetPowerX, targetPowerY);
    }

    /**
     * Sets the power proportional to the robot's top speed in reference to the robot's heading
     */
    public synchronized void setIntrinsicTargetPower(double targetPowerX, double targetPowerY) {
        setExtrinsicTargetPower(targetPowerX, targetPowerY);
        isPowerIntrinsic = true;
    }

    public synchronized void setIntrinsicTargetPower(double targetPower, Angle direction) {
        setExtrinsicTargetPower(targetPowerX, direction);
        isPowerIntrinsic = true;
    }

    /**
     * A positive power of theta refers to a counter-clockwise rotation
     */
    public synchronized void setTargetRotatePower(double targetRotatePower) {
        this.targetRotatePower = targetRotatePower;
    }
}
