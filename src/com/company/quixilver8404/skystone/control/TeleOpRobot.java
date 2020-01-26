package com.company.quixilver8404.skystone.control;

import com.company.simulator.MecanumKinematics;

import com.company.quixilver8404.skystone.util.measurement.Angle;
import com.company.quixilver8404.skystone.util.measurement.Distance;
import com.company.quixilver8404.skystone.util.measurement.Pose2D;

public class TeleOpRobot extends BaseRobot {

    private boolean driveIntrinsic = true;

    private boolean driveIntrinsicFlipped = false;

    public TeleOpRobot(Pose2D startPose, MecanumKinematics kinematics) {
        super(startPose, kinematics);
    }

    public TeleOpRobot(Distance startX, Distance startY, Angle startHeading, MecanumKinematics kinematics) {
        this(new Pose2D(startX, startY, startHeading), kinematics);
    }

    public TeleOpRobot(double startXInches, double startYInches, double startHeadingDegs, MecanumKinematics kinematics) {
        this(new Distance(startXInches, Distance.Unit.INCHES), new Distance(startYInches, Distance.Unit.INCHES), new Angle(startHeadingDegs, Angle.Unit.DEGREES), kinematics);
    }

    /**
     * Commands the robot to move relative to the field based on the current drive mode,
     * intrinsic or extrinsic (default).
     * Unlike the DriveModule, this method treats positive y as straight forward
     * A positive rotatePower refers to a counter-clockwise rotation.
     * The power is proportional to the max velocity
     */
    public void setTransformedTargetMovePower(double movePower, Angle moveDirection) {
        Angle newMoveDirection;
        if (driveIntrinsic) {
            newMoveDirection = Angle.subtractAngles(moveDirection, new Angle(90, Angle.Unit.DEGREES));
        } else {
            newMoveDirection = moveDirection;
        }

        double moveDirectionRad = newMoveDirection.getStandard(Angle.Unit.RADIANS);
        double targetPowerX = movePower * Math.cos(moveDirectionRad);
        double targetPowerY = movePower * Math.sin(moveDirectionRad);

        if (driveIntrinsic) {
            if (isDriveIntrinsicFlipped()) {
                driveModule.setIntrinsicTargetPower(-targetPowerX, -targetPowerY);
            } else {
                driveModule.setIntrinsicTargetPower(targetPowerX, targetPowerY);
            }
        } else {
            driveModule.setExtrinsicTargetPower(targetPowerX, targetPowerY);
        }
    }

    /**
     * Gets whether the current drive mode is intrinsic
     */
    public boolean isDriveIntrinsic() {
        return driveIntrinsic;
    }

    /**
     * Whether the front and back of the robot is flipped
     */
    public boolean isDriveIntrinsicFlipped() {
        return driveIntrinsicFlipped;
    }

    /**
     * Sets the current drive mode to intrinsic
     */
    public void setDriveIntrinsic() {
        driveIntrinsic = true;
    }

    /**
     * Sets the current drive mode to extrinsic
     */
    public void setDriveExtrinsic() {
        driveIntrinsic = false;
    }

    public void toggleDriveIntrinsicFlipped() {
        driveIntrinsicFlipped = !driveIntrinsicFlipped;
    }
}
