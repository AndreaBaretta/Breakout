package org.quixilver8404;

public class RobotParams {
    final double maxAcc;
    final double maxVel;
    final double maxRotAcc;
    final double maxRotVel;

    public RobotParams(final double maxAcc, final double maxVel, final double maxRotAcc, final double maxRotVel) {
        this.maxAcc = maxAcc;
        this.maxVel = maxVel;
        this.maxRotAcc = maxRotAcc;
        this.maxRotVel = maxRotVel;
    }
}
