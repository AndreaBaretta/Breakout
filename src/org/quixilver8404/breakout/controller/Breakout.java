package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.feedforward.ActionEventListener;
import org.quixilver8404.breakout.feedforward.Path;
import org.quixilver8404.breakout.feedforward.RobotState;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.io.File;
import java.io.InputStream;
import java.util.List;

public class Breakout {
    public Controller controller;
    public final PowerProfile powerProfile;
    public final Path path;
    protected Vector3 lastDesiredPos;
    protected final Config config;

    public Breakout(final File foxtrotFile, final int foxtrotConfig, final List<ActionEventListener> actionEventListeners, final Config robotConfig) {
        System.out.println("Hey hey ho ho");
        System.out.println("====================BEGIN INITIALIZING BREAKOUT====================");
        config = robotConfig;
        config.set();
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        powerProfile = new PowerProfile(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y, true);
        path = Path.fromFile(foxtrotFile, foxtrotConfig, actionEventListeners);
        lastDesiredPos = new Vector3(path.startX,path.startY,path.startHeading);
        System.out.println("====================DONE INITIALIZING BREAKOUT====================");
    }

    public Breakout(final InputStream foxtrotFile, final int foxtrotConfig, final List<ActionEventListener> actionEventListeners, final Config robotConfig) {
        System.out.println("====================BEGIN INITIALIZING BREAKOUT====================");
        config = robotConfig;
        config.set();
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        powerProfile = new PowerProfile(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y, true);
        path = new Path(foxtrotFile, foxtrotConfig, actionEventListeners);
        lastDesiredPos = new Vector3(path.startX,path.startY,path.startHeading);
        System.out.println("====================DONE INITIALIZING BREAKOUT====================");
    }

    protected double prev_s = 0;

    public double[] iterate(final Vector3 pos, final Vector3 vel, final double dt) {
        final double s = path.calcS(pos.x, pos.y); //Get length along the path
        final double s_dot = (s - prev_s)/dt; //Get velocity at which we are going along the path
        final double s_dot_dot = path.calcAccelerationCorrection(s, s_dot); //Get acceleration along the path

        prev_s = s;

        final RobotState state = toBreakoutCoords(path.evaluate(s, s_dot, s_dot_dot)); //Pathing code generates headings based on Foxtrot's understanding of them. This flips them 90 degrees to the right thing.
        lastDesiredPos = toFoxtrotCoords(state.pos); //Track the last known desired position of the robot.

        final double[] correction = controller.correction(Vector3.subtractVector2(pos, state.pos),
                Vector3.subtractVector(vel, state.vel)); //Calculate the correction

        final double[] powerSettings = powerProfile.powerSetting(state.acc, state.vel, correction, pos.theta); //Calculate feedforward power setting

        final double[] frictionAdjustedPowerSettings; //Add friction adjustments to it
        if (isFinished()) {
            frictionAdjustedPowerSettings = FrictionCorrection.correction(vel, Vector3.subtractVector2(pos, state.pos), pos.theta, powerSettings, true);
        } else {
            frictionAdjustedPowerSettings = FrictionCorrection.correction(vel, new Vector3(0,0,0), pos.theta, powerSettings, false);
        }

        return frictionAdjustedPowerSettings;
//        return powerSettings;
    }

    public boolean isFinished() {
        return path.isFinished();
    }

    public Vector3 getLastDesiredPos() {
        return lastDesiredPos;
    }

    public static RobotState toBreakoutCoords(final RobotState state) {
        return new RobotState(
                new Vector3(state.pos.x, state.pos.y, state.pos.theta-Math.PI/2),
                state.vel,
                state.acc
        );
    }

    public static Vector3 toFoxtrotCoords(final Vector3 pos) {
        return new Vector3(pos.x, pos.y, pos.theta+Math.PI/2);
    }

    public void setVoltage(final double voltage) {
        config.setVoltage(voltage);
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
    }
}
