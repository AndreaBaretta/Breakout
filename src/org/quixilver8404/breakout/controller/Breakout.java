package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.feedforward.ActionEventListener;
import org.quixilver8404.breakout.feedforward.Path;
import org.quixilver8404.breakout.feedforward.RobotState;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.io.*;
import java.util.List;

public class Breakout {
    public Controller controller;
    public final PowerProfile powerProfile;
    public final Path path;
    protected Vector3 lastDesiredPos;
    protected final Config config;

    public Breakout(final File foxtrotFile, final int foxtrotConfig, final List<ActionEventListener> actionEventListeners, final Config robotConfig) {
        System.out.println("====================BEGIN INITIALIZING BREAKOUT: VARIANT " + foxtrotConfig + "====================");
        config = robotConfig;
        config.set();
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        powerProfile = new PowerProfile(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y, true);
        path = Path.fromFile(foxtrotFile, foxtrotConfig, actionEventListeners);
        lastDesiredPos = new Vector3(path.startX,path.startY,path.startHeading);
        System.out.println("====================DONE INITIALIZING BREAKOUT: VARIANT " + foxtrotConfig + "====================");
    }

    public Breakout(final byte[] foxtrotFile, final int foxtrotConfig, final List<ActionEventListener> actionEventListeners, final Config robotConfig) {
        System.out.println("====================BEGIN INITIALIZING BREAKOUT: VARIANT " + foxtrotConfig + "====================");
        config = robotConfig;
        config.set();
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        powerProfile = new PowerProfile(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y, true);
        path = new Path(new ByteArrayInputStream(foxtrotFile), foxtrotConfig, actionEventListeners);
        lastDesiredPos = new Vector3(path.startX,path.startY,path.startHeading);
        System.out.println("====================DONE INITIALIZING BREAKOUT: VARIANT " + foxtrotConfig + "====================");
    }

    protected double prev_s = 0;

    public double[] iterate(final Vector3 pos, final Vector3 vel, final double dt) {
        final double s = path.calcS(pos.x, pos.y); //Get length along the path
        final double s_dot = Math.max((s - prev_s)/dt, 0); //Get velocity at which we are going along the path
        final double[] velCorrection = path.calcAccelerationCorrection(s, s_dot); //Get acceleration along the path

        final double s_tilde_dot = velCorrection[0];
        final double s_tilde_dot_dot = velCorrection[1];

//        System.out.println("s_dot: " + s_dot + "  s_tilde_dot: " + s_tilde_dot +  "  s_tilde_dot_dot: " + s_tilde_dot_dot + "  s: " + s + "  dt: " + dt*1000 + "ms");

        prev_s = s;

        final RobotState state = toBreakoutCoords(path.evaluate(s, s_tilde_dot, s_tilde_dot_dot)); //Pathing code generates headings based on Foxtrot's understanding of them. This flips them 90 degrees to the right thing.
        lastDesiredPos = toFoxtrotCoords(state.pos); //Track the last known desired position of the robot.

        final double[] correction = controller.correction(Vector3.subtractVector2(pos, state.pos),
                Vector3.subtractVector(vel, state.vel)); //Calculate the correction

        final double[] powerSettings = powerProfile.powerSetting(state.acc, state.vel, correction, pos.theta); //Calculate feedforward power setting

        final double[] frictionAdjustedPowerSettings; //Add friction adjustments to it
        if (isFinished()) {
            frictionAdjustedPowerSettings = FrictionCorrection.correction(vel, Vector3.subtractVector2(pos, state.pos), pos.theta, powerSettings, true);
//            throw new Error("Whasssup");
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
