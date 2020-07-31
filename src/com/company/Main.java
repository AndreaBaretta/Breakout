package com.company;

import com.company.feedforward.*;
import com.company.simulator.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Main {

    public static void main(String[] args) {
        System.out.println("Howdy world");

        Display window1 = new Display(1000, 1000, 100);
        window1.init();

        final double v_max = 1.17749;
        final double a_max = 11.2;

        final double rX = 0.4572/2;
        final double rY = 0.4572/2;
        final double Tmax = 2.1;
        final double omegamax = 31.4;
        final double R = 37.5/1000;
        final double m = 20;
        final double J = MecanumKinematics.FindMomentOfInertia(0.5, 0.5, m);


        final MecanumKinematics kinematics = new MecanumKinematics(
                50, m, 0.5, 0.5,
                new Vector3(0,0,0), new Vector3(0.1, 0.3,0.1),
                window1,
                J, rX, rY, Tmax, R, omegamax);

        final PowerProfile powerProfile = new PowerProfile(m, R, J, omegamax, Tmax, rX, rY, false);
        final Controller controller = new Controller(Controller.computeK(m, R, J, omegamax, Tmax, rX, rY));

        final FeedForwardTest feedForwardTest = new FeedForwardTest();

        double error_xy = 0;
        double error_alpha = 0;
        double t = 0;
        final double updateControllerEveryHz = 10;
        double counter = 0;
        double[] prevCorrection = new double[]{0,0,0};
        double t1 = System.currentTimeMillis()/(double)1000;
        double t2 = System.currentTimeMillis()/(double)1000;
        final double time_limit = 5;
        double maxAccel = 0;

        final List<AnchorPoint> anchorPoints = new ArrayList<AnchorPoint>();
        anchorPoints.add(new AnchorPoint(0, 0, 0, AnchorPoint.Heading.FRONT, 0, 0, null, 0,
                1, new Point2D(0, 1), 0, null, new Point2D(1, 1), Config.MAX_VELOCITY, true, false));
        anchorPoints.add(new AnchorPoint(2, 3, 0, AnchorPoint.Heading.FRONT, 1, 1, new Point2D(2, 2), Math.PI,
                1, new Point2D(2,2), 0, new Point2D(1, 2), new Point2D(3,2), Config.MAX_VELOCITY, false, false));
        anchorPoints.add(new AnchorPoint(4, 0, 0, AnchorPoint.Heading.FRONT, 0, 1, new Point2D(4,1), Math.PI,
                         0, null, 0, new Point2D(3,1), null, Config.MAX_VELOCITY, false, true));
        final Path path = new Path(anchorPoints);

        double prev_s = 0;
        double prev_s_dot = 0;
        while (true) {
//            System.out.println("tan(pi/2): " + Math.tan(Math.PI/2));
//            System.out.println(Math.PI);
//            final double dt = 0.0001; //Preset time
            double dt = t2 - t1;
//            double dt = 0.001;

            final double s;
            final double s_dot;
            final double s_dot_dot;
            if (dt == 0) {
//                throw new Error("dt = 0");
//                dt = 0.001;
                s = path.calcS(kinematics.getFieldPos().x, kinematics.getFieldPos().y);
                s_dot = 0;
                s_dot_dot = 0;
//                System.out.println("Set to 0");
            } else {
                s = path.calcS(kinematics.getFieldPos().x, kinematics.getFieldPos().y);
                s_dot = (s - prev_s)/dt;
                s_dot_dot = path.calcAccelerationCorrection(s, s_dot);
            }
            t1 = t2;
//            final Vector3 pos = feedForwardTest.getPosition(t);
//            final Vector3 vel = feedForwardTest.getVelocity(t);
//            final Vector3 acc = feedForwardTest.getAcceleration(t);
//            final Vector3 pos = path.evaluate(t, 0, 0).pos;
//            final double s = path.calcS(kinematics.getFieldPos().x, kinematics.getFieldPos().y);
//            final double s_dot = (s - prev_s)/dt;
//            final double s_dot_dot = path.calcAccelerationCorrection(s, s_dot);
            prev_s = s;
            prev_s_dot = s_dot;
            final RobotState state = path.evaluate(s, s_dot, s_dot_dot);

            final double[] correction;
            if (counter == updateControllerEveryHz) {
                correction = controller.correction(Vector3.subtractVector(kinematics.getFieldPos(), state.pos),
                        Vector3.subtractVector(kinematics.getFieldVel(), state.vel));
                prevCorrection = correction;
                counter = 0;
            } else {
                correction = prevCorrection;
            }

            final double[] powerSettings = powerProfile.powerSetting(state.acc, state.vel, correction, kinematics.getFieldPos().theta);
//            System.out.println(Arrays.toString(powerSettings));
            kinematics.update(powerSettings, dt);
//            kinematics.update(new double[]{1, -1, 1, -1}, dt);

//            System.out.println("Desired pos: " + pos.toString() + " actual pos: " + kinematics.getFieldPos() +
//                    " || Desired vel: " + vel.toString() + " actual vel: " + kinematics.getFieldVel() +
//                    " || Desired acc: " + acc.toString() + " actual acc: " + kinematics.getFieldAcc() +
//                    " || Power settings: " + Arrays.toString(powerSettings));

//            System.out.println(path.mainSegments.get(0).circleSegment1.theta1_);

//            kinematics.ui.setBackground(new double[]{255,255,255});

            kinematics.ui.drawCircle(state.pos.x, state.pos.y, 0.05, 100, new double[]{255,0,0});
//            kinematics.ui.drawCircle(path.evaluate(t, 0, 0).pos.x, path.evaluate(t, 0, 0).pos.y, 0.05, 100, new double[]{255,0,0});
            kinematics.ui.drawCircle(0, 0, 0.05, 100, new double[]{0,255,0});
//            kinematics.ui.drawCompassPixel(path.evaluate(t, 0, 0).pos.theta, 400, -400, 50);
            kinematics.ui.drawCompassPixel(state.pos.theta, 400, -400, 50);

            kinematics.ui.update();
//            System.out.println("Current accel: " + Math.abs(kinematics.getFieldAcc().y));
//            if (Math.abs(kinematics.getFieldAcc().y) > maxAccel) {
//                maxAccel = Math.abs(kinematics.getFieldAcc().y);
//                System.out.println("maxAccel: " + maxAccel);
//            }
//            System.out.println(kinematics.getFieldAcc());
            counter++;
            t+=dt;
            error_xy += Math.sqrt(Math.hypot(state.pos.x-kinematics.getFieldPos().x, state.pos.y-kinematics.getFieldPos().y))*dt;
            error_alpha += (state.pos.theta-kinematics.getFieldPos().theta)*dt;
            t2 = System.currentTimeMillis()/(double)1000;
            if (counter >= 20000) {
                kinematics.ui.terminate();
//                System.out.println("Average error in x-y: " + error_xy/t);
//                System.out.println("Average error in alpha: " + error_alpha/t);
                break;
            }
        }
    }
}