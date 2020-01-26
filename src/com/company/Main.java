package com.company;

import com.company.quixilver8404.skystone.opmode.test.FoxtrotTest;
import com.company.simulator.*;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.lwjgl.*;
import com.company.simulator.Motor.LowPassFilter;

import java.util.Arrays;

import static com.company.simulator.MecanumKinematics.*;

public class Main {

    public Main() {

    }

    public static void run(final MecanumKinematics kinematics) {
        System.out.println("Hello LWJGL " + Version.getVersion() + "!");
//        window1.init();

        int count = 0;

        long t_new = System.nanoTime();
        long t_old = t_new;
        final FoxtrotTest foxtrotTest = new FoxtrotTest();
        double[] power = new double[]{1,1,1,1};

        //for (int i = 0; i < 1000; i++) {
        while(/*window1.isRunning()*/ true) {
            t_new = System.nanoTime();
            final double dt = (t_new - t_old)/1e9d;
            t_old = t_new;

            assert dt > 0.0d;
            System.out.println("dt = " + dt);
//            t1 = System.currentTimeMillis();
//            try {
//                foxtrotTest.runOpMode(kinematics);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            kinematics.updatePresetFrequency(power);
            kinematics.update(power, dt);
            count++;
            System.out.println("count = " + count);
//            if (count == 100) {
//                power = new double[]{0,0,0,0};
//            }
//            GL11.glClear(GL11.GL_COLOR_BUFFER_BIT);
//            GLFW.glfwSwapBuffers(window1.window);
//            try {
//                foxtrotTest.runOpMode();
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

//            window1.setBackground(new double[] {1,1,1});

//            window1.drawPolygon(new double[][] {
//                    {0+count,0},{100+count,0},{100+count,100},{0+count,100} }, new double[] {255,0,0});
//
//            window1.drawCircle(-100,-100, 50, 100, new double[] {0,255,0});
            final VectorXYAlpha pos = kinematics.getFieldPos();
            final VectorXYAlpha vel = kinematics.getFieldVel();
//            window1.drawRobot(pos.x,pos.y, pos.phi, kinematics.width, kinematics.length, 100);
//            window1.update();


        }



//        window1.terminate();
    }

    public static void main(String[] args) {
//        System.out.println();
//        new HelloWorld().run();

//        run();
//        int power = 0;
//        double vel = 0;
//        final MotorOld motorOld = new MotorOld(1146, 724, 0);
//        final MecanumWheelOld mecanumWheelOld = new MecanumWheelOld(10,10, Math.PI/4, motorOld, 1);
//        final double wheelDiam = 75*Math.PI;
//        final MecanumDriveTrainOld drive = new MecanumDriveTrainOld(new MecanumWheelOld[] {
//                new MecanumWheelOld(100,100, Math.PI/4, motorOld.clone(), 1),
//                new MecanumWheelOld(-100, 100, -Math.PI/4, motorOld.clone(), 1),
//                new MecanumWheelOld(-100, -100, Math.PI/4, motorOld.clone(), 1),
//                new MecanumWheelOld(100, -100, -Math.PI/4, motorOld.clone(), 1)
//        }, 13.6, 1, 457.2, 457.2, 50);


        final Motor motor = new Motor(2.1, 31.4, new LowPassFilter(20, 0));
        final double phi = Math.PI/4;
        final double m = 20;
        final double[][] u_Pi = new double[][] {
                {-Math.sin(phi), Math.cos(phi)},
                {Math.sin(phi), Math.cos(phi)},
                {-Math.sin(phi), Math.cos(phi)},
                {Math.sin(phi), Math.cos(phi)}
        };
        final MecanumWheel wheel = new MecanumWheel(motor.clone(), 0.1, 1, 1, phi, m, u_Pi[0]);
        final double r = 37.5d/1000d;
        final double X = 0.4572/2d;
        final double Y = 0.4572/2d;
        final MecanumWheel[] wheels = new MecanumWheel[] {
                new MecanumWheel(motor.clone(), r, X, Y, -phi, m, u_Pi[0]),
                new MecanumWheel(motor.clone(), r, -X, Y, phi, m, u_Pi[1]),
                new MecanumWheel(motor.clone(), r, -X, -Y, -phi, m, u_Pi[2]),
                new MecanumWheel(motor.clone(), r, X, -Y, phi, m, u_Pi[3])
        };

        Display window1 = new Display(1000, 1000);
        window1.init();

        final MecanumKinematics kinematics = new MecanumKinematics(wheels, 50, m, X*2d, Y*2d, new VectorXYAlpha(0,0,0), new VectorXYAlpha(0,0,0), window1);
//        double curSpeed = 31.4;
        int count = 0;

        //while (true) {
            run(kinematics);
//            kinematics.updatePresetFrequency(new double[] {0,0,0,0});
//            break;
//            kinematics.updatePresetFrequency(new double[]{1,-1,-1,1});
//            System.out.println("Velocity: " + kinematics.getFieldVel());
//            count++;
//            if (count == 100) {
//                break;
//            }
//            for (final MecanumWheel temp : wheels) {
//                System.out.println("Wheel force: " + Arrays.toString(temp.updateForce(1,0, 0.02)));
//            }



//            break;
//            try {
//                TimeUnit.MILLISECONDS.sleep((long) 5);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            motor.update(power, vel, 0.02);
//            vel = motor.getWheelVelocity()*0.9;
//            System.out.println("Power: " + power + "  Wheel velocity: " + motor.getWheelVelocity() + "  Chassis velocity: " + vel);
//            if (power<100) {
//                power += 1;
//            }
//            mecanumWheel.iterate(power, vel, 0.02);
//            vel = mecanumWheel.getWheelVelocity();

//            drive.iteratePresetFrequency();

        //}
    }

}