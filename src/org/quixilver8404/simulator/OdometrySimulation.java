package org.quixilver8404.simulator;

import com.opencsv.CSVIterator;
import com.opencsv.CSVReader;
import com.opencsv.exceptions.CsvValidationException;
import org.lwjgl.opengl.GL11;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;

public class OdometrySimulation {

    public static final double ODOMETRY_ENCODER_M_PER_TICK = 0.035*Math.PI/8192 * 3.13055/3.105;
    public static final Vector3 ODOMETRY_1_POSITION = new Vector3(-0.1115,-0.0525,-Math.PI/2);
    public static final Vector3 ODOMETRY_2_POSITION = new Vector3(0.0065,0.185,Math.PI);
    public static final Vector3 ODOMETRY_3_POSITION = new Vector3(0.1115,0.0155,-Math.PI/2);
    public static final Vector3[] ODOMETRY_WHEEL_SETUP = new Vector3[]{ODOMETRY_1_POSITION, ODOMETRY_2_POSITION, ODOMETRY_3_POSITION};

    public static void main(final String[] args) throws IOException, CsvValidationException, InterruptedException {
        Display ui = new Display(1000, 1000, 100);
        ui.init();

        final File file1 = new File("/home/andrea/Desktop/run-3.csv");
        final CSVIterator odoFile1 = new CSVIterator(new CSVReader(new FileReader(file1)));

        odoFile1.next();
        String[] line = odoFile1.next();
        final Odometry odometry = new Odometry(ODOMETRY_WHEEL_SETUP, new Vector3(Double.parseDouble(line[3]),Double.parseDouble(line[4]), Angles.toStandardHalf(Double.parseDouble(line[5]))));
        final Odometry odometryImu = new Odometry(ODOMETRY_WHEEL_SETUP, new Vector3(Double.parseDouble(line[3]),Double.parseDouble(line[4]), Angles.toStandardHalf(Double.parseDouble(line[5]))));

        final double initialImu = Double.parseDouble(line[6]);
        long t1 = Long.parseLong(line[7]);

        double imu = Angles.toStandardHalf(Double.parseDouble(line[5]));
        double lastRawImu = initialImu;

        while (odoFile1.hasNext()) {
            line = odoFile1.next();
            final long t2 = Long.parseLong(line[7]);

            final double odo1 = Integer.parseInt(line[0]) * ODOMETRY_ENCODER_M_PER_TICK;
            final double odo2 = Integer.parseInt(line[1]) * ODOMETRY_ENCODER_M_PER_TICK;
            final double odo3 = Integer.parseInt(line[2]) * ODOMETRY_ENCODER_M_PER_TICK;
            odometry.update(new double[]{odo1, odo2, odo3}, (t2 - t1)/1000d, odometry.pos.theta);

            t1 = t2;

            final double t265x = Double.parseDouble(line[3]);
            final double t265y = Double.parseDouble(line[4]);
            final double t265h = Double.parseDouble(line[5]);

            final double rawImu = Double.parseDouble(line[6]);

            final double dTheta = Angles.toStandardHalf(rawImu - lastRawImu);
            System.out.println("rawImu: " + rawImu + "  lastRawImu: " + lastRawImu + "  dTheta: " + dTheta);
            imu += dTheta;
            lastRawImu = rawImu;

            odometryImu.update(new double[]{odo1, odo2, odo3}, (t2 - t1)/1000d, odometryImu.pos.theta);


            odometryImu.setPosition(new Vector3(odometryImu.getRawPosition().x, odometryImu.getRawPosition().y, imu));

//            System.out.println("Data: " + Arrays.toString(line));
//            System.out.println("odoPos: " + odometry.pos.toString());

//            kinematics.setFieldPos(odometry.pos);
            GL11.glClear(GL11.GL_COLOR_BUFFER_BIT);
            ui.setBackground(new double[]{255,255,255});
            ui.drawCircle(0, 0, 0.03, 100, new double[]{255,0,0});
            ui.drawCompass(odometry.pos.theta + Math.PI/2, odometry.pos.x, odometry.pos.y, 50, new double[]{0,0,255});
            ui.drawCompass(t265h + Math.PI/2, t265x, t265y, 50, new double[]{0,255,0});
            ui.drawCompass(odometryImu.pos.theta + Math.PI/2, odometryImu.pos.x, odometryImu.pos.y, 50, new double[]{255,0,0});
            System.out.println("Odometry-IMU: " + odometryImu.pos.toString() + "  Raw Odometry: " + odometry.pos.toString());

//            kinematics.ui.drawCompassPixel(breakout.getLastDesiredPos().theta, 0, 0, 50);
            ui.update();

            Thread.sleep(10);
            System.out.println();
        }
    }

}
