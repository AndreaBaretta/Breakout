package com.company.simulator;

import org.apache.commons.math3.linear.*;

public class CoordinateTransformationsOld {

    public static RealVector toRelativeCoordinates(final RealVector fieldCoords) {
        final double alpha = fieldCoords.toArray()[2];
        final Array2DRowRealMatrix transform = new Array2DRowRealMatrix( new double[][]{
                {Math.cos(alpha), -Math.sin(alpha), 0},
                {Math.sin(alpha), Math.cos(alpha), 0},
                {0, 0, 1}
        });

        final RealVector relativeCoords = transform.operate(fieldCoords);
        return relativeCoords;
    }

    public static RealVector toFieldCoordinates(final RealVector relativeCoords) {
        final double alpha = relativeCoords.toArray()[2];
        final Array2DRowRealMatrix transform = new Array2DRowRealMatrix( new double[][]{
                {Math.cos(alpha), Math.sin(alpha), 0},
                {-Math.sin(alpha), Math.cos(alpha), 0},
                {0, 0, 1}
        });

        final RealVector fieldCoords = transform.operate(relativeCoords);
        return fieldCoords;
    }

    public static RealVector toRelativeVelocity(final RealVector relativeCoords, final RealVector fieldVelocity) {
        final double d_alpha_dt = fieldVelocity.toArray()[2];
        final double alpha = relativeCoords.toArray()[2];


        final Array2DRowRealMatrix transform = new Array2DRowRealMatrix( new double[][]{
                {Math.cos(alpha), -Math.sin(alpha), 0},
                {Math.sin(alpha), Math.cos(alpha), 0},
                {0, 0, 1}
        });

        final Array2DRowRealMatrix d_inv_transform_d_alpha = new Array2DRowRealMatrix(new double[][] {
                {-Math.sin(alpha), Math.cos(alpha), 0},
                {-Math.cos(alpha), -Math.sin(alpha), 0},
                {0, 0, 0}
        });

        final RealVector R = relativeCoords.mapMultiplyToSelf(d_alpha_dt);

        final RealVector d_R_dt = transform.operate(
                fieldVelocity.subtract(
                        d_inv_transform_d_alpha.operate( R ) ) );
        return d_R_dt;
    }

    public static RealVector toFieldVelocity(final RealVector fieldCoords, final RealVector relativeVelocity) {
        final double d_alpha_dt = relativeVelocity.toArray()[2];
        final double alpha = fieldCoords.toArray()[2];


        final Array2DRowRealMatrix inv_transform = new Array2DRowRealMatrix(new double[][]{
                {-Math.sin(alpha), -Math.cos(alpha), 0},
                {Math.cos(alpha), -Math.sin(alpha), 0},
                {0, 0, 1}
        });

//        final RealMatrix d_inv_transform_d_alpha = MatrixUtils.inverse(d_transform_d_alpha);

        final Array2DRowRealMatrix d_transform_d_alpha = new Array2DRowRealMatrix(new double[][] {
                {-Math.sin(alpha), -Math.cos(alpha), 0},
                {Math.cos(alpha), -Math.sin(alpha), 0},
                {0, 0, 0}
        });

        final RealVector R = fieldCoords.mapMultiplyToSelf(d_alpha_dt);

        final RealVector d_r_dt = inv_transform.operate(
                relativeVelocity.subtract(
                        d_transform_d_alpha.operate(R) ) );
        return d_r_dt;
    }
}

