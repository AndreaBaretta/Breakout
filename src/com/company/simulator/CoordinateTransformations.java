package com.company.simulator;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class CoordinateTransformations {
    public static RealVector toFieldCoordinates(final RealVector relativeCoords, final double alpha) {
        final Array2DRowRealMatrix transform = new Array2DRowRealMatrix( new double[][]{
                {Math.cos(alpha), -Math.sin(alpha)},
                {Math.sin(alpha), Math.cos(alpha)},
        });

        final RealVector fieldCoords = transform.operate(relativeCoords);
        return fieldCoords;
    }

    public static RealVector toRelativeCoordinates(final RealVector fieldCoords, final double alpha) {
        final Array2DRowRealMatrix transform = new Array2DRowRealMatrix( new double[][]{
                {Math.cos(alpha), Math.sin(alpha)},
                {-Math.sin(alpha), Math.cos(alpha)},
        });
        final RealVector relativeCoords = transform.operate(fieldCoords);
        return relativeCoords;
    }

    public static RealVector toRelativeVelocity(final RealVector relativeCoords, final RealVector fieldVelocity) {
        final double d_alpha_dt = fieldVelocity.toArray()[2];
        final double alpha = relativeCoords.toArray()[2];


        final Array2DRowRealMatrix transform = new Array2DRowRealMatrix( new double[][]{
                {Math.cos(alpha), Math.sin(alpha), 0},
                {-Math.sin(alpha), Math.cos(alpha), 0},
                {0, 0, 1}
        });

        final Array2DRowRealMatrix d_inv_transform_d_alpha = new Array2DRowRealMatrix(new double[][] {
                {-Math.sin(alpha), -Math.cos(alpha), 0},
                {Math.cos(alpha), -Math.sin(alpha), 0},
                {0, 0, 0}
        });

        final RealVector R = relativeCoords.mapMultiplyToSelf(d_alpha_dt);

        final RealVector d_R_dt = transform.operate(
                fieldVelocity.subtract(
                        d_inv_transform_d_alpha.operate( R ) ) );
        return d_R_dt;
    }
}
