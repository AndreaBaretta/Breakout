package org.quixilver8404.breakout.util;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.NelderMeadSimplex;

import java.util.*;

public class Optimizer {

    final MultivariateFunction func;
    final NelderMeadSimplex nms;
    final Comparator<PointValuePair> comparator;

    public Optimizer(final MultivariateFunction func, final double[] startPos, final boolean maximize) {
        this.func = func;
        nms = new NelderMeadSimplex(startPos.length);
        nms.build(startPos);
        final int mul = maximize ? 1 : -1;
        comparator = new Comparator<PointValuePair>() {
            @Override
            public int compare(final PointValuePair p1, final PointValuePair p2) {
                if (p1.getValue() > p2.getValue()) {
                    return -1 * mul;
                } else if (p1.getValue() < p2.getValue()) {
                    return 1 * mul;
                }  else {
                    return 0;
                }
            }
        };
    }

    public PointValuePair getBestPoint() {
        return Collections.max(new ArrayList<PointValuePair>(Arrays.asList(nms.getPoints())), comparator);
    }

    public PointValuePair iterateAndReturn() {
        nms.iterate(func, comparator);
        return getBestPoint();
    }

    public void iterate() {
        nms.iterate(func, comparator);
    }

    public PointValuePair iterateForSteps(final int steps) {
        for (int i = 0; i < steps; i++) {
            iterate();
        }
        return getBestPoint();
    }

}
