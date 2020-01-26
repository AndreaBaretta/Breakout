package com.company.quixilver8404.skystone.control;

import java.util.ArrayList;
import java.util.Map;
import java.util.TreeMap;

public class ActionSet {
    private final int[] tVals;
    private final ArrayList<Integer>[] actionVals;

    private int pendingIndex = 0;

    public ActionSet(int[] tVals, ArrayList<Integer>[] actionVals) {
        this.tVals = tVals;
        this.actionVals = actionVals;
    }

    public ActionSet(Map<Integer, ArrayList<Integer>> actions) {
        tVals = new int[actions.size()];
        actionVals = new ArrayList[actions.size()];
        TreeMap<Integer, ArrayList<Integer>> sortedActions = new TreeMap<>(actions);
        int index = 0;
        for (int tVal : sortedActions.keySet()) {
            ArrayList<Integer> curActionVals = sortedActions.get(tVal);
            tVals[index] = tVal;
            actionVals[index] = new ArrayList<>();
            actionVals[index].addAll(curActionVals);
            index++;
        }
    }

    public void runUpTo(double t, BaseRobot baseRobot) {
        while (true) {
            if (pendingIndex >= tVals.length) {
                break;
            }
            int curT = tVals[pendingIndex];
            if (curT > t) {
                break;
            }
            for (Object actionVal : actionVals[pendingIndex]) {
                int actionID = (Integer) actionVal;
//                AutonActions.runAction(actionID, baseRobot);
            }
            pendingIndex++;
        }
    }

    public void runRemaining(BaseRobot baseRobot) {
        runUpTo(Integer.MAX_VALUE, baseRobot);
    }

    public void reset() {
        pendingIndex = 0;
    }
}
