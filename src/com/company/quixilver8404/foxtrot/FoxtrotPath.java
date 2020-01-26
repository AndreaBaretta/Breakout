package com.company.quixilver8404.foxtrot;

import com.google.gson.Gson;

import com.company.quixilver8404.skystone.control.ActionSet;
import com.company.quixilver8404.skystone.control.PurePursuitPath;
import com.company.quixilver8404.skystone.control.WaypointPPPath;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class FoxtrotPath {
    private class FTPathRaw {
        public double[] xVals;
        public double[] yVals;
        public double[] headings;
        public double[] velocities;
        public int[][] actions;
    }

    private double[] xVals;
    private double[] yVals;
    private double[] headings;
    private double[] velocities;
    private Map<Integer, ArrayList<Integer>> actions = new HashMap<>();

    public FoxtrotPath(String jsonStr) {
        loadFromJsonStr(jsonStr);
    }

    public FoxtrotPath(InputStream jsonInputStream) throws IOException {
        int size = jsonInputStream.available();
        byte[] buffer = new byte[size];
        //noinspection ResultOfMethodCallIgnored
        jsonInputStream.read(buffer);
        jsonInputStream.close();
        String jsonStr = new String(buffer, StandardCharsets.UTF_8);
        loadFromJsonStr(jsonStr);
    }

    private void loadFromJsonStr(String jsonStr) {
        Gson gson = new Gson();
        FTPathRaw raw = gson.fromJson(jsonStr, FTPathRaw.class);
        xVals = raw.xVals;
        yVals = raw.yVals;
        headings = raw.headings;
        velocities = raw.velocities;

        for (int[] action : raw.actions) {
            int t = action[0];
            int actionVal = action[1];
            if (!actions.containsKey(t)) {
                actions.put(t, new ArrayList<Integer>());
            }
            actions.get(t).add(actionVal);
        }
    }

    public PurePursuitPath getPPPath() {
        return new WaypointPPPath(xVals, yVals, headings, velocities);
    }

    public ActionSet getActionSet() {
        return new ActionSet(actions);
    }

    public double startX() {
        return xVals[0];
    }

    public double startY() {
        return yVals[0];
    }

    public double startHeading() {
        return headings[0];
    }
}
