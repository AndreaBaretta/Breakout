package org.quixilver8404.breakout.feedforward;

import java.util.List;
import java.util.Set;

public interface ActionPoint {
    public double getS();

    public Set<Integer> getActions();

    public List<ActionEventListener> getActionEventListeners();

    public void runActions();

}
