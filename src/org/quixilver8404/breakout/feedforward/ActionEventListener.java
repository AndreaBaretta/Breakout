package org.quixilver8404.breakout.feedforward;

public class ActionEventListener {
    public final int action;
    public final ActionFunction actionFunction;
    public ActionEventListener(final int action, final ActionFunction actionFunction) {
        this.action = action;
        this.actionFunction = actionFunction;
    }

    public void run() {
        actionFunction.run();
    }

    public String toString() {
        return "action=" + action;
    }
}
