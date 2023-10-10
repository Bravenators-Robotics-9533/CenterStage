package com.bravenatorsrobotics.eventSystem;

import java.util.ArrayList;

public class CallbackSystem {

    private ArrayList<Callback> callbacks = new ArrayList<>();

    public void addCallback(Callback callback) {
        this.callbacks.add(callback);
    }

    public void removeCallback(Callback callback) {
        this.callbacks.remove(callback);
    }

    public void fireCallback() {
        for(Callback callback : this.callbacks)
            callback.onCallback();
    }

}
