package com.bravenatorsrobotics.multiComponentSystem;

import com.bravenatorsrobotics.components.ScissorLiftComponent;
import com.bravenatorsrobotics.components.SuspensionLiftComponent;

public class SuspensionMultiComponentSystem {

    private final SuspensionLiftComponent suspensionLiftComponent;
    private final ScissorLiftComponent scissorLiftComponent;

    private enum State {

        PRE_STANDBY,
        RAISING,
        RAISED_STANDBY,
        SUSPENDING,
        SUSPENDED

    }

    private State state = State.PRE_STANDBY;
    private boolean shouldExpedite = false;

    public SuspensionMultiComponentSystem(SuspensionLiftComponent suspensionLiftComponent, ScissorLiftComponent scissorLiftComponent) {

        this.suspensionLiftComponent = suspensionLiftComponent;
        this.scissorLiftComponent = scissorLiftComponent;

    }

    public void update() {

        switch (this.state) {

            case PRE_STANDBY -> {
                if(shouldExpedite) {
                    this.state = State.RAISING;
                    this.shouldExpedite = false;
                }
            }

            case RAISING -> {



            }

            case RAISED_STANDBY -> {
                if(shouldExpedite) {
                    this.state = State.SUSPENDING;
                    this.shouldExpedite = false;
                }
            }

            case SUSPENDING -> {

            }

            case SUSPENDED -> {

            }

        }

    }

    public void onTrigger() {

        State previousState = this.state;

        this.state = switch (this.state) {
            case PRE_STANDBY -> State.RAISING;
            case RAISED_STANDBY -> State.SUSPENDED;

            default -> previousState;
        };

        if(this.state == previousState)
            this.shouldExpedite = true;

    }

}
