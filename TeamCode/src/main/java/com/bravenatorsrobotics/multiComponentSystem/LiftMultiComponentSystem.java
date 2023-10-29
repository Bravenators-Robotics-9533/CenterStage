package com.bravenatorsrobotics.multiComponentSystem;

import com.bravenatorsrobotics.components.LiftComponent;
import com.bravenatorsrobotics.components.PixelPouchComponent;
import com.bravenatorsrobotics.components.SwingArmComponent;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftMultiComponentSystem {

    private final LiftComponent liftComponent;
    private final SwingArmComponent swingArmComponent;
    private final PixelPouchComponent pixelPouchComponent;

    private int targetLiftPosition;

    private enum SystemPosition {
        SCORING_POSITION,
        INTAKE_POSITION
    }

    private enum State {

        AT_BOTTOM,
        LIFTING,
        AWAITING_LIFT,
        SWINGING_ARM,
        AWAITING_ARM_SWING,
        AT_TOP

    }

    private State state = State.AT_BOTTOM;

    private SystemPosition targetPosition = SystemPosition.INTAKE_POSITION;

    public LiftMultiComponentSystem(LiftComponent liftComponent, SwingArmComponent swingArmComponent, PixelPouchComponent pixelPouchComponent) {
        this.liftComponent = liftComponent;
        this.swingArmComponent = swingArmComponent;
        this.pixelPouchComponent = pixelPouchComponent;
    }

    public void goToScoringPosition(int liftPosition) {

        this.targetPosition = SystemPosition.SCORING_POSITION;
        this.targetLiftPosition = liftPosition;

    }

    public void goToIntakePosition() {

        this.targetPosition = SystemPosition.INTAKE_POSITION;

    }

    public void update() {

            switch (this.targetPosition) {

                case INTAKE_POSITION:
                    updateIntakeStateMachine();
                    break;
                case SCORING_POSITION:
                    updateScoringStateMachine();
                    break;

            }

    }

    private final ElapsedTime timeoutTimer = new ElapsedTime();

    private static final int SAFE_TARGET_TOLERANCE = 10;

    private void updateScoringStateMachine() {

        switch (this.state) {

            case AT_BOTTOM:
                this.state = State.LIFTING;

                break;

            case LIFTING:
                liftComponent.goToEncoderPosition(targetLiftPosition, LiftComponent.LIFT_SPEED);
                this.state = State.AWAITING_LIFT;

                this.timeoutTimer.reset();

                break;

            case AWAITING_LIFT:
                if(this.liftComponent.getCurrentPosition() >= LiftComponent.LIFT_POSITION_ARM_SAFE - SAFE_TARGET_TOLERANCE || this.timeoutTimer.seconds() > 2) {

                    if (timeoutTimer.seconds() > 2)
                        System.err.println("TIMEOUT HIT");

                    this.state = State.SWINGING_ARM;
                }

                break;

            case SWINGING_ARM:
                this.swingArmComponent.swingOut();
                this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_RELEASE_POSITION);

                this.timeoutTimer.reset();

                this.state = State.AWAITING_ARM_SWING;

                break;

            case AWAITING_ARM_SWING:
                if ((!this.swingArmComponent.isBusy() &&
                        this.pixelPouchComponent.getPouchPosition() == PixelPouchComponent.POUCH_RELEASE_POSITION)
                        || timeoutTimer.seconds() > 2) {

                    if (timeoutTimer.seconds() > 2)
                        System.err.println("TIMEOUT HIT");

                    this.state = State.AT_TOP;

                }

                break;

            case AT_TOP: // Done
                break;

        }

    }

    private void updateIntakeStateMachine() {

        switch (this.state) {

            case AT_TOP:
                this.state = State.SWINGING_ARM;

                break;

            case SWINGING_ARM:
                this.swingArmComponent.goToIntakePosition();
                this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_INTAKE_POSITION);

                this.timeoutTimer.reset();

                this.state = State.AWAITING_ARM_SWING;

                break;

            case AWAITING_ARM_SWING:

                if ((!this.swingArmComponent.isBusy() &&
                        this.pixelPouchComponent.getPouchPosition() == PixelPouchComponent.POUCH_RELEASE_POSITION)
                        || timeoutTimer.seconds() > 2) {

                    if (timeoutTimer.seconds() > 2)
                        System.err.println("TIMEOUT HIT");

                    this.state = State.LIFTING;

                }

                break;

            case LIFTING:

                liftComponent.goToEncoderPosition(LiftComponent.LIFT_POSITION_INTAKE, LiftComponent.LIFT_SPEED);
                this.state = State.AWAITING_LIFT;

                this.timeoutTimer.reset();

                break;

            case AWAITING_LIFT:

                if(this.liftComponent.isBusy() || this.timeoutTimer.seconds() > 2) {

                    if(this.timeoutTimer.seconds() > 2) {
                        System.err.println("TIMEOUT HIT");
                    }

                    this.state = State.AT_BOTTOM;

                }

                break;

            case AT_BOTTOM:
                break;


        }

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Target Position", targetPosition.name());
        telemetry.addData("State", state.name());
    }
}
