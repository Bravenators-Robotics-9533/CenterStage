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
        INTERNAL_LIFTING,
        AWAITING_INTERNAL_LIFT,
        SWINGING_ARM,
        AWAITING_ARM_SWING,
        EXTERNAL_LIFTING,
        AWAITING_EXTERNAL_LIFT,
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

    private void updateScoringStateMachine() {

        switch (this.state) {

            case AT_BOTTOM:
                this.state = State.INTERNAL_LIFTING;

                break;

            case INTERNAL_LIFTING:
                liftComponent.goToEncoderPosition(LiftComponent.LIFT_POSITION_ARM_SAFE, LiftComponent.LIFT_SPEED);
                this.state = State.AWAITING_INTERNAL_LIFT;

                this.timeoutTimer.reset();

                break;

            case AWAITING_INTERNAL_LIFT:
                if(!this.liftComponent.isBusy() || timeoutTimer.seconds() > 2) {

                    if (timeoutTimer.seconds() > 2)
                        System.err.println("TIMEOUT HIT");

                    this.state = State.SWINGING_ARM;
                }

                break;

            case SWINGING_ARM:
                this.swingArmComponent.goToScoringPosition();
                this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_SCORING_POSITION);

                this.timeoutTimer.reset();

                this.state = State.AWAITING_ARM_SWING;

                break;

            case AWAITING_ARM_SWING:
                if (!this.swingArmComponent.isMotorBusy() && this.pixelPouchComponent.servoAtScoringPosition()) {

                    if (timeoutTimer.seconds() > 2)
                        System.err.println("TIMEOUT HIT");

                    this.state = State.EXTERNAL_LIFTING;

                }

                break;

            case EXTERNAL_LIFTING:
                liftComponent.goToEncoderPosition(targetLiftPosition, LiftComponent.LIFT_SPEED);
                this.state = State.AWAITING_EXTERNAL_LIFT;

                this.timeoutTimer.reset();

                break;

            case AWAITING_EXTERNAL_LIFT:
                if(!this.liftComponent.isBusy() || this.timeoutTimer.seconds() > 2) {
                    if(this.timeoutTimer.seconds() > 2) {
                        System.err.println("TIMEOUT HIT");
                    }

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
                this.state = State.EXTERNAL_LIFTING;

                break;

            case EXTERNAL_LIFTING:
                liftComponent.goToEncoderPosition(LiftComponent.LIFT_POSITION_ARM_SAFE, LiftComponent.LIFT_SPEED);

                this.state = State.AWAITING_EXTERNAL_LIFT;

                break;

            case AWAITING_EXTERNAL_LIFT:
                if(!liftComponent.isBusy()) {
                    this.state = State.SWINGING_ARM;
                }

                break;

            case SWINGING_ARM:
                this.swingArmComponent.goToIntakePosition();
                this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_INTAKE_POSITION);

                this.timeoutTimer.reset();

                this.state = State.AWAITING_ARM_SWING;

                break;

            case AWAITING_ARM_SWING:

                if ((!this.swingArmComponent.isMotorBusy() &&
                        this.pixelPouchComponent.getPouchServoPosition() == PixelPouchComponent.POUCH_SCORING_POSITION)
                        || timeoutTimer.seconds() > 2) {

                    if (timeoutTimer.seconds() > 2)
                        System.err.println("TIMEOUT HIT");

                    this.state = State.INTERNAL_LIFTING;

                }

                break;

            case INTERNAL_LIFTING:

                liftComponent.goToEncoderPosition(LiftComponent.LIFT_POSITION_INTAKE, LiftComponent.LIFT_SPEED);
                this.state = State.AWAITING_INTERNAL_LIFT;

                this.timeoutTimer.reset();

                break;

            case AWAITING_INTERNAL_LIFT:

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
        this.swingArmComponent.printTelemetry(telemetry);
        this.pixelPouchComponent.printTelemetry(telemetry);

        telemetry.addData("Is Swing Arm Motor Busy", swingArmComponent.isMotorBusy());
        telemetry.addData("Target Position", targetPosition.name());
        telemetry.addData("State", state.name());
    }
}
