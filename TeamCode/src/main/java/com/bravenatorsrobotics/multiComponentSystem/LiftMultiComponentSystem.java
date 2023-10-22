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

    private enum SystemPosition {
        SCORING_POSITION,
        INTAKE_POSITION
    }

    private enum ScoringPositionState {

        LIFTING,
        AWAITING_LIFT,
        SWINGING_ARM,
        AWAITING_ARM_SWING,
        DONE

    }

    private enum IntakePositionState {

        SWINGING_ARM,
        AWAITING_ARM_SWING,
        LIFTING,
        AWAITING_LIFT,
        DONE

    }

    private SystemPosition actualPosition = SystemPosition.INTAKE_POSITION;
    private SystemPosition targetPosition = SystemPosition.INTAKE_POSITION;

    private ScoringPositionState scoringPositionState = ScoringPositionState.LIFTING;
    private IntakePositionState intakePositionState = IntakePositionState.SWINGING_ARM;

    public LiftMultiComponentSystem(LiftComponent liftComponent, SwingArmComponent swingArmComponent, PixelPouchComponent pixelPouchComponent) {
        this.liftComponent = liftComponent;
        this.swingArmComponent = swingArmComponent;
        this.pixelPouchComponent = pixelPouchComponent;
    }

    public void goToScoringPosition() {

        this.scoringPositionState = ScoringPositionState.LIFTING;
        this.targetPosition = SystemPosition.SCORING_POSITION;

    }

    public void goToIntakePosition() {

        this.intakePositionState = IntakePositionState.SWINGING_ARM;
        this.targetPosition = SystemPosition.INTAKE_POSITION;

    }

    public void update() {

        if (this.actualPosition != this.targetPosition) {

            switch (this.targetPosition) {

                case INTAKE_POSITION:
                    updateIntakeStateMachine();
                    break;
                case SCORING_POSITION:
                    updateScoringStateMachine();
                    break;

            }

        }

    }

    private final ElapsedTime timeoutTimer = new ElapsedTime();

    private static final int SAFE_TARGET_TOLERANCE = 10;

    private void updateScoringStateMachine() {

        switch (this.scoringPositionState) {

            case LIFTING:
                liftComponent.goToEncoderPosition(LiftComponent.LIFT_POSITION_STAGE_LOWER_RELEASE, LiftComponent.LIFT_SPEED);
                this.scoringPositionState = ScoringPositionState.AWAITING_LIFT;

                this.timeoutTimer.reset();

                break;

            case AWAITING_LIFT:
                if (this.liftComponent.getCurrentPosition() >= LiftComponent.LIFT_POSITION_ARM_SAFE - SAFE_TARGET_TOLERANCE || this.timeoutTimer.seconds() > 2) {

                    if (timeoutTimer.seconds() > 2)
                        System.err.println("TIMEOUT HIT");

                    this.scoringPositionState = ScoringPositionState.SWINGING_ARM;
                }

                break;

            case SWINGING_ARM:
                this.swingArmComponent.swingOut();
                this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_RELEASE_POSITION);

                this.timeoutTimer.reset();

                this.scoringPositionState = ScoringPositionState.AWAITING_ARM_SWING;

                break;

            case AWAITING_ARM_SWING:
                if ((!this.swingArmComponent.isBusy() &&
                        this.pixelPouchComponent.getPouchPosition() == PixelPouchComponent.POUCH_RELEASE_POSITION)
                        || timeoutTimer.seconds() > 2) {

                    if (timeoutTimer.seconds() > 2)
                        System.err.println("TIMEOUT HIT");

                    this.scoringPositionState = ScoringPositionState.DONE;

                }

                break;

            case DONE:
                this.actualPosition = SystemPosition.SCORING_POSITION;
                break;

        }

    }

    private void updateIntakeStateMachine() {

        switch (this.intakePositionState) {

            case SWINGING_ARM:

                this.swingArmComponent.goToIntakePosition();
                this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_INTAKE_POSITION);

                this.timeoutTimer.reset();

                this.intakePositionState = IntakePositionState.AWAITING_ARM_SWING;

                break;

            case AWAITING_ARM_SWING:

                if ((!this.swingArmComponent.isBusy() &&
                        this.pixelPouchComponent.getPouchPosition() == PixelPouchComponent.POUCH_RELEASE_POSITION)
                        || timeoutTimer.seconds() > 2) {

                    if (timeoutTimer.seconds() > 2)
                        System.err.println("TIMEOUT HIT");

                    this.intakePositionState = IntakePositionState.LIFTING;

                }

                break;

            case LIFTING:

                liftComponent.goToEncoderPosition(LiftComponent.LIFT_POSITION_INTAKE, LiftComponent.LIFT_SPEED);
                this.intakePositionState = IntakePositionState.AWAITING_LIFT;

                this.timeoutTimer.reset();

                break;

            case AWAITING_LIFT:

                if(this.liftComponent.isBusy() || this.timeoutTimer.seconds() > 2) {

                    if(this.timeoutTimer.seconds() > 2) {
                        System.err.println("TIMEOUT HIT");
                    }

                    this.intakePositionState = IntakePositionState.DONE;

                }

                break;

            case DONE:
                this.actualPosition = SystemPosition.INTAKE_POSITION;
                break;
        }

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Target Position",this.targetPosition.name());
        telemetry.addData("Actual Position", this.actualPosition.name());
        telemetry.addData("Scoring State", this.scoringPositionState.name());
        telemetry.addData("Intake State", this.intakePositionState.name());
    }
}
