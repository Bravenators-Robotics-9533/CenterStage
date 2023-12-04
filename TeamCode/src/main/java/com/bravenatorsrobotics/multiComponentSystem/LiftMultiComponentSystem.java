package com.bravenatorsrobotics.multiComponentSystem;

import com.bravenatorsrobotics.components.LiftComponent;
import com.bravenatorsrobotics.components.PixelPouchComponent;
import com.bravenatorsrobotics.components.SwingArmComponent;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftMultiComponentSystem {

    private final LiftComponent liftComponent;
    private final SwingArmComponent swingArmComponent;
    private final PixelPouchComponent pixelPouchComponent;

    private enum TargetPosition {
        INTAKE_POSITION,
        SCORING_POSITION
    }

    private enum State {

        AT_INTAKE_POSITION,
        INTERNAL_LIFT,
        SWING_ARM,
        EXTERNAL_LIFT,
        AT_SCORING_POSITION

    }

    private TargetPosition targetPosition = TargetPosition.INTAKE_POSITION;
    private State state = State.AT_INTAKE_POSITION;

    private int targetExternalLiftPosition = LiftComponent.LIFT_POSITION_ARM_SAFE;

    public LiftMultiComponentSystem(LiftComponent liftComponent, SwingArmComponent swingArmComponent, PixelPouchComponent pixelPouchComponent) {
        this.liftComponent = liftComponent;
        this.swingArmComponent = swingArmComponent;
        this.pixelPouchComponent = pixelPouchComponent;
    }

    public void goToScoringPosition(int targetExternalLiftPosition) {

        this.targetPosition = TargetPosition.SCORING_POSITION;
        this.targetExternalLiftPosition = targetExternalLiftPosition;

    }

    public void goToIntakePosition() {

        this.targetPosition = TargetPosition.INTAKE_POSITION;

    }

    public void update() {

        switch (this.targetPosition) {

            case SCORING_POSITION:
                this.scoringPositionFlow();
                break;

            case INTAKE_POSITION:
                this.intakePositionFlow();
                break;

        }

    }

    private void scoringPositionFlow() {

        switch (this.state) {

            case AT_INTAKE_POSITION:
                this.state = State.INTERNAL_LIFT;

                break;

            case INTERNAL_LIFT:

                if(!this.liftComponent.isBusy()) {

                    if(this.liftComponent.getCurrentPosition() != LiftComponent.LIFT_POSITION_ARM_SAFE) { // Start the lift
                        this.liftComponent.goToEncoderPositionAsync(LiftComponent.LIFT_POSITION_ARM_SAFE, LiftComponent.LIFT_SPEED);
                    } else { // Lift is in position
                        this.state = State.SWING_ARM;
                    }

                }

                break;

            case SWING_ARM:

                if(!this.swingArmComponent.isMotorBusy()) {

                    if (this.swingArmComponent.getSwingArmMotorPosition() != SwingArmComponent.SWING_ARM_SCORING_POSITION) { // Start the swing arm
                        this.swingArmComponent.goToScoringPosition();
                        this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_SCORING_POSITION);
                    } else { // At Scoring Position
                        this.state = State.EXTERNAL_LIFT;
                    }

                }

                break;


            case EXTERNAL_LIFT:

                if(!this.liftComponent.isBusy()) {

                    if(this.liftComponent.getCurrentPosition() != targetExternalLiftPosition) { // Start the lift
                        this.liftComponent.goToEncoderPositionAsync(targetExternalLiftPosition, LiftComponent.LIFT_SPEED);
                    } else { // Lift is in position
                        this.state = State.AT_SCORING_POSITION;
                    }

                }

                break;

            case AT_SCORING_POSITION:
                break; // DONE

        }

    }

    private void intakePositionFlow() {

        switch (this.state) {

            case AT_SCORING_POSITION:
                this.state = State.EXTERNAL_LIFT;

                break;

            case EXTERNAL_LIFT:

                if(!this.liftComponent.isBusy()) {

                    if(this.liftComponent.getCurrentPosition() != LiftComponent.LIFT_POSITION_ARM_SAFE) { // Start the lift
                        this.liftComponent.goToEncoderPositionAsync(LiftComponent.LIFT_POSITION_ARM_SAFE, LiftComponent.LIFT_SPEED);
                    } else { // Lift is in position
                        this.state = State.SWING_ARM;
                    }

                }

                break;

            case SWING_ARM:

                if(!this.swingArmComponent.isMotorBusy()) {

                    if(this.swingArmComponent.getSwingArmMotorPosition() != SwingArmComponent.SWING_ARM_INTAKE_POSITION) { // Start the swing arm
                        this.swingArmComponent.goToIntakePosition();
                        this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_INTAKE_POSITION);
                    } else { // At Intake Position
                        this.state = State.INTERNAL_LIFT;
                    }

                }

                break;

            case INTERNAL_LIFT:

                if(!this.liftComponent.isBusy()) {

                    if(this.liftComponent.getCurrentPosition() != LiftComponent.LIFT_POSITION_INTAKE) { // Start the lift
                        this.liftComponent.goToEncoderPositionAsync(LiftComponent.LIFT_POSITION_INTAKE, LiftComponent.LIFT_SPEED);
                    } else { // Lift is in position
                        this.state = State.AT_INTAKE_POSITION;
                    }

                }

                break;

            case AT_INTAKE_POSITION:
                break; // DONE

        }

    }

    public boolean isBusy() {
        return this.swingArmComponent.isMotorBusy();
    }

    public void telemetry(Telemetry telemetry) {
        this.swingArmComponent.printTelemetry(telemetry);
        this.pixelPouchComponent.printTelemetry(telemetry);

        telemetry.addData("Is Swing Arm Motor Busy", swingArmComponent.isMotorBusy());
        telemetry.addData("Lift Multi-Component System State", this.state.name());
    }
}
