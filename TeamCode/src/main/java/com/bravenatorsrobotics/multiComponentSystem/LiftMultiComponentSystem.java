package com.bravenatorsrobotics.multiComponentSystem;

import com.bravenatorsrobotics.components.LiftComponent;
import com.bravenatorsrobotics.components.PixelPouchComponent;
import com.bravenatorsrobotics.components.SwingArmComponent;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftMultiComponentSystem {

    public static final int RETURN_CRITICAL_POINT = 150;
    private static final int LIFT_ENCODER_TICK_TOLERANCE = 5;

    private final LiftComponent liftComponent;
    private final SwingArmComponent swingArmComponent;
    private final PixelPouchComponent pixelPouchComponent;

    private enum TargetPosition {
        INTAKE_POSITION,
        SCORING_POSITION
    }

    public enum State {

        AT_INTAKE_POSITION,
        AT_SCORING_POSITION

    }

    private TargetPosition targetPosition = TargetPosition.INTAKE_POSITION;
    private State state = State.AT_INTAKE_POSITION;

    private int targetExternalLiftPosition = LiftComponent.LIFT_POSITION_ARM_SAFE;
    private int liveExternalLiftPosition = targetExternalLiftPosition;

    public LiftMultiComponentSystem(LiftComponent liftComponent, SwingArmComponent swingArmComponent, PixelPouchComponent pixelPouchComponent) {
        this.liftComponent = liftComponent;
        this.swingArmComponent = swingArmComponent;
        this.pixelPouchComponent = pixelPouchComponent;
    }

    public void goToScoringPosition(int targetExternalLiftPosition) {

        this.targetPosition = TargetPosition.SCORING_POSITION;
        this.targetExternalLiftPosition = targetExternalLiftPosition;
        this.liveExternalLiftPosition = targetExternalLiftPosition;

    }

    public void goToIntakePosition() {

        this.targetPosition = TargetPosition.INTAKE_POSITION;

    }

    public void update() {

        switch (this.targetPosition) {
            case SCORING_POSITION -> this.scoringPositionFlow();
            case INTAKE_POSITION -> this.intakePositionFlow();
        }

    }

    private void scoringPositionFlow() {

        switch (this.state) {

            case AT_INTAKE_POSITION -> {

                if(this.swingArmComponent.getTargetPosition() != SwingArmComponent.SWING_ARM_SCORING_POSITION) {

                    this.swingArmComponent.goToScoringPosition();
                    this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_FLIP_POSITION);
                    this.liftComponent.goToEncoderPositionAsync(targetExternalLiftPosition, LiftComponent.LIFT_SPEED);

                } else {
                    if(!this.swingArmComponent.isMotorBusy()) {

                        this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_SCORING_POSITION);
                        this.state = State.AT_SCORING_POSITION;

                    } else if(this.swingArmComponent.getSwingArmMotorPosition() >= SwingArmComponent.SWING_ARM_FLIP_POSITION) {
                        this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_SCORING_POSITION);
                    }
                }

            }

            case AT_SCORING_POSITION -> {

                if(this.swingArmComponent.getTargetPosition() != SwingArmComponent.SWING_ARM_SCORING_POSITION) {
                    this.state = State.AT_INTAKE_POSITION;
                }

                if(this.liftComponent.getMotorTargetPosition() != targetExternalLiftPosition)
                    this.liftComponent.goToEncoderPositionAsync(targetExternalLiftPosition, LiftComponent.LIFT_SPEED);
            }

        }

    }

    private void intakePositionFlow() {

        switch (this.state) {

            case AT_SCORING_POSITION -> {

                if(this.swingArmComponent.getTargetPosition() != SwingArmComponent.SWING_ARM_INTAKE_POSITION) {
                    this.swingArmComponent.goToIntakePosition();
                    this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_FLIP_POSITION);
                    this.liftComponent.goToEncoderPositionAsync(LiftComponent.LIFT_POSITION_INTAKE, LiftComponent.LIFT_SPEED);
                } else {

                    if(!this.swingArmComponent.isMotorBusy()) {
                        this.swingArmComponent.goToIntakePosition();
                        this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_INTAKE_POSITION);
                        this.state = State.AT_INTAKE_POSITION;
                    } else if(this.swingArmComponent.getSwingArmMotorPosition() <= SwingArmComponent.SWING_ARM_FLIP_POSITION - RETURN_CRITICAL_POINT) {
                        this.pixelPouchComponent.setPouchPosition(PixelPouchComponent.POUCH_INTAKE_POSITION);
                    }

                }

            }

            case AT_INTAKE_POSITION -> {

                if(this.swingArmComponent.getSwingArmMotorPosition() != SwingArmComponent.SWING_ARM_INTAKE_POSITION)
                    this.state = State.AT_SCORING_POSITION;

            }

        }

    }

    public void liveAdjustLiftHeight(int encoderTicks) { this.liveExternalLiftPosition += encoderTicks; }

    public boolean isBusy() {
        return switch (this.targetPosition) {
            case SCORING_POSITION -> this.state != State.AT_SCORING_POSITION;
            case INTAKE_POSITION  -> this.state != State.AT_INTAKE_POSITION;
        };
    }

    public void telemetry(Telemetry telemetry) {
        this.swingArmComponent.printTelemetry(telemetry);
        this.pixelPouchComponent.printTelemetry(telemetry);

        telemetry.addData("Is Swing Arm Motor Busy", swingArmComponent.isMotorBusy());
        telemetry.addData("Is Lift Motor Busy", liftComponent.isMotorBusy());
        telemetry.addData("Lift Multi-Component System State", this.state.name());
        telemetry.addData("Lift Live Adjust Position", this.liveExternalLiftPosition);
        telemetry.addData("Lift Position", this.liftComponent.getCurrentPosition());
        telemetry.addData("Is Lift Multi Component System Busy", this.isBusy());
    }

    private boolean isMotorAtPositionWithTolerance(int currentPos, int targetPosition) {

        int min = targetPosition - LIFT_ENCODER_TICK_TOLERANCE;
        int max = targetPosition + LIFT_ENCODER_TICK_TOLERANCE;

        return currentPos >= min && currentPos <=  max;

    }

    public State getCurrentState() { return this.state; }
}
