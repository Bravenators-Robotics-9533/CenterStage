package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.bravenatorsrobotics.eventSystem.Callback;
import com.bravenatorsrobotics.eventSystem.CallbackSystem;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PixelPouchComponent {

    public static final double CLAMP_OPEN_POSITION = 1.0;
    public static final double CLAMP_CLOSE_POSITION = 0.7;

    public static final double POUCH_INTAKE_POSITION = 0;
    public static final double POUCH_SCORING_POSITION = 0.7;

    private static final double POUCH_SENSOR_DISTANCE = 26;

    private final CallbackSystem onClampCallbackSystem = new CallbackSystem();

    private enum PixelPouchStatus {

        OPEN,
        CLOSED,

        OPEN_REQUESTED,
        CLOSE_REQUESTED,

        OPEN_AWAITING_PIXEL_REMOVAL

    }

    private PixelPouchStatus pixelPouchStatus = PixelPouchStatus.OPEN;

    private final Servo clampServo;
    private final Servo pouchServo;
    private final RevColorSensorV3 pouchColorSensor;

    private final boolean shouldDetect;

    public PixelPouchComponent(HardwareMap hardwareMap, boolean shouldDetect) {

        this.shouldDetect = shouldDetect;

        this.clampServo         = hardwareMap.get(Servo.class, HardwareMapIdentities.SERVO_CLAMP);
        this.pouchServo         = hardwareMap.get(Servo.class, HardwareMapIdentities.POUCH_SERVO);
        this.pouchServo.setDirection(Servo.Direction.REVERSE);

        this.pouchColorSensor   = hardwareMap.get(RevColorSensorV3.class, HardwareMapIdentities.POUCH_SENSOR);

    }

    public void initializeServo() {

        this.clampServo.setPosition(CLAMP_OPEN_POSITION);
        this.pouchServo.setPosition(POUCH_INTAKE_POSITION);

    }

    public void update() {

        boolean isPixelDetected = isPixelDetected();

        switch (pixelPouchStatus) {

            case OPEN_REQUESTED:
                pixelPouchStatus = isPixelClampOpen() ? PixelPouchStatus.OPEN : PixelPouchStatus.OPEN_AWAITING_PIXEL_REMOVAL;
                break;

            case OPEN_AWAITING_PIXEL_REMOVAL:
                if(!isPixelClampOpen()) {
                    clampServo.setPosition(CLAMP_OPEN_POSITION);
                } else if(!isPixelDetected) {
                    pixelPouchStatus = PixelPouchStatus.OPEN;
                }

                break;

            case OPEN:
                if(isPixelDetected && shouldDetect) {
                    pixelPouchStatus = PixelPouchStatus.CLOSE_REQUESTED;
                    clampServo.setPosition(CLAMP_CLOSE_POSITION);
                }

                break;

            case CLOSE_REQUESTED:
                if(clampServo.getPosition() != CLAMP_CLOSE_POSITION)
                    clampServo.setPosition(CLAMP_CLOSE_POSITION);
                else {
                    pixelPouchStatus = PixelPouchStatus.CLOSED;
                    onClampCallbackSystem.fireCallback();
                }

                break;

            case CLOSED:
                if(!isPixelDetected && shouldDetect)
                    this.pixelPouchStatus = PixelPouchStatus.OPEN_REQUESTED;

                break;

        }

    }

    public void togglePouchPosition() {

        if(pouchServo.getPosition() != POUCH_INTAKE_POSITION)
            pouchServo.setPosition(POUCH_INTAKE_POSITION);
        else
            pouchServo.setPosition(POUCH_SCORING_POSITION);

    }

    public void setClampPosition(double position) { this.clampServo.setPosition(position); }

    public void setPouchPosition(double position) {
        pouchServo.setPosition(position);
    }

    public double getPouchServoPosition() { return this.pouchServo.getPosition(); }

    public boolean isServoAtScoringPosition() { return this.pouchServo.getPosition() == POUCH_SCORING_POSITION; }
    public boolean isServoAtIntakePosition() { return this.pouchServo.getPosition() == POUCH_INTAKE_POSITION; }

    public void requestRelease() {
        pixelPouchStatus = PixelPouchStatus.OPEN_REQUESTED;
    }

    public void requestClose() { this.pixelPouchStatus = PixelPouchStatus.CLOSE_REQUESTED; }

    public boolean isPixelDetected() {
        return this.pouchColorSensor.getDistance(DistanceUnit.MM) <= POUCH_SENSOR_DISTANCE;
    }

    public boolean isPixelClampOpen() { return this.clampServo.getPosition() == CLAMP_OPEN_POSITION; }

    public void addOnClampCallback(Callback callback) { this.onClampCallbackSystem.addCallback(callback); }
    public void removeOnClampCallback(Callback callback) { this.onClampCallbackSystem.removeCallback(callback); }

    public PixelPouchStatus getPixelPouchStatus() { return this.pixelPouchStatus; }

    public NormalizedRGBA getPouchSensorActiveColor() { return this.pouchColorSensor.getNormalizedColors(); }

    public void printTelemetry(Telemetry telemetry) {

        telemetry.addData("Pixel Pouch Servo Position", this.pouchServo.getPosition());

    }

}
