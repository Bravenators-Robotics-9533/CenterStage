package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.bravenatorsrobotics.eventSystem.Callback;
import com.bravenatorsrobotics.eventSystem.CallbackSystem;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PixelPouchComponent {

    private static final double CLAMP_OPEN_POSITION = 1.0;
    private static final double CLAMP_CLOSE_POSITION = 0.75;

    private static final double POUCH_INTAKE_POSITION = 0;
    private static final double POUCH_RELEASE_POSITION = 0.5;

    private static final double POUCH_SENSOR_DISTANCE = 22.0;

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

    public PixelPouchComponent(HardwareMap hardwareMap) {

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
                if(isPixelDetected) {
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

        }

    }

    public void togglePouchPosition() {

        if(pouchServo.getPosition() != POUCH_INTAKE_POSITION)
            pouchServo.setPosition(POUCH_INTAKE_POSITION);
        else
            pouchServo.setPosition(POUCH_RELEASE_POSITION);

    }

    public void requestRelease() {
        pixelPouchStatus = PixelPouchStatus.OPEN_REQUESTED;
    }

    public boolean isPixelDetected() {
        return this.pouchColorSensor.getDistance(DistanceUnit.MM) <= POUCH_SENSOR_DISTANCE;
    }

    public boolean isPixelClampOpen() { return this.clampServo.getPosition() == CLAMP_OPEN_POSITION; }

    public void addOnClampCallback(Callback callback) { this.onClampCallbackSystem.addCallback(callback); }
    public void removeOnClampCallback(Callback callback) { this.onClampCallbackSystem.removeCallback(callback); }

    public PixelPouchStatus getPixelPouchStatus() { return this.pixelPouchStatus; }

}
