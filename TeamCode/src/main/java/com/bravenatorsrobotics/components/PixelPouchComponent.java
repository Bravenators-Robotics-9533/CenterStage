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

    private static final double CLAMP_OPEN_POSITION = 0.85;
    private static final double CLAMP_CLOSE_POSITION = 0.75;

    private static final double POUCH_SENSOR_DISTANCE = 22.0;
    private static final double RELEASE_WAIT_SECONDS = 0.75;

    private final CallbackSystem onClampCallbackSystem = new CallbackSystem();

    private final Servo clampServo;
    private final RevColorSensorV3 pouchColorSensor;

    private boolean isOpen = true;
    private boolean isReleaseRequested = false;

    private ElapsedTime releaseTimer = null;

    public PixelPouchComponent(HardwareMap hardwareMap) {

        this.clampServo         = hardwareMap.get(Servo.class, HardwareMapIdentities.SERVO_CLAMP);
        this.pouchColorSensor   = hardwareMap.get(RevColorSensorV3.class, HardwareMapIdentities.POUCH_SENSOR);

    }

    public void initializeServo() {

        this.clampServo.setPosition(CLAMP_OPEN_POSITION);

    }

    public void update() {

        // Check to see if release is requested
        if(this.isReleaseRequested) {

            // If the clamp is already open then cancel the release request
            if(this.isOpen) {
                this.isReleaseRequested = false;
            } else {
                // Start the release timer and open the clamp
                if (this.releaseTimer == null) {
                    this.clampServo.setPosition(CLAMP_OPEN_POSITION);
                    this.isOpen = true;

                    this.releaseTimer = new ElapsedTime();
                }

                // If the release timer is up cancel the release request and nullify the timer for next use
                if (this.releaseTimer.seconds() >= RELEASE_WAIT_SECONDS) {
                    this.isReleaseRequested = false;
                    this.releaseTimer = null;
                }
            }

        }

        if(isOpen && !isReleaseRequested && this.isPixelDetected()) {

            this.clampServo.setPosition(CLAMP_CLOSE_POSITION);
            this.isOpen = false;

            this.onClampCallbackSystem.fireCallback();

        }

    }

    public void requestRelease() { this.isReleaseRequested = true; }

    public boolean isPixelDetected() {
        return this.pouchColorSensor.getDistance(DistanceUnit.MM) <= POUCH_SENSOR_DISTANCE;
    }

    public void addOnClampCallback(Callback callback) { this.onClampCallbackSystem.addCallback(callback); }
    public void removeOnClampCallback(Callback callback) { this.onClampCallbackSystem.removeCallback(callback); }

}
