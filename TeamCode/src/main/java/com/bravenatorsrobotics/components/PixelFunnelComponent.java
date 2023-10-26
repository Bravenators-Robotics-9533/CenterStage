package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelFunnelComponent {

    private final Servo pixelFunnelServo;

    private static final double CAPTURE_SERVO_POSITION = 0.0;
    private static final double RELEASE_SERVO_POSITION = 0.35;

    public PixelFunnelComponent(HardwareMap hardwareMap) {

        this.pixelFunnelServo = hardwareMap.get(Servo.class, HardwareMapIdentities.FUNNEL_SERVO);
        this.pixelFunnelServo.setDirection(Servo.Direction.REVERSE);

    }

    public void releasePixel() {
        this.pixelFunnelServo.setPosition(RELEASE_SERVO_POSITION);
    }

    public void capturePixel() {
        this.pixelFunnelServo.setPosition(CAPTURE_SERVO_POSITION);
    }

}
