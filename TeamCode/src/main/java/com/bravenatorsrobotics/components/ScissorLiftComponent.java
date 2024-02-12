package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.bravenatorsrobotics.utils.ServoUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class ScissorLiftComponent {

    // TODO: EXPERIMENT
    public static double MAX_SERVO_VELOCITY = 0.84; // revs per second or .78 for older servos
    public static double INCHES_PER_REV = 10;

    private final Servo scissorLiftServo;

    public ScissorLiftComponent(HardwareMap hardwareMap) {

        this.scissorLiftServo = hardwareMap.get(Servo.class, HardwareMapIdentities.SCISSOR_LIFT_SERVO);

    }

    public void update() {



    }

    public void setVelocity(double velocity) {

        velocity = Range.clip(velocity, -MAX_SERVO_VELOCITY, MAX_SERVO_VELOCITY);

        double scaledValue = ServoUtils.convertStandardToContinualRange(velocity / MAX_SERVO_VELOCITY);
        scissorLiftServo.setPosition(scaledValue);

    }

}
