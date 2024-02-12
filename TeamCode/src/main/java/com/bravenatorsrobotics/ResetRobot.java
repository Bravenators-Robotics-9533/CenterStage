package com.bravenatorsrobotics;

import com.bravenatorsrobotics.components.SuspensionLiftComponent;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Reset Robot", group = "Utils")
public class ResetRobot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SuspensionLiftComponent suspensionLiftComponent = new SuspensionLiftComponent(super.hardwareMap);

        waitForStart();

        waitForAPressed("Move Suspension Hooks to Locked Position");

        // Move Suspension Hooks

        while(opModeIsActive()) {
            suspensionLiftComponent.update();
        }

//        waitForAPressed("Zero Suspension Lift");

    }

    private void waitForAPressed(String message) {

        boolean isBumperPressed = false;

        telemetry.addLine(message);
        telemetry.addLine("Press Right Bumper to Continue...");
        telemetry.update();

        while(opModeIsActive() && !isBumperPressed) {

            if(gamepad1.right_bumper)
                isBumperPressed = true;

        }

        telemetry.clearAll();
        telemetry.update();

    }

    private final ElapsedTime timer = new ElapsedTime();


    private void waitMillis(int millis) {

        this.timer.reset();

        while(timer.milliseconds() < millis) {
            if(!opModeIsActive())
                break;
        }
    }

}
