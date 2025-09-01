package org.firstinspires.ftc.teamcode.mechanisms_and_movement.itd.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakePivot {
    private final Servo armClaw;

    public IntakePivot(HardwareMap hardwareMap) {
        armClaw = hardwareMap.get(Servo.class, "armClaw");
    }

    //-----------------------------Transfer--------------------------------------\\
    public class IPTransfer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armClaw.setPosition(0.5);
            return false;
        }
    }
    public Action toTransfer() {
        return new IPTransfer();
    }

    //-----------------------------Intake-----------------------------------\\
    public class IPReady implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armClaw.setPosition(0);
            return false;
        }
    }
    public Action toIntake() {
        return new IPReady();
    }


}

