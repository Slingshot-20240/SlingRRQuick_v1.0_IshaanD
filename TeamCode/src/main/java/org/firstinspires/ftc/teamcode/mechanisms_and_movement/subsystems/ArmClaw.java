package org.firstinspires.ftc.teamcode.mechanisms_and_movement.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmClaw {
    private final Servo armClaw;

    public ArmClaw(HardwareMap hardwareMap) {
        armClaw = hardwareMap.get(Servo.class, "armClaw");
    }

    //-----------------------------Open--------------------------------------\\
    public class OpenArmClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armClaw.setPosition(1.0);
            return false;
        }
    }
    public Action open() {
        return new OpenArmClaw();
    }

    //-----------------------------Close-----------------------------------\\
    public class CloseArmClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armClaw.setPosition(0.55);
            return false;
        }
    }
    public Action close() {
        return new CloseArmClaw();
    }


}

