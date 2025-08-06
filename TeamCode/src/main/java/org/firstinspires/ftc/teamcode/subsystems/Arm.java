package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private final Servo arm;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(Servo.class, "arm");
    }

    //-----------------------------Transfer-----------------------------------\\
    public class ArmTransfer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition(0);
            return false;
        }
    }
    public Action toTransfer() {
        return new ArmTransfer();
    }

    //-----------------------------Score--------------------------------------\\
    public class ArmScore implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setPosition(0.6);
            return false;
        }
    }
    public Action toScore() {
        return new ArmScore();
    }
}

