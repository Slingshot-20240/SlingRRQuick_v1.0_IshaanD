package org.firstinspires.ftc.teamcode.mechanisms_and_movement.itd.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawPivot {
    private final Servo clawPivot;

    public ClawPivot(HardwareMap hardwareMap) {
        clawPivot = hardwareMap.get(Servo.class, "clawPivot");
    }

    //-----------------------------Transfer--------------------------------------\\
    public class CPTransfer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawPivot.setPosition(0);
            return false;
        }
    }
    public Action toTransfer() {
        return new CPTransfer();
    }

    //-----------------------------Score-----------------------------------\\
    public class CPScore implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawPivot.setPosition(0.3);
            return false;
        }
    }
    public Action toScore() {
        return new CPScore();
    }


}

