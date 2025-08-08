package org.firstinspires.ftc.teamcode.mechanisms_and_movement.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extendo {
    private final Servo extendoL;
    private final Servo extendoR;


    public Extendo(HardwareMap hardwareMap) {
        extendoL = hardwareMap.get(Servo.class, "extendoL");
        extendoR = hardwareMap.get(Servo.class, "extendoR");

    }
    //-----------------------------In---------------------------------------\\
    public class ExtendoIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            extendoL.setPosition(0.1);
            extendoR.setPosition(0.1);
            return false;
        }
    }
    public Action in() {
        return new ExtendoIn();
    }

    //-----------------------------Mini Out---------------------------------------\\
    public class Extendo_MiniOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            extendoL.setPosition(0.2);
            extendoR.setPosition(0.2);
            return false;
        }
    }
    public Action mini_out() {
        return new Extendo_MiniOut();
    }

    //-----------------------------Out---------------------------------------\\

    public class ExtendoOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            extendoL.setPosition(0.45);
            extendoR.setPosition(0.45);

            return false;
        }
    }
    public Action out() {
        return new ExtendoOut();
    }
}

