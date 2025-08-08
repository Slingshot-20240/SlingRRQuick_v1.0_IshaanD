package org.firstinspires.ftc.teamcode.mechanisms_and_movement.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ActiveIntake {
    private final DcMotorEx activeIntake;

    public ActiveIntake(HardwareMap hardwareMap) {
        activeIntake = hardwareMap.get(DcMotorEx.class, "activeIntake");
        activeIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        activeIntake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //----------------------------In----------------------------------\\
    public class IntakeIn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            activeIntake.setPower(0.7);
            return false;
        }
    }
    public Action in() {
        return new IntakeIn();
    }

    //-----------------------------Idle--------------------------------------\\
    public class IntakeIdle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            activeIntake.setPower(0);
            return false;
        }
    }
    public Action idle() {
        return new IntakeIdle();
    }

    //-----------------------------Out--------------------------------------\\
    public class IntakeOut implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            activeIntake.setPower(-0.7);
            return false;
        }
    }
    public Action out() {
        return new IntakeOut();
    }

}

