package org.firstinspires.ftc.teamcode.mechanisms_and_movement.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtendoM {
    private final DcMotorEx extendom;


    public ExtendoM(HardwareMap hardwareMap) {
        extendom = hardwareMap.get(DcMotorEx.class, "extendom");
        extendom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendom.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    //-----------------------------In---------------------------------------\\
    public class ExtendoM_In implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                extendom.setPower(-0.8);
                initialized = true;
            }

            double pos = extendom.getCurrentPosition();
            packet.put("extendomPos", pos);
            if (pos > 0) {
                return true;
            } else {
                //negative gravity
                extendom.setPower(0.01);
                return false;
            }
        }
    }
    public Action in() {
        return new ExtendoM_In();
    }

    //-----------------------------Mini Out---------------------------------------\\
    public class ExtendoM_MiniOut implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                extendom.setPower(0.8);
                initialized = true;
            }

            double pos = extendom.getCurrentPosition();
            packet.put("extendomPos", pos);
            if (pos < 195 || pos > 205) {
                return true;
            } else {
                //negative gravity
                extendom.setPower(0.01);
                return false;
            }
        }
    }
    public Action mini_out() {
        return new ExtendoM_MiniOut();
    }

    //-----------------------------Out---------------------------------------\\

    public class ExtendoM_Out implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                extendom.setPower(0.8);
                initialized = true;
            }

            double pos = extendom.getCurrentPosition();
            packet.put("extendomPos", pos);
            if (pos < 995 || pos > 1005) {
                return true;
            } else {
                //negative gravity
                extendom.setPower(0.01);
                return false;
            }
        }
    }
    public Action out() {
        return new ExtendoM_Out();
    }
}

