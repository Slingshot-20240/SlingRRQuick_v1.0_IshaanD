package org.firstinspires.ftc.teamcode.mechanisms_and_movement.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private final DcMotorEx lift;

    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    //-----------------------------Lift Down--------------------------------------\\
    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setPower(-0.8);
                initialized = true;
            }

            double pos = lift.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos > 100.0) {
                return true;
            } else {
                //negative gravity
                lift.setPower(0.01);
                return false;
            }
        }
    }
    public Action toDown(){
        return new LiftDown();
    }

    //----------------------------Lift Transfer----------------------------------\\
    public class LiftTransfer implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setPower(0.8);
                initialized = true;
            }

            double pos = lift.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 395 || pos > 405) {
                return true;
            } else {
                //negative gravity
                lift.setPower(0.01);
                return false;
            }
        }
    }
    public Action toTransfer() {
        return new LiftTransfer();
    }

    //---------------------------Lift Low Basket-----------------------------------\\
    public class LiftLowBasket implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setPower(0.8);
                initialized = true;
            }

            double pos = lift.getCurrentPosition();
            packet.put("liftPos", pos);
            //Gives a safety range
            if (pos < 1395 || pos > 1405) {
                return true;
            } else {
                //negative gravity
                lift.setPower(0.01);
                return false;
            }
        }
    }
    public Action toLowBasket() {
        return new LiftLowBasket();
    }

    //--------------------------Lift High Basket---------------------------------\\
    public class LiftHighBasket implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setPower(0.8);
                initialized = true;
            }

            double pos = lift.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 2000) {
                return true;
            } else {
                //negative gravity
                lift.setPower(0.01);
                return false;
            }
        }
    }
    public Action toHighBasket() {
        return new LiftHighBasket();
    }



}

