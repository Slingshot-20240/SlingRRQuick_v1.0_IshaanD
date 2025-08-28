package org.firstinspires.ftc.teamcode.mechanisms_and_movement.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private final DcMotorEx liftl;
    private final DcMotorEx liftr;

    public Lift(HardwareMap hardwareMap) {
        liftl = hardwareMap.get(DcMotorEx.class, "liftl");
        liftr = hardwareMap.get(DcMotorEx.class, "liftr");

        liftl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftl.setDirection(DcMotorSimple.Direction.FORWARD);

        liftr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftr.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //-----------------------------Lift Down--------------------------------------\\
    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftl.setPower(-0.8);
                liftr.setPower(-0.8);
                initialized = true;
            }

            double posl = liftl.getCurrentPosition();
            double posr = liftr.getCurrentPosition();
            double avgPos = (Math.abs(posl - posr) <= 50) ? (posl + posr) / 2.0 : posl;

            packet.put("left liftPos", posl);
            packet.put("right liftPos", posr);
            packet.put("avg liftPos", avgPos);

            if (avgPos > 100.0) {
                return true;
            } else {
                //negative gravity
                liftl.setPower(0.01);
                liftr.setPower(0.01);
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
                liftl.setPower(0.8);
                liftr.setPower(0.8);
                initialized = true;
            }

            double posl = liftl.getCurrentPosition();
            double posr = liftr.getCurrentPosition();
            double avgPos = (Math.abs(posl - posr) <= 50) ? (posl + posr) / 2.0 : posl;

            packet.put("left liftPos", posl);
            packet.put("right liftPos", posr);
            packet.put("avg liftPos", avgPos);

            if (avgPos >= 395 && avgPos <= 405) {
                liftl.setPower(0.01);
                liftr.setPower(0.01);
                return false;
            } else {
                return true;
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
                liftl.setPower(0.8);
                liftr.setPower(0.8);
                initialized = true;
            }

            double posl = liftl.getCurrentPosition();
            double posr = liftr.getCurrentPosition();
            double avgPos = (Math.abs(posl - posr) <= 50) ? (posl + posr) / 2.0 : posl;

            packet.put("left liftPos", posl);
            packet.put("right liftPos", posr);
            packet.put("avg liftPos", avgPos);

            if (avgPos >= 1395 && avgPos <= 1405) {
                liftl.setPower(0.01);
                liftr.setPower(0.01);
                return false;
            } else {
                return true;
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
                initialized = true;
                liftl.setPower(0.8);
                liftr.setPower(0.8);
            }

            double posl = liftl.getCurrentPosition();
            double posr = liftr.getCurrentPosition();
            double avgPos = (Math.abs(posl - posr) <= 50) ? (posl + posr) / 2.0 : posl;

            packet.put("left liftPos", posl);
            packet.put("right liftPos", posr);
            packet.put("avg liftPos", avgPos);

            // stop when reaching high basket
            if (avgPos < 2000) {
                return true;
            } else {
                //negative gravity
                liftl.setPower(0.01);
                liftr.setPower(0.01);
                return false;
            }
        }
    }
    public Action toHighBasket() {
        return new LiftHighBasket();
    }
}
