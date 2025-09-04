package org.firstinspires.ftc.teamcode.mechanisms_and_movement.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "FTC Dashboard Servo Control", group = "Dashboard")
public class ServoDashTester extends LinearOpMode {

    // Exposed to FTC Dashboard
    public static double SERVO_POSITION = 0.5; // between 0.0 and 1.0

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "servo"); // make sure this matches your config

        waitForStart();

        while (opModeIsActive()) {
            // Continuously set servo to dashboard value
            servo.setPosition(SERVO_POSITION);

            telemetry.addData("Servo Position: ", SERVO_POSITION);
            telemetry.update();
        }
    }
}
