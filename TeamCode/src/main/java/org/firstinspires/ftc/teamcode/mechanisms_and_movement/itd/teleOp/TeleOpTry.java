package org.firstinspires.ftc.teamcode.mechanisms_and_movement.itd.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms_and_movement.itd.subsystems.ActiveIntake;
import org.firstinspires.ftc.teamcode.mechanisms_and_movement.itd.subsystems.Arm;
import org.firstinspires.ftc.teamcode.mechanisms_and_movement.itd.subsystems.ArmClaw;
import org.firstinspires.ftc.teamcode.mechanisms_and_movement.itd.subsystems.ClawPivot;
import org.firstinspires.ftc.teamcode.mechanisms_and_movement.itd.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms_and_movement.itd.subsystems.IntakePivot;
import org.firstinspires.ftc.teamcode.mechanisms_and_movement.itd.subsystems.Lift;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TeleOpTry extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();


    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    ActiveIntake activeIntake = new ActiveIntake(hardwareMap);
    Arm arm = new Arm(hardwareMap);
    ArmClaw armClaw = new ArmClaw(hardwareMap);
    ClawPivot clawPivot = new ClawPivot(hardwareMap);
    Extendo extendo = new Extendo(hardwareMap);
    IntakePivot intakePivot = new IntakePivot(hardwareMap);
    Lift lift = new Lift(hardwareMap);

    @Override
    public void init() {
        Actions.runBlocking(new SequentialAction(
            new SleepAction(1),
            armClaw.close()
        ));

    }

    @Override
    public void loop() {
    //-------Running Actions Loop-------\\
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

    //---------Actions---------\\


//////////**********  Gamepad 1  **********\\\\\\\\\\
        //Extendo out Lift down Arm transfer Claw Transfer
        if (gamepad1.right_bumper) {
            runningActions.add(new ParallelAction(
                    new InstantAction(() -> lift.toTransfer()),
                    new InstantAction(() -> arm.toTransfer()),
                    new InstantAction(() -> clawPivot.toTransfer()),
                    new InstantAction(() -> armClaw.open()),
                    new InstantAction(() -> extendo.out())

            ));
        }

        //Extendo in and Transfer Block
        if (gamepad1.left_bumper) {
            runningActions.add(new SequentialAction(
                    new ParallelAction(
                        new InstantAction(() -> armClaw.open()),
                        new InstantAction(() -> extendo.in())
                    ),
                    new InstantAction(() -> activeIntake.out()),
                    new SleepAction(0.8),
                    new InstantAction(() -> armClaw.close()),
                    new InstantAction(() -> activeIntake.idle()),
                    new InstantAction(() -> extendo.mini_out())
            ));
        }

        //When pressed - Intake In Pivot Down
        if (gamepad1.right_trigger > 0) {
            runningActions.add(new ParallelAction(
                    new InstantAction(() -> intakePivot.toIntake()),
                    new InstantAction(() -> activeIntake.in())
            ));
        }
        //When released - Intake Idle Pivot Transfer
        if (gamepad1.right_trigger == 0) {
            runningActions.add(new ParallelAction(
                    new InstantAction(() -> intakePivot.toTransfer()),
                    new InstantAction(() -> activeIntake.idle())
            ));
        }

//////////**********  Gamepad 2  **********\\\\\\\\\\

        //Lift to high basket
        if (gamepad2.left_bumper) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> lift.toHighBasket()),
                    new ParallelAction(
                        new InstantAction(() -> arm.toScore()),
                        new InstantAction(() -> clawPivot.toScore()),
                        new InstantAction(() -> extendo.in())
                    )
            ));
        }

        //Lift to low basket
        if (gamepad2.left_trigger > 0) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> lift.toLowBasket()),
                    new ParallelAction(
                            new InstantAction(() -> arm.toScore()),
                            new InstantAction(() -> clawPivot.toScore()),
                            new InstantAction(() -> extendo.in())
                    )
            ));
        }

        //Open armClaw
        if (gamepad2.a) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> armClaw.open())
            ));
        }



        //----Drive Code----\\
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad2.left_stick_y,
                        -gamepad2.left_stick_x
                ),
                -gamepad2.right_stick_x
        ));

        drive.updatePoseEstimate();

        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();

        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);






    }

}
