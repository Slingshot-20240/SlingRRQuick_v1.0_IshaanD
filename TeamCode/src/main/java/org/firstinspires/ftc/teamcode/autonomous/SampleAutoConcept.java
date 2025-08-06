package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmClaw;
import org.firstinspires.ftc.teamcode.subsystems.ClawPivot;
import org.firstinspires.ftc.teamcode.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.IntakePivot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.opencv.core.Mat;

@Config
@Autonomous(name = "ITD_SampleAuton", group = "Autonomous")
public class SampleAutoConcept extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-63, 39, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ActiveIntake activeIntake = new ActiveIntake(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        ArmClaw armClaw = new ArmClaw(hardwareMap);
        ClawPivot clawPivot = new ClawPivot(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        IntakePivot intakePivot = new IntakePivot(hardwareMap);
        Lift lift = new Lift(hardwareMap);


        Action scorePreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-58,57),Math.toRadians(315))
                .build();

        Action grabPickup1 = drive.actionBuilder(new Pose2d(-58, 57, Math.toRadians(315)))
                .strafeToLinearHeading(new Vector2d(-35,49),Math.toRadians(0))
                .build();

        Action scorePickup1 = drive.actionBuilder(new Pose2d(-35, 49, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-58,57),Math.toRadians(315))
                .build();

        Action grabPickup2 = drive.actionBuilder(new Pose2d(-58, 57, Math.toRadians(315)))
                .strafeToLinearHeading(new Vector2d(-29,58),Math.toRadians(0))
                .build();

        Action scorePickup2 = drive.actionBuilder(new Pose2d(-29, 58, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-58,57),Math.toRadians(315))
                .build();

        Action grabPickup3 = drive.actionBuilder(new Pose2d(-58, 57, Math.toRadians(315)))
                .strafeToLinearHeading(new Vector2d(-23,63),Math.toRadians(0))
                .build();

        Action scorePickup3 = drive.actionBuilder(new Pose2d(-23, 63, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-58,57),Math.toRadians(315))
                .build();

        Action park = drive.actionBuilder(new Pose2d(-58,57,Math.toRadians(315)))
                //Control Pose
                .strafeToLinearHeading(new Vector2d(-12,32), Math.toRadians(90))
                //Park Pose
                .strafeToLinearHeading(new Vector2d(-12,26),Math.toRadians(90))
                .build();

        // Initialize
        Actions.runBlocking(armClaw.close());


        waitForStart();

        if (isStopRequested()) return;





        //Make code with Actions here
        Actions.runBlocking(
                new SequentialAction(

                        //----------Score preload----------\\
                        new ParallelAction(
                                scorePreload,
                                AutonSequences.scoreHigh()
                        ),
                        //Start active intake
                        activeIntake.in(),

                        //----------Block 1 cycle----------\\
                        new ParallelAction(
                                grabPickup1,
                                AutonSequences.readyForPickup()
                        ),
                        AutonSequences.pickUp(3),
                        AutonSequences.transferBlock(),

                        //Score Pickup 1
                        new ParallelAction(
                                scorePickup1,
                                AutonSequences.scoreHigh()
                        ),

                        //----------Block 1 cycle----------\\
                        //Grab Block 2
                        new ParallelAction(
                                grabPickup2,
                                AutonSequences.readyForPickup()
                        ),
                        AutonSequences.pickUp(3),
                        AutonSequences.transferBlock(),

                        //Score Pickup 2
                        new ParallelAction(
                                scorePickup2,
                                AutonSequences.scoreHigh()
                        ),

                        //----------Block 3 cycle----------\\
                        //Grab Block 3
                        new ParallelAction(
                                grabPickup3,
                                AutonSequences.readyForPickup()
                        ),
                        AutonSequences.pickUp(3),
                        AutonSequences.transferBlock(),

                        //Score Pickup 1
                        new ParallelAction(
                                scorePickup3,
                                AutonSequences.scorePickup3()
                        ),
                        extendo.mini_out(),

                        //----------Park----------\\
                        new ParallelAction(
                                park,
                                //reset all subsystems
                                lift.toDown(),
                                extendo.in(),
                                activeIntake.idle()
                        )
                )
        );


    }
}