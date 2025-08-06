package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.subsystems.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmClaw;
import org.firstinspires.ftc.teamcode.subsystems.ClawPivot;
import org.firstinspires.ftc.teamcode.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.IntakePivot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


public class AutonSequences {

    static ActiveIntake activeIntake = new ActiveIntake(hardwareMap);
    static Arm arm = new Arm(hardwareMap);
    static ArmClaw armClaw = new ArmClaw(hardwareMap);
    static ClawPivot clawPivot = new ClawPivot(hardwareMap);
    static Extendo extendo = new Extendo(hardwareMap);
    static IntakePivot intakePivot = new IntakePivot(hardwareMap);
    static Lift lift = new Lift(hardwareMap);
    /**
     * Lift up, Arm scores
     */
    public static Action scoreHigh() {
        return new SequentialAction(
                new ParallelAction(
                        lift.toHighBasket(),
                        extendo.out()
                ),
                new ParallelAction(
                        arm.toScore(),
                        clawPivot.toScore(),
                        intakePivot.toIntake()
                ),
                //Tune delay for optimal scoring
                new SleepAction(1),
                armClaw.open()

        );
    }

    /**
     * Arm, ClawPivot, and Lift ready for Transfer
     */
    public static Action readyForPickup() {
        return new SequentialAction(
                new ParallelAction(
                        lift.toTransfer(),
                        arm.toTransfer(),
                        clawPivot.toTransfer()
                )
        );
    }

    /**
     * Intake in for (seconds), then IntakePivot ready for transfer
     * @param seconds - Time the intake runs for
     */
    public static Action pickUp(double seconds) {
        return new SequentialAction(
                activeIntake.in(),
                new SleepAction(seconds),

                new ParallelAction(
                        activeIntake.idle(),
                        intakePivot.toTransfer()
                )
        );
    }
    /**
     * Extendo in, Claw grabs
     */
    public static Action transferBlock() {
        return new SequentialAction(
                extendo.in(),
                //Move block to claw
                new ParallelAction(
                        moveBlockInClaw(),
                        extendo.mini_out()
                )

        );
    }
    /**
     * Pushes block towards claw for transfer
     */
    public static Action moveBlockInClaw() {
        return new SequentialAction(
                activeIntake.out(),
                new SleepAction(0.8),
                armClaw.close(),
                activeIntake.idle()
        );
    }

    public static Action scorePickup3() {
        return new SequentialAction(

                lift.toHighBasket(),

                new ParallelAction(
                        arm.toScore(),
                        clawPivot.toScore()
                ),

                //Tune Delay for optimal scoring
                new SleepAction(1),
                armClaw.open()

        );
    }




}
