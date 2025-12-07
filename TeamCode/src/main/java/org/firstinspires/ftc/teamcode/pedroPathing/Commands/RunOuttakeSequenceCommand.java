package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.OuttakeSubsystem;

public class RunOuttakeSequenceCommand extends SequentialCommandGroup {

    private static final double SERVO_FIRE = 0.85;
    private static final double SERVO_HOLD = 0.93;

    public RunOuttakeSequenceCommand(IntakeSubsystem intake, OuttakeSubsystem outtake) {
        super(
                new RunOuttakeShooterCommand(outtake, 0.2),
                new WaitCommand(1500),
                new RunIntakeBeltCommand(intake, 0.50),
                new RunIntakeBrushCommand(intake, 0.4),
                new RunOuttakeServoPosCommand(outtake, SERVO_FIRE),
                new WaitCommand(450),
                new RunOuttakeServoPosCommand(outtake, SERVO_HOLD),
                new WaitCommand(1000),
                new RunOuttakeServoPosCommand(outtake, SERVO_FIRE),
                new WaitCommand(1700),


                new RunOuttakeShooterCommand(outtake, 0.0),
                new RunIntakeBrushCommand(intake, 0.0),
                new RunIntakeBeltCommand(intake, 0.0),
                new RunOuttakeServoPosCommand(outtake, SERVO_HOLD)
        );
    }
}