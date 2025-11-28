package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.OuttakeSubsystem;


import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

public class RunOuttakeSequenceCommand extends SequentialCommandGroup {

    private static final double SERVO_FIRE = 0.83;
    private static final double SERVO_HOLD = 0.87;
    public RunOuttakeSequenceCommand(IntakeSubsystem intake, OuttakeSubsystem outtake) {
        addCommands(
                new RunOuttakeShooterCommand(outtake, 0.75),
                new WaitCommand(1000),
                new RunIntakeBeltCommand(intake, 0.5),
                new RunIntakeBrushCommand(intake, 0.5),
                new RunOuttakeServoPosCommand(outtake,SERVO_FIRE),

                new WaitCommand(2000),

                new RunOuttakeShooterCommand(outtake, 0.0),
                new RunIntakeBrushCommand(intake, 0.0),
                new RunIntakeBeltCommand(intake, 0.0),
                new RunOuttakeServoPosCommand(outtake,SERVO_HOLD)

        );
        addRequirements(intake, outtake);
    }
}
