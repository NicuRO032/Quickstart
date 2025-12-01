package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.OuttakeSubsystem;


import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

public class RunOuttakeSequenceCommand extends SequentialCommandGroup {

    private static final double SERVO_FIRE = 0.85;
    private static final double SERVO_HOLD = 0.90;
    public RunOuttakeSequenceCommand(IntakeSubsystem intake, OuttakeSubsystem outtake) {
        addCommands(

                new RunOuttakeShooterCommand(outtake, 0.5),
                new WaitCommand(1300),
                new RunIntakeBeltCommand(intake, 0.70),
                new RunIntakeBrushCommand(intake, 0.4),
                new RunOuttakeServoPosCommand(outtake,SERVO_FIRE),
                new WaitCommand(300),
                new RunOuttakeServoPosCommand(outtake,SERVO_HOLD),
                new WaitCommand(900),
                new RunOuttakeServoPosCommand(outtake,SERVO_FIRE),
                new WaitCommand(1500),


                new RunOuttakeShooterCommand(outtake, 0.0),
                new RunIntakeBrushCommand(intake, 0.0),
                new RunIntakeBeltCommand(intake, 0.0),
                new RunOuttakeServoPosCommand(outtake,SERVO_HOLD)

        );
        addRequirements(intake, outtake);
    }
}
