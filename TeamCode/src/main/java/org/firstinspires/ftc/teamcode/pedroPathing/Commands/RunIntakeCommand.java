package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;

public class RunIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final double power;

    public RunIntakeCommand(IntakeSubsystem intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true; // instant command
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
