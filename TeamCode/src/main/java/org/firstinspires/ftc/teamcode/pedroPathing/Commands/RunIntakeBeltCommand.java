package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;

public class RunIntakeBeltCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final double beltMotorPower;

    public RunIntakeBeltCommand(IntakeSubsystem intake, double beltMotorPower) {
        this.intake = intake;
        this.beltMotorPower = beltMotorPower;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setBeltMotorPower(beltMotorPower);
    }

    @Override
    public boolean isFinished() {
        return true; // instant command
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopBeltMotor();
    }
}
