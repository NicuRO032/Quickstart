package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;

public class RunIntakeBrushCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final double brushMotorPower;

    public RunIntakeBrushCommand(IntakeSubsystem intake, double brushMotorPower) {
        this.intake = intake;
        this.brushMotorPower = brushMotorPower;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setBrushMotorPower(brushMotorPower);
    }

    @Override
    public boolean isFinished() {
        return true; // instant command
    }

    @Override
    public void end(boolean interrupted) {
        //intake.stopBrushMotor();
    }
}
