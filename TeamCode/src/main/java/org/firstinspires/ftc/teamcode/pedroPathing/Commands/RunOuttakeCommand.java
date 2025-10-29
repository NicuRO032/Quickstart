package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.OuttakeSubsystem;

public class RunOuttakeCommand extends CommandBase {
    private final OuttakeSubsystem outtake;
    private final double motorPower;
    private final double servoPosition;

    public RunOuttakeCommand(OuttakeSubsystem outtake, double motorPower, double servoPosition) {
        this.outtake = outtake;
        this.motorPower = motorPower;
        this.servoPosition = servoPosition;
        addRequirements(outtake);
    }

    @Override
    public void initialize() {
        outtake.runMotor(motorPower);
        outtake.setServo(servoPosition);
    }

    @Override
    public boolean isFinished() {
        return true; // instant command
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stopMotor();
    }
}
