package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;

public class ShootAllBallsCommand extends SequentialCommandGroup {

    public static final long SHOOTER_TIMEOUT_MS = 250; // Timp maxim de așteptare: 3 secunde

    public ShootAllBallsCommand(CarouselSubsystem1 carousel) {
        addCommands(
                new ParallelRaceGroup(
                        new WaitUntilCommand(() ->
                                (carousel.getShooterCurrentRPM() >= carousel.getShooterTargetRPM() * 0.95)
                                        &&
                                        (carousel.isReadyToShoot())
                        ),
                        new WaitCommand(SHOOTER_TIMEOUT_MS)
                ),
                new InstantCommand(carousel::triggerShoot),
                new WaitUntilCommand(() -> carousel.getOuttakeState().equals("OUT_IDLE"))
        );
        addRequirements(carousel);
    }
}