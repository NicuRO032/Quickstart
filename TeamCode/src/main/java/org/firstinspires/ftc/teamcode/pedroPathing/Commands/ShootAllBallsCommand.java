package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;

public class ShootAllBallsCommand extends SequentialCommandGroup {

    public static final long SHOOTER_TIMEOUT_MS = 1000;

    public ShootAllBallsCommand(CarouselSubsystem1 carousel) {
        addCommands(
                // 1. Pregătire: Motor ON și setare ordine 2-1-0
                new InstantCommand(carousel::prepareOuttakeDirect),

                // --- BILA 1 (Slot 2) ---
                new WaitUntilCommand(carousel::isReadyToShoot).withTimeout(SHOOTER_TIMEOUT_MS),
                new InstantCommand(carousel::triggerShoot),

                // --- BILA 2 (Slot 1) ---
                // Așteptăm ca pointerul să crească (semn că bila 1 a plecat)
                new WaitUntilCommand(() -> carousel.getOuttakePtr() == 1).withTimeout(500),
                new WaitUntilCommand(carousel::isReadyToShoot).withTimeout(SHOOTER_TIMEOUT_MS),
                new InstantCommand(carousel::triggerShoot),

                // --- BILA 3 (Slot 0) ---
                new WaitUntilCommand(() -> carousel.getOuttakePtr() == 2).withTimeout(500),
                new WaitUntilCommand(carousel::isReadyToShoot).withTimeout(SHOOTER_TIMEOUT_MS),
                new InstantCommand(carousel::triggerShoot),

                // Finalizare
                new WaitUntilCommand(() -> carousel.getOuttakeStateEnum() == CarouselSubsystem1.OuttakeState.OUT_IDLE)
        );
        addRequirements(carousel);
    }
}