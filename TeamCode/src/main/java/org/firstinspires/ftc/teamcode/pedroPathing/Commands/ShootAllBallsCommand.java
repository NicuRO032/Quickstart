package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;

public class ShootAllBallsCommand extends SequentialCommandGroup {

    public static final long SHOOTER_TIMEOUT_MS = 3000;

    public ShootAllBallsCommand(CarouselSubsystem1 carousel) {
        addCommands(
                // 1. Așteaptă până când caruselul este pregătit (aliniat și shooter la turație)
                new WaitUntilCommand(carousel::isReadyToShoot).withTimeout(SHOOTER_TIMEOUT_MS),
                new WaitCommand(25),

                // 2. Comandă declanșarea salvei
                new InstantCommand(carousel::triggerShoot),

                // 3. Așteaptă până când subsistemul se resetează singur în starea IDLE după finalizare
                new WaitUntilCommand(carousel::allSlotsEmpty)
        );
        addRequirements(carousel);
    }


}

