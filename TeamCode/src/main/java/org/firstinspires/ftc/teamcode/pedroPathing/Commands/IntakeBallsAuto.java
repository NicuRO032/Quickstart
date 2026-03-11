package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem1;

public class IntakeBallsAuto extends SequentialCommandGroup {


    public IntakeBallsAuto (CarouselSubsystem1 carousel, IntakeSubsystem1 intake){
        addCommands(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            carousel.activateIntake();
                            carousel.setCarouselForIntake();
                            intake.collect();
                        }),

                        // 3. Așteaptă până când subsistemul se resetează singur în starea IDLE după finalizare
                        new WaitUntilCommand(() -> intake.getState() == IntakeSubsystem1.IntakeState.CLEANUP_BALL3),
                        new WaitCommand(300)
                )
        );
        addRequirements(carousel, intake);
    }
}
