package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;

public class AutoAimTurretCommand extends CommandBase {

    private final TurretSubsystem turret;
    private final VisionSubsystem vision;
    private final int targetId;

    /**
     * Constructor actualizat pentru Limelight.
     */
    public AutoAimTurretCommand(TurretSubsystem turret, VisionSubsystem vision, int targetId) {
        this.turret = turret;
        this.vision = vision;
        this.targetId = targetId;
        addRequirements(turret, vision);
    }

    @Override
    public void initialize() {
        // Nu mai avem nevoie de initialize pentru bestTag
    }

    @Override
    public void execute() {
        // Trimitem datele brute de la Limelight direct la subsistemul de turelă
        turret.commandAutoAim(
                vision.hasValidTag(),
                vision.getLastTagId(),
                vision.getLastBearing(),
                this.targetId
        );
    }

    @Override
    public boolean isFinished() {
        // 1. Verificăm dacă vedem ceva și dacă este ID-ul corect
        if (!vision.hasValidTag() || vision.getLastTagId() != this.targetId) {
            return true; // Eșec sau pierdere țintă, oprim comanda
        }

        // 2. Verificăm dacă eroarea orizontală (bearing/tx) este în toleranță
        // Folosim direct getLastBearing() care vine de la Limelight
        double error = vision.getLastBearing();

        // Comanda se termină cu succes când eroarea este mică
        return Math.abs(error) < TurretSubsystem.AIMING_TOLERANCE_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        // Menținem unghiul curent pentru a nu lăsa turela liberă (Holding State)
        turret.setTargetAngle(turret.getCurrentAngle());
    }
}