package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AutoAimTurretCommand extends CommandBase {

    private final TurretSubsystem turret;
    private final VisionSubsystem vision;
    private AprilTagDetection bestTag;

    public AutoAimTurretCommand(TurretSubsystem turret, VisionSubsystem vision) {
        this.turret = turret;
        this.vision = vision;
        addRequirements(turret, vision); // Ne asigurăm că nicio altă comandă nu folosește turela
    }

    @Override
    public void initialize() {
        // La început, doar verificăm dacă avem un tag.
        // Nu facem nimic altceva, logica principală este în execute().
        this.bestTag = vision.getBestDetection();
    }

    @Override
    public void execute() {
        // Această metodă este apelată în buclă (50 de ori pe secundă)
        this.bestTag = vision.getBestDetection();

        // Dacă avem un tag valid, comandăm turelei să-l urmărească
        if (bestTag != null && bestTag.metadata != null) {
            turret.commandAutoAim(bestTag); // Aceasta este logica de la LEFT_BUMPER
        }
        // Dacă nu vedem un tag, turela va rămâne pe ultima poziție comandată.
    }

    @Override
    public boolean isFinished() {
        // Comanda se termină în una din două situații:
        // 1. Nu am văzut niciodată un tag la început.
        if (bestTag == null || bestTag.metadata == null) {
            return true; // Eșec, ieșim imediat
        }

        // 2. Turela s-a aliniat cu ținta (eroarea de bearing este în toleranță)
        double error = bestTag.ftcPose.bearing;
        return Math.abs(error) < TurretSubsystem.AIMING_TOLERANCE_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        // Când comanda se termină (fie că s-a aliniat, fie că a fost întreruptă de timeout),
        // este o practică bună să comutăm turela înapoi în modul de menținere a poziției.
        turret.setTargetAngle(turret.getCurrentAngle());
    }
}