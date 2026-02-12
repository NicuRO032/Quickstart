package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AutoAimTurretCommand extends CommandBase {

    private final TurretSubsystem turret;
    private final VisionSubsystem vision;
    private final int targetId; // NOU: Variabilă pentru a stoca ID-ul țintei
    private AprilTagDetection bestTag;

    /**
     * Constructor actualizat pentru a primi și ID-ul țintei.
     * @param turret Subsistemul turelei
     * @param vision Subsistemul de viziune
     * @param targetId ID-ul AprilTag-ului pe care trebuie să-l urmărim (ex: 20, 24)
     */
    public AutoAimTurretCommand(TurretSubsystem turret, VisionSubsystem vision, int targetId) {
        this.turret = turret;
        this.vision = vision;
        this.targetId = targetId; // NOU: Salvăm ID-ul țintei
        addRequirements(turret, vision);
    }

    @Override
    public void initialize() {
        // La început, doar actualizăm detecția curentă.
        this.bestTag = vision.getBestDetection();
    }

    @Override
    public void execute() {
        // Această metodă este apelată în buclă.
        this.bestTag = vision.getBestDetection();

        // NOU: Apelăm metoda commandAutoAim cu ambii parametri.
        // Logica de verificare (dacă tag-ul e null sau ID-ul e greșit) este acum în interiorul subsistemului.
        turret.commandAutoAim(bestTag, this.targetId);
    }

    @Override
    public boolean isFinished() {
        // Comanda se termină în una din două situații:
        this.bestTag = vision.getBestDetection(); // Re-actualizăm tag-ul

        // 1. Nu vedem un tag SAU tag-ul vizibil nu este cel pe care îl căutăm.
        if (bestTag == null || bestTag.metadata == null || bestTag.id != this.targetId) {
            return true; // Eșec, ieșim imediat pentru a nu rămâne blocați
        }

        // 2. Turela s-a aliniat cu ținta corectă (eroarea de bearing este în toleranță)
        double error = bestTag.ftcPose.bearing;
        return Math.abs(error) < TurretSubsystem.AIMING_TOLERANCE_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        // Când comanda se termină (fie că s-a aliniat, fie că a eșuat sau a fost întreruptă),
        // comutăm turela înapoi în modul de menținere a poziției curente.
        turret.setTargetAngle(turret.getCurrentAngle());
    }
}
