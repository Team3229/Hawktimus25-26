package frc.robot.subsystems.manipSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathPlannerCommands extends SubsystemBase {

	ManipSubsystem manipSubsystem;
	
    public PathPlannerCommands(ManipSubsystem man) {
        manipSubsystem = man;
    }

    public Command pathIntake() {
        return manipSubsystem.intake().withTimeout(3);
    }

    public Command pathSpinUp() {
        return manipSubsystem.spinUp().withTimeout(6);
    }

    public Command pathShoot() {
        return manipSubsystem.shoot().withTimeout(7);
    }

    public Command pathExtake() {
        return manipSubsystem.extake().withTimeout(3);
    }

    public Command pathStow() {
        return manipSubsystem.stow().withTimeout(1);
    }
}