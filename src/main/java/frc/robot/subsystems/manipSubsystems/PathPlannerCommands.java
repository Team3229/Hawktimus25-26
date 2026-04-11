package frc.robot.subsystems.manipSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathPlannerCommands extends SubsystemBase {

	ManipSubsystem manipSubsystem;
	
    public PathPlannerCommands(ManipSubsystem man) {
        manipSubsystem = man;
    }

    public Command pathIntake() {
        return manipSubsystem.intake().withTimeout(9);
    }

    public Command pathSpinUp() {
        return manipSubsystem.spinUp().withTimeout(5.5);
    }

    public Command pathShoot() {
        return manipSubsystem.shoot().withTimeout(5);
    }

    public Command pathExtake() {
        return manipSubsystem.extake().withTimeout(3);
    }

    public Command pathStow() {
        return manipSubsystem.stow();
    }

    public Command pathIntakeArmOut() {
        return manipSubsystem.forceIntakeArmOut();
    }

    public Command pathIntakeStop() {
        return manipSubsystem.intake().withTimeout(0.00001);
    }

}