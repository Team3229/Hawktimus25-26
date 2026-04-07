package frc.robot.subsystems.manipSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        return new ParallelCommandGroup(
            manipSubsystem.spinUp().withTimeout(5.5),
            Commands.runOnce(() -> manipSubsystem.driveSubsystem.distanceFromHub())
        );
    }

    public Command pathShoot() {
        return new ParallelCommandGroup(
            manipSubsystem.shoot().withTimeout(5.5),
            Commands.runOnce(() -> manipSubsystem.driveSubsystem.distanceFromHub())
        );
    }

    public Command pathExtake() {
        return manipSubsystem.extake().withTimeout(3);
    }

    public Command pathStow() {
        return manipSubsystem.stow().withTimeout(3);
    }
}