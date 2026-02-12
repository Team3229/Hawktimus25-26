package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathPlannerCommands extends SubsystemBase {

	ManipSubsystem manipSubsystem;
	
    public PathPlannerCommands () {
        manipSubsystem = new ManipSubsystem();
    }
}