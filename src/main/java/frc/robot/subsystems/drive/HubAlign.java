package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class HubAlign {
    DriveSubsystem driveSubsystem;
    public HubAlign() {
        driveSubsystem = new DriveSubsystem(
            "swerve",
			TelemetryVerbosity.HIGH
        );
    }

    	// Might be the right dimensions
	public Translation2d getHubPose() {
		return new Translation2d(4.034663d, 4.611624d);
	}
	
	/*
	* Returns the angle from the robot to the hub (in radians)
	*/
	public double angleFromHub() {
		return Math.atan2(getHubPose().getY() - driveSubsystem.getPose().getY(), getHubPose().getX() - driveSubsystem.getPose().getX());
	}
	
	/*
	* Returns the distance from the robot to the hub 
	*/
	public double distanceFromHub() {
		return Math.sqrt(Math.pow(getHubPose().getX() - driveSubsystem.getPose().getX(), 2) + Math.pow(getHubPose().getY() - driveSubsystem.getPose().getY(), 2));
	}
	
	/**Rotates the bot to be facing the hub*/
    public Command alignToHub() {
		Command out = new Command() {
			@Override
			public void execute() {
				driveSubsystem.driveToPose(() -> {
					return new Pose2d(
						driveSubsystem.getPose().getX(),
						driveSubsystem.getPose().getY(),
						driveSubsystem.getPose().getRotation().plus(new Rotation2d(angleFromHub()))
					);
				});
			};
		};

		return out;
		
	};
}
