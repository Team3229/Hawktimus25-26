// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.inputs.ButtonBoard;
import frc.robot.inputs.FlightStick;
import frc.robot.subsystems.VisualizerSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.manipSubsystems.ManipSubsystem;
import frc.robot.subsystems.manipSubsystems.PathPlannerCommands;
import swervelib.SwerveInputStream;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class RobotContainer {

	FlightStick driverController;
	FlightStick manipController;
	ButtonBoard buttonBoard;
	DriveSubsystem driveSubsystem;
	ManipSubsystem manipSubsystem;

	
	VisualizerSubsystem visualizerSubsystem;
	PathPlannerCommands pathPlannerCommands;

	private SendableChooser<Command> autoChooser;
	private Command autoCommand;

	public RobotContainer() {

		driverController = new FlightStick(0);
		manipController = new FlightStick(1);

		driveSubsystem = new DriveSubsystem(
			"swerve",
			TelemetryVerbosity.HIGH
		);
		manipSubsystem = new ManipSubsystem(driveSubsystem);

		pathPlannerCommands = new PathPlannerCommands(manipSubsystem);

		configureBindings();
		initTelemetery();
	}

	private void configureBindings() {

		NamedCommands.registerCommand("Intake", pathPlannerCommands.pathIntake());
		NamedCommands.registerCommand("ArmOut", manipSubsystem.intakeArmOut());
		NamedCommands.registerCommand("WheelSpinUp", pathPlannerCommands.pathSpinUp());
		NamedCommands.registerCommand("Shoot", pathPlannerCommands.pathShoot());
		NamedCommands.registerCommand("Stow", manipSubsystem.stow());

		DriverStation.silenceJoystickConnectionWarning(true);

		configDriveControls();
		configManipControls();
		
	}

	public void teleopInit() {

		System.out.println("TELEOP INIT");
		
	}

	public void autoInit() {
		driveSubsystem.zeroGyroCommand();
	}

	private void configDriveControls() {

		SwerveInputStream driveAngularVelocity = driveSubsystem.getInputStream(
			() -> -driverController.a_Y(),
			() -> -driverController.a_X(),
			() -> -driverController.a_Z()
		)
			.deadband(0.1)
			.cubeRotationControllerAxis(true)
			.cubeTranslationControllerAxis(true)
			.scaleTranslation(0.6)
			.scaleRotation(0.9)
			.allianceRelativeControl(true);
			
		driveSubsystem.setDefaultCommand(
			driveSubsystem.driveFieldOriented(
				driveAngularVelocity
			)
		);

		driverController.b_10().onTrue(
			driveSubsystem.zeroGyroWithLimelight()
		);

		driverController.b_11().onTrue(
			driveSubsystem.zeroGyroCommand()
		);

		driverController.b_Hazard().onTrue(
			Commands.runOnce(() -> {
				driveSubsystem.getCurrentCommand().cancel();
				// cancels ALL DRIVING on driver controller
			})
		);

		driverController.b_Trigger().whileTrue(
			driveSubsystem.toggleHubAlign()
		);

	}

	private void configManipControls() {
		// CURRENTLY AVAILABLE: 6, 11, slider

		manipController.b_Trigger().whileTrue(
			manipSubsystem.shoot()
		);

		manipController.b_Hazard().onTrue(
			manipSubsystem.stow()
		);

		manipController.b_3().whileTrue(
			manipSubsystem.spinUp()
		);

		manipController.b_4().whileTrue(
			manipSubsystem.intake()
		);

		manipController.b_5().onTrue(
			manipSubsystem.intakeArmOut()
		);

		manipController.b_10().whileTrue(
			manipSubsystem.extake()
		);

		manipController.b_12().onTrue(
			Commands.runOnce(() -> {
				manipSubsystem.getCurrentCommand().cancel(); // TODO: currently crashes bot
				// cancels ALL manipING on manip controller
			})
		);

		manipController.p_Up().onTrue(
			manipSubsystem.upSRPSCommand()
		);

		manipController.p_Down().onTrue(
			manipSubsystem.downSRPSCommand()
		);

		manipController.p_Right().onTrue(
			manipSubsystem.upFRPSCommand()
		);

		manipController.p_Left().onTrue(
			manipSubsystem.downFRPSCommand()
		);

	}

	public void initTelemetery() {
		SmartDashboard.putData(CommandScheduler.getInstance());


		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Autonomous Chooser", autoChooser);

		autoChooser.onChange(
			(selected) -> {
				pathPreview(selected.getName());
				preloadAutoCommand(selected);
			}
		);
	}

	public void pathPreview(String autoName) {

		System.out.println("Displaying " + autoName);
        List<PathPlannerPath> pathPlannerPaths;
		try {
			pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
			List<Pose2d> poses = new ArrayList<>();
			for (PathPlannerPath path : pathPlannerPaths) {
				poses.addAll(path.getAllPathPoints().stream().map( point -> 
					new Pose2d(point.position.getX(), point.position.getY(), 
					new Rotation2d())).collect(Collectors.toList())
				);
			}
			driveSubsystem.postTrajectoryToField(poses);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void preloadAutoCommand(Command command) {
		autoCommand = command;
	}

	public Command getAutonomousCommand() {
		return autoCommand;
	}

}