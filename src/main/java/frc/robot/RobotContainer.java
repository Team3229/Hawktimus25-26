// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

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
import swervelib.SwerveInputStream;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.subsystems.drive.HubAlign;

public class RobotContainer {

	FlightStick driverController;
	FlightStick manipController;
	ButtonBoard buttonBoard;
	DriveSubsystem driveSubsystem;
	ManipSubsystem manipSubsystem;
	HubAlign hubAlign;

	VisualizerSubsystem visualizerSubsystem;

	private SendableChooser<Command> autoChooser;
	private Command autoCommand;

	public RobotContainer() {

		driverController = new FlightStick(0);
		manipController = new FlightStick(1);
		buttonBoard = new ButtonBoard(2);

		// buttonBoard = new ButtonBoard(1);
		driveSubsystem = new DriveSubsystem(
			"swerve",
			TelemetryVerbosity.HIGH
		);
		manipSubsystem = new ManipSubsystem();
		hubAlign = new HubAlign();

		configureBindings();
		initTelemetery();
	}

	private void configureBindings() {

		DriverStation.silenceJoystickConnectionWarning(true);

		configDriveControls();
		configManipControls();
		configButtonControls();

	}

	public void teleopInit() {

		System.out.println("TELEOP INIT");
		driveSubsystem.zeroGyroWithAlliance();
		
	}

	public void autoInit() {
		driveSubsystem.zeroGyroWithAlliance();
	}

	private void configDriveControls() {

		NamedCommands.registerCommand("Intake", manipSubsystem.intake());
		NamedCommands.registerCommand("ArmOut", manipSubsystem.extendStorage());
		NamedCommands.registerCommand("WheelSpinUp", manipSubsystem.spinUp());
		NamedCommands.registerCommand("Shoot", manipSubsystem.shoot());

		SwerveInputStream driveAngularVelocity = driveSubsystem.getInputStream(
			() -> -driverController.a_Y(),
			() -> -driverController.a_X(),
			() -> -driverController.a_Z()
		)
			.deadband(0.1)
			.cubeRotationControllerAxis(true)
			.cubeTranslationControllerAxis(true)
			.scaleTranslation(0.8)
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
			driveSubsystem.zeroGyroWithAllianceCommand()
		);

		driverController.b_3().onTrue(
			driveSubsystem.zeroGyroWithLimelight()
		);

		driverController.b_Hazard().onTrue(
			Commands.runOnce(() -> {
				driveSubsystem.getCurrentCommand().cancel();
				// cancels ALL DRIVING on driver controller
			})
		);

		driverController.b_6().onTrue(
			driveSubsystem.slowDrive()
		);

		driverController.b_Trigger().onTrue(
			hubAlign.alignToHub()
		);

	}

	private void configManipControls() {
		
		manipController.b_Trigger().whileTrue(
			manipSubsystem.shoot()
		);

		manipController.b_3().whileTrue(
			manipSubsystem.spinUp()
		);

		manipController.b_Hazard().onTrue(
			Commands.runOnce(() -> {
				manipSubsystem.getCurrentCommand().cancel();
				// cancels ALL manipING on manip controller
			})
		);

		manipController.b_5().onTrue(
			manipSubsystem.extendStorage()
		);

		manipController.b_4().onTrue(
			manipSubsystem.intake()
		);

		manipController.b_12().onTrue(
			manipSubsystem.stow()
		);

	}

	private void configButtonControls() {

		buttonBoard.b_1().onTrue(
			manipSubsystem.shoot()
		);

		buttonBoard.b_2().onTrue(
			manipSubsystem.spinUp()
		);

		buttonBoard.b_3().onTrue(
			manipSubsystem.intake()
		);

		buttonBoard.b_4().onTrue(
			manipSubsystem.extendStorage()
		);

		buttonBoard.b_5().onTrue(
			manipSubsystem.manualShoot()
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
				poses.addAll(path.getAllPathPoints().stream().map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d())).collect(Collectors.toList()));
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