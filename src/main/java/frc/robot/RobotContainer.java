// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.inputs.ButtonBoard;
import frc.robot.inputs.FlightStick;
import frc.robot.subsystems.VisualizerSubsystem;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveConstants;

import frc.robot.subsystems.manipSubsystems.ManipSubsystem;
import frc.robot.subsystems.manipSubsystems.PathPlannerCommands;
import frc.robot.subsystems.manipSubsystems.SpitterSubsystem;


public class RobotContainer {
	public final DriveSubsystem drivetrain = DriveConstants.createDrivetrain();
	private double MaxSpeed = 1.0 * DriveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.205).in(RadiansPerSecond); 
	
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(MaxSpeed * 0.1)
		.withRotationalDeadband(MaxAngularRate * 0.1) 
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.RobotCentric robotRelative = new SwerveRequest.RobotCentric()
		.withDeadband(MaxSpeed * 0.1)
		.withRotationalDeadband(MaxAngularRate * 0.1) 
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.FieldCentricFacingAngle hubAlign = new SwerveRequest.FieldCentricFacingAngle()
		.withDeadband(MaxSpeed * 0.1)
		.withHeadingPID(2, 0, 0)
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	FlightStick driverController;
	FlightStick manipController;
	ButtonBoard buttonBoard;
	ManipSubsystem manipSubsystem;
	SpitterSubsystem spitterSubsystem;
	
	VisualizerSubsystem visualizerSubsystem;
	PathPlannerCommands pathPlannerCommands;

	private SendableChooser<Command> autoChooser;
	private Command autoCommand;


	public RobotContainer() {
		CameraServer.startAutomaticCapture("Intake Camera", 0);

		driverController = new FlightStick(0);
		manipController = new FlightStick(1);

		manipSubsystem = new ManipSubsystem(drivetrain);
		spitterSubsystem = new SpitterSubsystem(drivetrain);

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
		NamedCommands.registerCommand("ZeroGyroWithLimelight", drivetrain.zeroGyroWithLimelight());

		DriverStation.silenceJoystickConnectionWarning(true); // TODO: MAKE THIS FALSE FOR COMP!!!!!!!!!!!!!!!!
		

		configDriveControls();
		configManipControls();
		
		final var idle = new SwerveRequest.Idle();
        	RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(false) //check on this wording is weird I think should be false based on reading
        );
	}
	
	public void teleopInit() {

		System.out.println("TELEOP INIT");
	
	}

	public void autoInit() {
		drivetrain.zeroGyroCommand();
	}

	private void configDriveControls() {
		drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
			drivetrain.applyRequest(() ->
				drive.withVelocityX(Math.pow(-driverController.a_Y(), 3) * MaxSpeed) // Drive forward with negative Y (forward)
				.withVelocityY(Math.pow(-driverController.a_X(), 3) * MaxSpeed) // Drive left with negative X (left)
				.withRotationalRate(Math.pow(-driverController.a_Z(), 3) * MaxAngularRate) // Drive counterclockwise with negative X (left)
			) 
        );

		driverController.b_10().onTrue(
			drivetrain.zeroGyroWithLimelight()
		);

		driverController.b_11().onTrue(
			drivetrain.zeroGyroCommand()
		);

		driverController.b_Hazard().onTrue(
			Commands.runOnce(() -> {
				drivetrain.getCurrentCommand().cancel();
				// cancels ALL DRIVING on driver controller
			})
		);

		driverController.p_Any().whileTrue(
			drivetrain.applyRequest(() ->
				robotRelative.withVelocityX(Math.pow(-driverController.a_Y(), 3) * MaxSpeed)
				.withVelocityY(Math.pow(-driverController.a_X(), 3) * MaxSpeed)
				.withRotationalRate(Math.pow(-driverController.a_Z(), 3) * MaxAngularRate)
			)
		);

		driverController.b_Trigger().whileTrue(
			drivetrain.applyRequest(() -> 
				hubAlign.withTargetDirection(drivetrain.getTargetTranslation().getAngle().rotateBy(Rotation2d.k180deg))
				.withVelocityX(Math.pow(-driverController.a_Y(), 3) * MaxSpeed)
				.withVelocityY(Math.pow(-driverController.a_X(), 3) * MaxSpeed)
			)
		);

	}

	private void configManipControls() {
		// TODO: delete

		manipController.b_7().whileTrue(
			manipSubsystem.spinShooter()
		);

		manipController.b_8().whileTrue(
			manipSubsystem.spinKicker()
		);

		// TODO: delete

		// CURRENTLY AVAILABLE: 6, 7, 8, 9, 11, slider

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

		manipController.p_Up().onTrue(
			manipSubsystem.upSRPSCommand()
		);

		manipController.p_Down().onTrue(
			manipSubsystem.downSRPSCommand()
		);

		// manipController.p_Right().onTrue(
		// 	manipSubsystem.upFRPSCommand()
		// );

		// manipController.p_Left().onTrue(
		// 	manipSubsystem.downFRPSCommand()
		// );
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
			drivetrain.postTrajectoryToField(poses);
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