package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.hawklibraries.utilities.Alliance;
import frc.hawklibraries.utilities.Alliance.AllianceColor;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.manipSubsystems.SpitterSubsystem;
import frc.robot.utilities.LimelightHelpers;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {
    
    public static LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4.0); // was 5
    
    private static final Distance TRANS_ERR_TOL = Meters.of(0.025); //TODO: Test this with a setpoint
	private static final LinearVelocity TRANS_VEL_TOL = MetersPerSecond.of(0.1);
	private static final Angle ROT_ERR_TOL = Degrees.of(0.5);
	private static final AngularVelocity ROT_VEL_TOL = DegreesPerSecond.of(0.5);

	private static final LinearVelocity TRANS_MAX_VEL = MetersPerSecond.of(3); //TODO: Was 1
	private static final LinearAcceleration TRANS_MAX_ACCEL = MetersPerSecondPerSecond.of(2);

	private static final AngularVelocity ROT_MAX_VEL = DegreesPerSecond.of(540); // TODO: We multiplied these by 0.75
	private static final AngularAcceleration ROT_MAX_ACCEL = DegreesPerSecondPerSecond.of(540);

    private static final Pose2d startingBluePose = new Pose2d(2, 4, new Rotation2d(0));
    private static final Pose2d startingRedPose = new Pose2d(2, 4, new Rotation2d(Math.PI));

	// private static final Pose2d redHubPose = new Pose2d(Inches.of(468.56), Inches.of(158.32), new Rotation2d());
	// private static final Pose2d blueHubPose = new Pose2d(Inches.of(152.56), Inches.of(158.32), new Rotation2d());

	public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6116, 4.0213);
	public static final Translation2d BLUE_HUB_BACK = new Translation2d(5.2342, 4.0213);

	public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9014, 4.0213);
	public static final Translation2d RED_HUB_BACK = new Translation2d(11.3044, 4.0213);

	public static final Translation2d RED_TARGET_LEFT = new Translation2d(16.54048, 5.101844);
	public static final Translation2d RED_TARGET_RIGHT = new Translation2d(16.54048 ,1.266444);
	
	public static final Translation2d BLUE_TARGET_LEFT = new Translation2d(0,5.101844);
	public static final Translation2d BLUE_TARGET_RIGHT = new Translation2d(0,1.266444);

	public static final Distance CENTER_FIELD_Y = Meters.of(4.021328);

	private static Sendable driveSendable;

	private static Transform2d SPITTER_OFFSET = new Transform2d(0, Units.inchesToMeters(-10), Rotation2d.k180deg);

	public boolean hubAlign = false;
	public boolean isAimed = false;

	public double distanceToTarget; 

	private Translation2d currentTarget = BLUE_HUB_CENTER;
	private double targetAngleRot;
	private double currentAngleRot;

	// Standard PID
    private static final PIDConstants TRANSLATION_CONSTANTS =
		new PIDConstants(
			5.5,
			0.2,
			0.1
		);

	private static final PIDConstants ROTATION_CONSTANTS =
		new PIDConstants(
			2.0,
			0.0,
			0.0
		);

    // Pathplanner PID
	private static final PIDConstants PP_TRANS = 
		new PIDConstants(
			5.2,
			0.0,
			0.01
		);

	private static final PIDConstants PP_ROT = 
		new PIDConstants(
			3.0,
			0.0,
			0.0
		);

	/**
	 * Estaablishes PID for X axis
	 */
    private ProfiledPIDController xTranslationPID = new ProfiledPIDController(
        TRANSLATION_CONSTANTS.kP,
        TRANSLATION_CONSTANTS.kI,
        TRANSLATION_CONSTANTS.kD,
		new Constraints(TRANS_MAX_VEL.in(MetersPerSecond), TRANS_MAX_ACCEL.in(MetersPerSecondPerSecond))
    );
	/**
	 * Establishes the PID for the y axis
	 */
	private ProfiledPIDController yTranslationPID = new ProfiledPIDController(
        TRANSLATION_CONSTANTS.kP,
        TRANSLATION_CONSTANTS.kI,
        TRANSLATION_CONSTANTS.kD,
		new Constraints(TRANS_MAX_VEL.in(MetersPerSecond), TRANS_MAX_ACCEL.in(MetersPerSecondPerSecond))
    );

	/**
 	* Establishing PID for the rotational axis
 	*/
    private ProfiledPIDController rotationPID = new ProfiledPIDController(
		ROTATION_CONSTANTS.kP,
		ROTATION_CONSTANTS.kI,
		ROTATION_CONSTANTS.kD,
		new Constraints(ROT_MAX_VEL.in(RadiansPerSecond), ROT_MAX_ACCEL.in(RadiansPerSecondPerSecond))
	);

    /**
	 * Swerve drive object.
	 */
    private SwerveDrive swerveDrive;

    /**
	 * Initialize {@link SwerveDrive}
	 *
	 * @param parser             The parser to create the swerve drive.
	 * @param maxChassisVelocity The maximum chassis velocity of the robot.
	 * @param initialPose        The initial pose of the robot.
	 * @param verbosity          The verbosity level for telemetry.
	 */
	public DriveSubsystem(
		String path,
		TelemetryVerbosity verbosity
	) {
		super();
		
		rotationPID.enableContinuousInput(0, 2 * Math.PI);

		xTranslationPID.setTolerance(TRANS_ERR_TOL.in(Meters), TRANS_VEL_TOL.in(MetersPerSecond));
		yTranslationPID.setTolerance(TRANS_ERR_TOL.in(Meters), TRANS_VEL_TOL.in(MetersPerSecond));
		rotationPID.setTolerance(ROT_ERR_TOL.in(Radians), ROT_VEL_TOL.in(RadiansPerSecond));
		
		SwerveDriveTelemetry.verbosity = verbosity;

		try { 
			swerveDrive = new SwerveParser(
				new File(Filesystem.getDeployDirectory(), path))
				.createSwerveDrive(
					MAX_VELOCITY.in(MetersPerSecond),
					new Pose2d(2, 4, new Rotation2d())// sets the position to the bottom right (paper view)
				);
		} catch (IOException e) {
			e.printStackTrace();
		}

		resetOdometry(new Pose2d(2, 4, swerveDrive.getYaw()));
			
		swerveDrive.setAngularVelocityCompensation(
			true,
			true,
			0.1
		);
		
		swerveDrive.chassisVelocityCorrection = false; //does nothing set to true and test later
		swerveDrive.autonomousChassisVelocityCorrection = false; //does nothing currently set to true and test later
		
		swerveDrive.useExternalFeedbackSensor();

		setupPathPlanner();  
		
		SmartDashboard.putData("XPID", xTranslationPID);
		SmartDashboard.putData("YPID", yTranslationPID);
		SmartDashboard.putData("RPID", rotationPID);  
		
		driveSendable = new Sendable() {
			@Override 
			public void initSendable(SendableBuilder builder) {
				builder.addDoubleProperty("PoseX", () -> getPose().getX(), null);
				builder.addDoubleProperty("PoseY", () -> getPose().getY(), null);
				builder.addBooleanProperty("Align to Hub", () -> hubAlign, null);
				builder.addBooleanProperty("Is Aimed", () -> isAimed, null);
				builder.addDoubleProperty("Distance from hub", () -> distanceToTarget, null);
				builder.addDoubleProperty("TargetX", () -> currentTarget.getX(), null);
				builder.addDoubleProperty("TargetY", () -> currentTarget.getY(), null);
				builder.addDoubleProperty("TargetRot", () -> targetAngleRot, null);
				builder.addDoubleProperty("CurrentRot", () -> currentAngleRot, null);
			}
		};
		SmartDashboard.putData("Drive", driveSendable);


	}

	public void setupPathPlanner() {
		RobotConfig config;

		try {
			config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;

            AutoBuilder.configure(
                () -> swerveDrive.getPose(),
                this::resetOdometry,
                () -> swerveDrive.getRobotVelocity(),
                (speedsRobotRelative, moduleFeedForwards) -> {
					if (enableFeedforward) {
						swerveDrive.drive(
							speedsRobotRelative,
							swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
							moduleFeedForwards.linearForces()
						);
					} else {
						swerveDrive.setChassisSpeeds(speedsRobotRelative);
					}
				},
				new PPHolonomicDriveController(
					PP_TRANS,
					PP_ROT
				),
				config,
				() -> Alliance.getAlliance() == AllianceColor.Red,
				this
            );
			PathfindingCommand.warmupCommand().schedule();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void periodic() {
		updateOdometry();
	}
	
	public void setIMUYaw(Rotation2d yaw) {
		getIMU().setYaw(yaw.getMeasure());
		swerveDrive.resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), yaw));
	}

	public Pigeon2 getIMU() {
		return ((Pigeon2) swerveDrive.getGyro().getIMU());
	}

	public Rotation2d getIMUYaw() {
		return getIMU().getRotation2d();
	}

	public AngularVelocity getIMUYawRate() {
		return getIMU().getAngularVelocityZWorld().getValue();
	}


	/**
   * Updates the drivetrain odometry object to the robot's current position on the
   * field.
   * 
   * @return The new updated pose of the robot.
   */
  public void updateOdometry() {

    for (String side : new String[] {"left", "right"}) {

    	LimelightHelpers.SetRobotOrientation(
			"limelight-" + side, getIMUYaw().getDegrees(), 
			getIMUYawRate().in(DegreesPerSecond), 
			0, 0, 0, 0
		);

		LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-" + side);

		if (estimate != null && estimate.tagCount > 0) {

			Translation3d aprilTagPosition = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-" + side).getTranslation();

			if (Math.hypot(aprilTagPosition.getX(), aprilTagPosition.getZ()) <= 3.5) {
					
				swerveDrive.addVisionMeasurement(new Pose2d(estimate.pose.getX(), estimate.pose.getY(), getIMUYaw()), estimate.timestampSeconds);
					
			}
				
		}

      }

    }


    /**
	 * Resets odometry to the given pose. Gyro angle and module positions do not
	 * need to be reset when calling this
	 * method. However, if either gyro angle or module position is reset, this must
	 * be called in order for odometry to
	 * keep working..
	 * @param initialHolonomicPose The pose to set the odometry to
	 */
	public void resetOdometry(Pose2d pose) {
		if(Alliance.getAlliance() == AllianceColor.Red) {
            if (pose == null) {
                swerveDrive.resetOdometry(startingRedPose);
                return;
            }
        } else {
            if (pose == null) {
				swerveDrive.resetOdometry(startingBluePose);
				return;
			}
        }
        swerveDrive.resetOdometry(pose);
    }

	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
		return run(() -> {
			if(hubAlign) {
				// overrides velocity on the z axis to align to the hub
				Pose2d currentPose = swerveDrive.getPose();
				ChassisSpeeds currentSpeed = swerveDrive.getFieldVelocity();
				
				Translation2d robotTranslation = currentPose.getTranslation();
				Translation2d spitterTranslation = robotTranslation.rotateBy(Rotation2d.k180deg); // our spitter is on the back of the bot
				
				Translation2d botVelocity = 
					new Translation2d(
						currentSpeed.vxMetersPerSecond,
						currentSpeed.vyMetersPerSecond
					);
				
				Translation2d tangentialVelocity = 
					new Translation2d(
						-currentSpeed.omegaRadiansPerSecond * SPITTER_OFFSET.getY(),
						currentSpeed.omegaRadiansPerSecond * SPITTER_OFFSET.getX()
					);
				
				Translation2d effectiveShooterVelocity = botVelocity.plus(tangentialVelocity);
				Translation2d virtualTarget = getTargetTranslation();
				currentTarget = virtualTarget;

				// measures distance to our target in meters
				double predictedDistance = spitterTranslation.getDistance(virtualTarget);

				// time of flight, includes mechanical/system latency
				double timeOfFlight = getToF(predictedDistance) + SpitterSubsystem.SYSTEM_LATENCY_SECONDS;

				// calculate the distance the ball will drift
				Translation2d predictedOffset = effectiveShooterVelocity.times(timeOfFlight);

				// Shifts aim to be ahead of drift
				virtualTarget = virtualTarget.minus(predictedOffset);

				// Set distance for use in other commands
				distanceToTarget = robotTranslation.getDistance(virtualTarget);

				/* Calculate needed angle to target */
				double targetAngleRad = Math.atan2(
					virtualTarget.getY() - robotTranslation.getY(),
					virtualTarget.getX() - robotTranslation.getX()
				);

				targetAngleRad += Math.PI; // trying to get back of bot to face forwards
				
				targetAngleRot = targetAngleRad / (2 * Math.PI);

				double currentAngleRad = currentPose.getRotation().getRadians();

				currentAngleRot = currentAngleRad / (2 * Math.PI);

				double angularSpeedRps = rotationPID.calculate(currentAngleRad, targetAngleRad);
				
				// will finish if the bot is correctly facing target
				isAimed = rotationPID.atSetpoint();	
				
				// overrides the drivers Z input with the calculated angle 
				ChassisSpeeds driverSpeed = velocity.get();
				ChassisSpeeds newVelocity = new ChassisSpeeds(driverSpeed.vxMetersPerSecond, driverSpeed.vyMetersPerSecond, angularSpeedRps);
				
				// running the bot in robot relative with the new calculated angle
				swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(newVelocity, getIMUYaw()));		
			} else {
				distanceToTarget = distanceFromHub();
				swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(velocity.get(), getIMUYaw()));
				// swerveDrive.driveFieldOriented(velocity.get()); //Field relative is relying on odemtry instead of IMUYaw
			}
		}).ignoringDisable(false);
	}

	public double getToF(double distanceMeters) {
		return SpitterSubsystem.SPITTER_MAP.get(distanceMeters).timeOfFlight();
	}

	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by
	 * odometry.
	 *
	 * @return The robot's pose
	 */	
	public Pose2d getPose() {
		if (RobotBase.isSimulation()) {
			return swerveDrive.field.getRobotPose();
		}
		return swerveDrive.getPose();
	}

	public void zeroGyro() {
		getIMU().setYaw(0);
		swerveDrive.resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d()));
	}

	public Command zeroGyroCommand() {
		return runOnce(() -> zeroGyro());
	}

	public void redGyro() {
		getIMU().setYaw(0);
		swerveDrive.resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(Math.PI)));
	}

	public Command zeroWithRedCommand() {
		return runOnce(() -> redGyro());
	}

	/**
	 * Zeros the gyro with the lime light based on 2d april tags
	 */
	public Command zeroGyroWithLimelight() {

		return runOnce(
			() -> {

				Rotation2d mt1_left = VisionSubsystem.getMT1Rotation("left");
				Rotation2d mt1_right = VisionSubsystem.getMT1Rotation("right");

				if (mt1_left != null && mt1_right == null) {
					setIMUYaw(mt1_left);
				} else if (mt1_right != null && mt1_left == null) {
					setIMUYaw(mt1_right);
				} else if (mt1_left != null && mt1_right != null) {
					Rotation2d average = new Rotation2d(
						Math.atan2(
							Math.sin(mt1_left.getRadians()) + Math.sin(mt1_right.getRadians()),
							Math.cos(mt1_left.getRadians()) + Math.cos(mt1_right.getRadians())
						)
					);
					setIMUYaw(average);
				}
				
			}
		);
	}

	public void postTrajectoryToField(List<Pose2d> trajectory) {
		swerveDrive.field.getObject("Trajectory").setPoses(trajectory);
	}

	public void addVisionReading(Pose2d pose, Time timestamp) {
		swerveDrive.addVisionMeasurement(pose, timestamp.in(Milliseconds));
	}

	public SwerveInputStream getInputStream(
		DoubleSupplier x,
		DoubleSupplier y,
		DoubleSupplier rot
	) {
		return new SwerveInputStream(swerveDrive, x, y, rot);
	}

	/**
	 * Use PID control to go to a point on the field.
	 *
	 * @param pose Target {@link Pose2d} to go to.
	 * @return PID command
	 */
	public Command driveToPose(Supplier<Pose2d> pose) {
		return
			runOnce(
				() -> {
					xTranslationPID.reset(getPose().getX(), swerveDrive.getFieldVelocity().vxMetersPerSecond);
					yTranslationPID.reset(getPose().getY(), swerveDrive.getFieldVelocity().vyMetersPerSecond);
					rotationPID.reset(getPose().getRotation().getRadians(), swerveDrive.getFieldVelocity().omegaRadiansPerSecond);
				}
			).andThen(
				driveFieldOriented(
					() -> {
						return new ChassisSpeeds(
							xTranslationPID.calculate(getPose().getX(), pose.get().getX()),
							yTranslationPID.calculate(getPose().getY(), pose.get().getY()),
							rotationPID.calculate(getPose().getRotation().getRadians(), pose.get().getRotation().getRadians())
						);
					}
				)
			)
			.ignoringDisable(false)
			.until(
				() -> xTranslationPID.atGoal() && yTranslationPID.atGoal() && rotationPID.atGoal()
			);
    }
	/**Takes the robots pose and selects a target based on where it is.
	 * Walks through your logic and sees if you are in the center field or the alliance zone then selects a target.
	 * @return Selected target Translation
	 */
	 public Translation2d getTargetTranslation() {
		Pose2d robotPose = getPose();
		if(Alliance.getAlliance().equals(AllianceColor.Red) ) {
		
			if(robotPose.getMeasureX().gt(RED_HUB_CENTER.getMeasureX())) {
				return RED_HUB_CENTER;
			}
			if(robotPose.getMeasureY().gt(CENTER_FIELD_Y)) {
				return RED_TARGET_LEFT;
			}	
			return RED_TARGET_RIGHT;

		}
		
		if(robotPose.getMeasureX().lt(BLUE_HUB_CENTER.getMeasureX())) {
			return BLUE_HUB_CENTER;
		}
		if(robotPose.getMeasureY().gt(CENTER_FIELD_Y)){
			return BLUE_TARGET_LEFT;
		}
		return BLUE_TARGET_RIGHT;
	}

	/*
	* Returns the angle from the robot to the hub (in radians)
	*/
	public double angleFromHub() {
		Translation2d hubPose = getTargetTranslation();
		return Math.atan2(hubPose.getY() - getPose().getY(), hubPose.getX() - getPose().getX());
	}
	
	/*
	* Returns the distance from the robot to the hub 
	*/
	private double distanceFromHub() {
		return getPose().getTranslation().getDistance(getTargetTranslation());
	}
	
	/**
	 * Toggles the bot to align to the hub 
	 */
	public Command toggleHubAlign() {
		return new Command() {
			@Override
			public void initialize() {
				hubAlign = true;
			}

			@Override
			public void end(boolean interrupted) {
				hubAlign = false;
			}
		};
		// return Commands.runOnce(() -> hubAlign = !hubAlign);
	}

}
