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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.hawklibraries.utilities.Alliance;
import frc.hawklibraries.utilities.Alliance.AllianceColor;
import frc.robot.subsystems.VisionSubsystem;
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

	private static Sendable driveSendable;

	private static boolean slowDrive = false;

	// Standard PID
    private static final PIDConstants TRANSLATION_CONSTANTS =
		new PIDConstants(
			5.5,
			0.2,
			0.1
		);

	private static final PIDConstants ROTATION_CONSTANTS =
		new PIDConstants(
			7.0,
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
				builder.addBooleanProperty("SlowToggle", () -> slowDrive, null);
				builder.addDoubleProperty("Distance from hub", () -> distanceFromHub(), null);
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
			
			@SuppressWarnings("unused")
			double tagID = LimelightHelpers.getFiducialID("limelight-" + side);

				Translation3d aprilTagPosition = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-" + side).getTranslation();

				if (Math.hypot(aprilTagPosition.getX(), aprilTagPosition.getZ()) <= 10.5) {
					
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
			// swerveDrive.driveFieldOriented(velocity.get()); //Field relative is relying on odemtry instead of IMUYaw
			swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(velocity.get(), getIMUYaw()));
		}).ignoringDisable(false);
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
	/**
	 * This will zero (calibrate) the robot to assume the current position is facing
	 * forward
	 * <p>
	 * If red alliance rotate the robot 180 after the drivebase zero command 
	 * this command is useless/actively harmful
	 */
	// public void zeroGyroWithAlliance() {
	// 	if (Alliance.getAlliance() == AllianceColor.Red) {
	// 		setIMUYaw(new Rotation2d(Math.PI));
	// 	} else {
	// 		zeroGyro();
	// 	}
	// }

	public void zeroGyro() {
		getIMU().setYaw(0);
		swerveDrive.resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d()));
	}

	public Command zeroGyroCommand() {
		return runOnce(() -> zeroGyro());
	}

	// public Command zeroGyroWithAllianceCommand() {
	// 	return runOnce(
	// 		this::zeroGyroWithAlliance
	// 	);
	// }

	/**
	 * Zeros the gyro with the lime light based on 2d april tags
	 */
	public Command zeroGyroWithLimelight() {

		return runOnce(
			() -> {

				Rotation2d mt1_left = VisionSubsystem.getMT1Rotation("left");

				if (mt1_left != null) {
					setIMUYaw(mt1_left);
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

	public void slowToggle() {
		if(slowDrive == true) {
			slowDrive = false;
			MAX_VELOCITY.times(2);
		} else {
			slowDrive = true;
			MAX_VELOCITY.div(2);
		}
	}

	public Command slowToggleCommand() {
		return runOnce(() -> slowToggle());
	}
	

	public Translation2d getHubTranslation() {
		return Alliance.getAlliance().equals(AllianceColor.Red) ? RED_HUB_CENTER : BLUE_HUB_CENTER;
	}

	/*
	* Returns the angle from the robot to the hub (in radians)
	*/
	public double angleFromHub() {
		Translation2d hubPose = getHubTranslation();
		return Math.atan2(hubPose.getY() - getPose().getY(), hubPose.getX() - getPose().getX());
	}
	
	/*
	* Returns the distance from the robot to the hub 
	*/
	public double distanceFromHub() {
		return getPose().getTranslation().getDistance(getHubTranslation());
	}
	
	/**Rotates the bot to be facing the hub*/
    public Command alignToHub() {
		Command out = new Command() {
			@Override
			public void execute() {
				driveToPose(() -> {
					return new Pose2d(
						getPose().getX(),
						getPose().getY(),
						new Rotation2d(angleFromHub())
					);
				});
			};
		};

		return out;
		
	};

}
