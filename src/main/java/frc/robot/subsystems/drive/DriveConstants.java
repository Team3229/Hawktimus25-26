package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;

public class DriveConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(50).withKI(0).withKD(0.32)
        .withKS(0).withKV(0).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(1).withKI(0).withKD(0.1)
        .withKS(0).withKV(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(20))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus();

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 4.5;

    private static final double kDriveGearRatio = 7.03125;
    private static final double kSteerGearRatio = 26.09090909090909;
    private static final Distance kWheelRadius = Inches.of(2);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 4;

    public static final Distance TRANS_ERR_TOL = Meters.of(0.025);
	public static final LinearVelocity TRANS_VEL_TOL = MetersPerSecond.of(0.1);
	public static final Angle ROT_ERR_TOL = Degrees.of(0.5);
	public static final AngularVelocity ROT_VEL_TOL = DegreesPerSecond.of(0.5);

	public static final LinearVelocity TRANS_MAX_VEL = MetersPerSecond.of(3);
	public static final LinearAcceleration TRANS_MAX_ACCEL = MetersPerSecondPerSecond.of(2);

	public static final AngularVelocity ROT_MAX_VEL = DegreesPerSecond.of(540);
	public static final AngularAcceleration ROT_MAX_ACCEL = DegreesPerSecondPerSecond.of(540);

    public static final Pose2d startingBluePose = new Pose2d(2, 4, new Rotation2d(0));
    public static final Pose2d startingRedPose = new Pose2d(2, 4, new Rotation2d(Math.PI));

	public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6116, 4.0213);
	public static final Translation2d BLUE_HUB_BACK = new Translation2d(5.2342, 4.0213);

	public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9014, 4.0213);
	public static final Translation2d RED_HUB_BACK = new Translation2d(11.3044, 4.0213);

	public static final Translation2d RED_TARGET_LEFT = new Translation2d(16.54048, 5.101844);
	public static final Translation2d RED_TARGET_RIGHT = new Translation2d(16.54048 ,1.266444);
	
	public static final Translation2d BLUE_TARGET_LEFT = new Translation2d(0,5.101844);
	public static final Translation2d BLUE_TARGET_RIGHT = new Translation2d(0,1.266444);

	public static final Distance CENTER_FIELD_Y = Meters.of(4.021328);

	public static Transform2d SPITTER_OFFSET = new Transform2d(0, Units.inchesToMeters(-10), Rotation2d.k180deg);

	public boolean hubAlign = false;
	public boolean isAimed = false;

	public double distanceToTarget; 

	public Translation2d currentTarget = BLUE_HUB_CENTER;
	public double targetAngleRot;
	public double currentAngleRot;

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
    public static ProfiledPIDController xTranslationPID = new ProfiledPIDController(
        TRANSLATION_CONSTANTS.kP,
        TRANSLATION_CONSTANTS.kI,
        TRANSLATION_CONSTANTS.kD,
		new Constraints(TRANS_MAX_VEL.in(MetersPerSecond), TRANS_MAX_ACCEL.in(MetersPerSecondPerSecond))
    );
	/**
	 * Establishes the PID for the y axis
	 */
	public static ProfiledPIDController yTranslationPID = new ProfiledPIDController(
        TRANSLATION_CONSTANTS.kP,
        TRANSLATION_CONSTANTS.kI,
        TRANSLATION_CONSTANTS.kD,
		new Constraints(TRANS_MAX_VEL.in(MetersPerSecond), TRANS_MAX_ACCEL.in(MetersPerSecondPerSecond))
    );

	/**
 	* Establishing PID for the rotational axis
 	*/
    public static ProfiledPIDController rotationPID = new ProfiledPIDController(
		ROTATION_CONSTANTS.kP,
		ROTATION_CONSTANTS.kI,
		ROTATION_CONSTANTS.kD,
		new Constraints(ROT_MAX_VEL.in(RadiansPerSecond), ROT_MAX_ACCEL.in(RadiansPerSecondPerSecond))
	);


    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 20;
    private static final int kFrontLeftSteerMotorId = 19;
    private static final int kFrontLeftEncoderId = 18;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.09619140625);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(13.5);
    private static final Distance kFrontLeftYPos = Inches.of(13.5);

    // Front Right
    private static final int kFrontRightDriveMotorId = 3;
    private static final int kFrontRightSteerMotorId = 2;
    private static final int kFrontRightEncoderId = 1;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.488037109375);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(13.5);
    private static final Distance kFrontRightYPos = Inches.of(-13.5);

    // Back Left
    private static final int kBackLeftDriveMotorId = 14;
    private static final int kBackLeftSteerMotorId = 15;
    private static final int kBackLeftEncoderId = 13;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.43505859375);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-13.5);
    private static final Distance kBackLeftYPos = Inches.of(13.5);

    // Back Right
    private static final int kBackRightDriveMotorId = 7;
    private static final int kBackRightSteerMotorId = 8;
    private static final int kBackRightEncoderId = 9;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.208740234375);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-13.5);
    private static final Distance kBackRightYPos = Inches.of(-13.5);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    public static DriveSubsystem createDrivetrain() {
        return new DriveSubsystem(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }


    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}
