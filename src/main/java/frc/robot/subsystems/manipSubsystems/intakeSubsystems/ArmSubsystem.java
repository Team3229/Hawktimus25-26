package frc.robot.subsystems.manipSubsystems.intakeSubsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ArmSubsystem extends SubsystemBase {

	RollerSubsystem rollerSubsystem;

	private static TalonFX armMotorRight;
	private static TalonFX armMotorLeft;
	private static TalonFXConfiguration armMotorConfig;

	private static MotionMagicVoltage rotateRequest;

	private static double motionMagicCruiseVelocity = 0.5;
	private static double motionMagicAcceleration = 1;
	private static double motionMagicJerk = 10;

	private static DigitalInput homeLimitSwitch;
	private static DigitalInput extendLimitSwitch;

	private static GravityTypeValue gravityTypeValue = GravityTypeValue.Arm_Cosine;
	
	private static final int ARM_R_CAN_ID = 16; 
	private static final int ARM_L_CAN_ID = 5; 
	
	private static final int HOME_LIMIT_PORT = 0; 
	private static final int EXTEND_LIMIT_PORT = 1;

	public static final Angle HOME_ANGLE = Rotations.of(0.139);
	public static final Angle STOW_ANGLE = Rotations.of(0.368);
	public static final Angle COLLECTION_POINT = Rotations.of(0.5);

	private static final Angle angleDeadBand = Rotations.of(0.0055555555555556);
	
	private static final Current CURRENT_LIMIT = Amps.of(40);
	
	private static double sensorToMechanismRatio = 30.5555555555555555555555555;
	
	private Angle requestedAngle = Rotations.of(0);
	
	private double aP = 0.5;
	private double aI = 0.0; 
	private double aD = 1; 
	private double aV = 2;
	private double aA = 0.0;
	private double aS = 0.4;
	private double aG = 0.5; 

	public static boolean stowSpin = false;

	public static final double ROD_CW_SPEED = RollerSubsystem.ROD_CW_SPEED;

	private static Sendable intakeSendable;
	private static Sendable armPIDSendable;
	
	public ArmSubsystem(RollerSubsystem roll) {

		super();

		rollerSubsystem = roll;

		homeLimitSwitch = new DigitalInput(HOME_LIMIT_PORT);
		
		extendLimitSwitch = new DigitalInput(EXTEND_LIMIT_PORT);
		
		rotateRequest = new MotionMagicVoltage(0).withSlot(0);
		
		armMotorLeft = new TalonFX(ARM_L_CAN_ID, CANBus.roboRIO());
		
		armMotorConfig = new TalonFXConfiguration()
		.withMotorOutput(
			new MotorOutputConfigs()
			.withNeutralMode(NeutralModeValue.Brake)
		)
		.withCurrentLimits(
			new CurrentLimitsConfigs()
			.withStatorCurrentLimit(CURRENT_LIMIT)
			.withStatorCurrentLimitEnable(true)
		)
		.withFeedback(
			new FeedbackConfigs()
			.withSensorToMechanismRatio(sensorToMechanismRatio)
		)
		.withVoltage(
			new VoltageConfigs()
				.withPeakForwardVoltage(12)
				.withPeakReverseVoltage(-12)
				.withSupplyVoltageTimeConstant(0)
		);
			
		armMotorConfig.Slot0.GravityType = gravityTypeValue;
						
		armMotorConfig.Slot0.kP = aP;
		armMotorConfig.Slot0.kI = aI;
		armMotorConfig.Slot0.kD = aD;
		armMotorConfig.Slot0.kV = aV;
		armMotorConfig.Slot0.kA = aA;
		armMotorConfig.Slot0.kS = aS;
		armMotorConfig.Slot0.kG = aG;
		armMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		
		armMotorConfig.MotionMagic
			.withMotionMagicCruiseVelocity(motionMagicCruiseVelocity)
			.withMotionMagicAcceleration(motionMagicAcceleration)
			.withMotionMagicJerk(motionMagicJerk);
		
		armMotorLeft.getConfigurator().apply(armMotorConfig);

		armMotorRight = new TalonFX(ARM_R_CAN_ID, CANBus.roboRIO());
		armMotorRight.getConfigurator().apply(armMotorConfig);
		armMotorRight.setControl(new Follower(ARM_L_CAN_ID, MotorAlignmentValue.Opposed));

		extendLimit().onTrue(
			Commands.runOnce(() -> {
				setExtended();
			})
		);

		homeLimit().onTrue(
			Commands.runOnce(() -> {
				System.out.println("Home limit hit, stopping motors and reseting position");
				setHome();
			})
		);

		setHome();

		intakeSendable = new Sendable() {
			@Override 
			public void initSendable(SendableBuilder builder) {
				builder.addDoubleProperty("PositionL", () -> armMotorLeft.getPosition().getValueAsDouble(), null);
				builder.addDoubleProperty("PositionR", () -> armMotorRight.getPosition().getValueAsDouble(), null);
				builder.addDoubleProperty("VelocityL", () -> armMotorLeft.getVelocity().getValueAsDouble(), null);
				builder.addDoubleProperty("VelocityR", () -> armMotorRight.getVelocity().getValueAsDouble(), null);
				builder.addDoubleProperty("VoltageL", ()-> armMotorLeft.getMotorVoltage().getValueAsDouble(), null); 
				builder.addDoubleProperty("VoltageR", ()-> armMotorRight.getMotorVoltage().getValueAsDouble(), null); 
				builder.addBooleanProperty("Arm is ready", () -> armIsReady(), null);
				builder.addDoubleProperty("Setpoint", () -> requestedAngle.magnitude(), null);
			}
		};
		SmartDashboard.putData("IntakeArm", intakeSendable);

		armPIDSendable = new Sendable() {
		@Override
			public void initSendable(SendableBuilder builder) {
				builder.addDoubleProperty("Arm P", () -> aP, (newaP) -> editArmP(newaP));
				builder.addDoubleProperty("Arm I", () -> aI, (newaI) -> editArmI(newaI));
				builder.addDoubleProperty("Arm D", () -> aD, (newaD) -> editArmD(newaD));
				builder.addDoubleProperty("Arm V", () -> aV, (newaV) -> editArmV(newaV));
				builder.addDoubleProperty("Arm A", () -> aA, (newaA) -> editArmA(newaA));
				builder.addDoubleProperty("Arm S", () -> aS, (newaS) -> editArmS(newaS));
				builder.addDoubleProperty("Arm G", () -> aG, (newaG) -> editArmG(newaG));
			}
		};
		SmartDashboard.putData("ArmPID", armPIDSendable);

	}

	private void editArmP(double newaP) {
        aP = newaP;
        armMotorConfig.Slot0.kP = aP;

        armMotorLeft.getConfigurator().apply(armMotorConfig);
		armMotorRight.getConfigurator().apply(armMotorConfig);
    }

	private void editArmI(double newaI) {
        aI = newaI;
        armMotorConfig.Slot0.kI = aI;
		armMotorConfig.Slot0.kI = aI;

        armMotorLeft.getConfigurator().apply(armMotorConfig);
		armMotorRight.getConfigurator().apply(armMotorConfig);
    }

	private void editArmD(double newaD) {
        aD = newaD;
        armMotorConfig.Slot0.kD = aD;

        armMotorLeft.getConfigurator().apply(armMotorConfig);
		armMotorRight.getConfigurator().apply(armMotorConfig);
    }

	private void editArmV(double newaV) {
        aV = newaV;
        armMotorConfig.Slot0.kV = aV;

        armMotorLeft.getConfigurator().apply(armMotorConfig);
		armMotorRight.getConfigurator().apply(armMotorConfig);
    }
	
	private void editArmA(double newaA) {
        aA = newaA;
        armMotorConfig.Slot0.kA = aA;

        armMotorLeft.getConfigurator().apply(armMotorConfig);
		armMotorRight.getConfigurator().apply(armMotorConfig);
    }

	private void editArmS(double newaS) {
        aS = newaS;
        armMotorConfig.Slot0.kS = aS;

        armMotorLeft.getConfigurator().apply(armMotorConfig);
		armMotorRight.getConfigurator().apply(armMotorConfig);
    }

	private void editArmG(double newaG) {
        aG = newaG;
        armMotorConfig.Slot0.kG = aG;

        armMotorLeft.getConfigurator().apply(armMotorConfig);
		armMotorRight.getConfigurator().apply(armMotorConfig);
    }
	
	public Command rotateTo(Angle setpoint) {
		Command out = new Command() {
			@Override
			public void initialize() {
				armMotorLeft.setControl(rotateRequest.withPosition(armMotorLeft.getPosition().getValue()));
				requestedAngle = setpoint;
			}

			@Override
			public void execute() {
				armMotorLeft.setControl(rotateRequest.withPosition(setpoint));
				if(stowSpin) {
					rollerSubsystem.rodSpin(ROD_CW_SPEED);
				}
			}

			@Override
			public void end(boolean interrupted) {
				armMotorLeft.setControl(new StaticBrake());
			}

		};

		out.addRequirements(this);
		return out;
	
	}

	public Command toCollection() {
		Command out = new Command() {
			@Override
			public void initialize() {
			}

			@Override
			public void execute() {
				armMotorLeft.setControl(new VelocityVoltage(0.25));
			}

			@Override
			public void end(boolean interrupted) {
				armMotorLeft.setControl(new StaticBrake());
			}

		};

		out.addRequirements(this);
		return out;
	}

	/** returns true when arm is within deadband */
	private boolean armIsReady() {
		if (requestedAngle == null) {
			return false;
		}

		Angle ArmAngleAverage = (armMotorLeft.getPosition().getValue().plus(armMotorRight.getPosition().getValue())).div(2);

		// System.out.println("req: " + requestedAngle.toShortString() + " Rcur: " + rightArmAngle.toShortString() + " Lcur: " + leftArmAngle.toShortString());
		// if (ArmAngleAverage.isNear(requestedAngle, angleDeadBand)) {
		// 	armMotorLeft.setControl(new StaticBrake());
		// } 
		return ArmAngleAverage.isNear(requestedAngle, angleDeadBand);
	}

	private boolean homeLimitSwitch() {
		return homeLimitSwitch.get();
	}

	public Trigger homeLimit() {
		return new Trigger(this::homeLimitSwitch);
	}

	public void setHome() {
		armMotorLeft.set(0);
		armMotorLeft.setPosition(HOME_ANGLE);
		armMotorRight.setPosition(HOME_ANGLE);
		System.out.println("Home has been set");
	}

	private boolean extendLimitSwitch() {
		return extendLimitSwitch.get();
	}

	public Trigger extendLimit() {
		return new Trigger(this::extendLimitSwitch);
	}

	public void setExtended() {
		armMotorLeft.set(0);
		armMotorLeft.setPosition(COLLECTION_POINT);
		armMotorRight.setPosition(COLLECTION_POINT);
		CommandScheduler.getInstance().cancel(rotateTo(COLLECTION_POINT));
		CommandScheduler.getInstance().cancel(rotateTo(STOW_ANGLE));
		CommandScheduler.getInstance().cancel(toCollection());
		System.out.println("Extend limit hit, stopping motors and reseting position");
	}

}