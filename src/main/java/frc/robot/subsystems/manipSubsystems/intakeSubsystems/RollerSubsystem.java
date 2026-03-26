package frc.robot.subsystems.manipSubsystems.intakeSubsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {

	private static TalonFX rodMotor;

	private static TalonFXConfiguration rodMotorConfig;

	private static final int ROD_CAN_ID = 17; 
	
	private static final Current CURRENT_LIMIT = Amps.of(40);
	
	private double requestedVelocity;

	private double rP = 0.1; 
	private double rI = 0.0; 
	private double rD = 0.0; 
	private double rV = 0.13; 
	
	public static final double ROD_CW_SPEED = 70; 
	public static final double ROD_CCW_SPEED = -70;

	public static boolean stowSpin = false;

	private static Sendable intakeRodSendable;
	private static Sendable intakeRodPIDSendable;
	
	public RollerSubsystem() {
        super();
		
		rodMotor = new TalonFX(ROD_CAN_ID, CANBus.roboRIO());
		rodMotorConfig = new TalonFXConfiguration()
		    .withMotorOutput(
			    new MotorOutputConfigs()
				    .withNeutralMode(NeutralModeValue.Coast)
		    )
		    .withCurrentLimits(
			    new CurrentLimitsConfigs()
				    .withStatorCurrentLimit(CURRENT_LIMIT)
				    .withStatorCurrentLimitEnable(true)
		    )
		    .withVoltage(
		    	new VoltageConfigs()
		    		.withPeakForwardVoltage(12)
		    		.withPeakReverseVoltage(-12)
		    		.withSupplyVoltageTimeConstant(0)
		    );

		rodMotorConfig.Slot0.kP = rP;
		rodMotorConfig.Slot0.kI = rI;
		rodMotorConfig.Slot0.kD = rD;
		rodMotorConfig.Slot0.kV = rV;

		rodMotor.getConfigurator().apply(rodMotorConfig);

		intakeRodSendable = new Sendable() {
		@Override 
		public void initSendable(SendableBuilder builder) {
			builder.addBooleanProperty("Intaking", () -> rodIsReady(), null);
			builder.addDoubleProperty("VelocityRod", () -> rodMotor.getVelocity().getValueAsDouble(), null);
			builder.addBooleanProperty("Intaking", ()-> rodIsReady(), null); 
			}
		};

		SmartDashboard.putData("Intake", intakeRodSendable);

		intakeRodPIDSendable = new Sendable() {
		@Override
			public void initSendable(SendableBuilder builder) {
				builder.addDoubleProperty("Rod P", () -> rP, (newrP) -> editRodP(newrP));
				builder.addDoubleProperty("Rod I", () -> rI, (newrI) -> editRodI(newrI));
				builder.addDoubleProperty("Rod D", () -> rD, (newrD) -> editRodD(newrD));
				builder.addDoubleProperty("Rod V", () -> rV, (newrV) -> editRodV(newrV));
			}
		};
		SmartDashboard.putData("IntakePID", intakeRodPIDSendable);
	}

	private void editRodP(double newrP) {
        rP = newrP;
        rodMotorConfig.Slot0.kP = rP;

        rodMotor.getConfigurator().apply(rodMotorConfig);
    }

	private void editRodI(double newrI) {
        rI = newrI;
        rodMotorConfig.Slot0.kI = rI;

        rodMotor.getConfigurator().apply(rodMotorConfig);
    }

	private void editRodD(double newrD) {
        rD = newrD;
        rodMotorConfig.Slot0.kP = rD;

        rodMotor.getConfigurator().apply(rodMotorConfig);
    }

	private void editRodV(double newrV) {
        rV = newrV;
        rodMotorConfig.Slot0.kV = rV;

        rodMotor.getConfigurator().apply(rodMotorConfig);
    }
	
	public Command rodSpin(double speedSetpoint) {
		Command out = new Command() {
			@Override 
			public void initialize() {
				requestedVelocity = speedSetpoint;
			}
			@Override
			public void execute() {
				rodMotor.setControl(new VelocityVoltage(speedSetpoint).withSlot(0));

			}
			@Override
			public void end(boolean interrupted) {
				rodMotor.setControl(new CoastOut());
			}
		};
		out.addRequirements(this);
		return out;
	}

	private boolean rodIsReady() {
		double deadBand = 1;
		double rodVelocity = rodMotor.getVelocity().getValueAsDouble();
		if(requestedVelocity == 0) {
			return false;
		} else {
		  return Math.abs(requestedVelocity - rodVelocity) <= deadBand;
		}
	}
}