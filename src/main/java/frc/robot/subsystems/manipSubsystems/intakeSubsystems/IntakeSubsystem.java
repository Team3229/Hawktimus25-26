package frc.robot.subsystems.manipSubsystems.intakeSubsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.subsystems.manipSubsystems.intakeSubsystems.RollerSubsystem;
// import frc.robot.subsystems.manipSubsystems.intakeSubsystems.ArmSubsystem;

public class IntakeSubsystem extends SubsystemBase{
    RollerSubsystem rollerSubsystem;
    ArmSubsystem armSubsystem;

    public static final double ROD_CW_SPEED = RollerSubsystem.ROD_CW_SPEED; 
	public static final double ROD_CCW_SPEED = RollerSubsystem.ROD_CCW_SPEED;

    public static final Angle HOME_ANGLE = ArmSubsystem.HOME_ANGLE;
	public static final Angle STOW_ANGLE = ArmSubsystem.STOW_ANGLE;
	public static final Angle COLLECTION_POINT = ArmSubsystem.COLLECTION_POINT;
	public static final Angle COLLECTION_POINT2 = ArmSubsystem.COLLECTION_POINT2;

    public IntakeSubsystem() {
		rollerSubsystem = new RollerSubsystem();
        armSubsystem = new ArmSubsystem(rollerSubsystem);
    }

    /**
	* creates a command to collect the fuel by spinning the rod
	* 
	* @return Command to spin rod
	*/
    public Command intake() {
        return rollerSubsystem.rodSpin(ROD_CW_SPEED);
    }

    /**
	 * creates a command to reverse the intake to put the fuel into the human player
	 * station
	 * 
	 * @return Command to spin the rod in reverse
	 */ 
    public Command extake() {
        return rollerSubsystem.rodSpin(ROD_CCW_SPEED);
    }

    /**
	 * Command to return to a safe angle 
     * 
     * @return Command to rotate the arm to stowing position
	 */
	public Command stow() {
		return armSubsystem.rotateTo(STOW_ANGLE);
	}

    /**
	 * Command to return to a initial angle 
     * 
     * @return Command to rotate the arm to the FIRST angle
	 */
    public Command goHome() {
		return armSubsystem.rotateTo(HOME_ANGLE);
	}

    /**
	 * creates a command that pushes the intake arm down to the collection point
	 * and pushes the storage area out.
	 * 
	 * @return Command to rotate the arm
	 */
	public Command extendIntake() {
		return armSubsystem.rotateTo(COLLECTION_POINT2);
	}

}
