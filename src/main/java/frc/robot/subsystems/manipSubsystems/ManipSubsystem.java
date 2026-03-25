package frc.robot.subsystems.manipSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ManipSubsystem extends SubsystemBase {
    IntakeSubsystem intakeSubsystem;
    IndexSubsystem indexSubsystem;
    SpitterSubsystem spitterSubsystem;
    DriveSubsystem driveSubsystem;
    
    public ManipSubsystem(DriveSubsystem drive) {
        intakeSubsystem = new IntakeSubsystem();
        spitterSubsystem = new SpitterSubsystem(drive);
        indexSubsystem = new IndexSubsystem(spitterSubsystem);
        driveSubsystem = drive;
    }

    /**
     * runs intake roller
     */
    public Command intake() {
        return runOnce(() -> System.out.println("Intaking..."))
        .andThen(new ParallelCommandGroup(
            intakeSubsystem.intake(),
            new ConditionalCommand(indexSubsystem.jitterIndex(), Commands.none(), () -> !spitterSubsystem.shooterIsReady())
        ));
    }

    /**
     * moves intake arm to be vertical
     */
    public Command stow() {
        return runOnce(() -> System.out.println("Beginning to stow"))
        .andThen(intakeSubsystem.stow())
        .andThen(runOnce(() -> System.out.println("Fully stowed")));
    }

    /**
     * moves intake arm to be at the home limit
     */
    public Command home() {
        return runOnce(() -> System.out.println("Returning to home"))
        .andThen(intakeSubsystem.goHome())
        .andThen(runOnce(() -> System.out.println("At home!")));
    }

    /**
     * moves the intake arm out
     */
    public Command intakeArmOut() {
        return runOnce(() -> System.out.println("Beginning the extention"))
        .andThen(intakeSubsystem.extendIntake())
        .andThen(runOnce(() -> System.out.println("Fully extended")));
    }

    /**
     * Spins the shooter wheel
     */
    public Command spinUp() {
        return spitterSubsystem.shoot();
    }

    /**
     * 
     * runs intake and fires the balls
     */
    public Command shoot() {
        //AUTOSHOOT
        return indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1)
        .andThen(new ParallelCommandGroup(
            // intakeSubsystem.agitateFuel(),
            spitterSubsystem.shoot(),
            indexSubsystem.index(indexSubsystem.forwards)
        ));

        // //MANUAL SHOOT
        // return indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1)
        // .andThen(new ParallelCommandGroup(
        //     // intakeSubsystem.agitateFuel(),
        //     spitterSubsystem.shoot(),
        //     indexSubsystem.index(indexSubsystem.forwards)
        // ));
    }
    
    /** 
     * reverses the intake to blast out balls from intake
     */
    public Command extake() {
        return new ParallelCommandGroup(
            indexSubsystem.index(indexSubsystem.reverse),
            intakeSubsystem.extake()
        );
    }

    public boolean shooterIsReady() {
        return spitterSubsystem.shooterIsReady();
    }

    public Command upSRPSCommand() {
        return spitterSubsystem.upSRPSCommand();
    }

    public Command downSRPSCommand() {
        return spitterSubsystem.downSRPSCommand();
    }

    public Command upFRPSCommand() {
        return spitterSubsystem.upFRPSCommand();
    }

    public Command downFRPSCommand() {
        return spitterSubsystem.downFRPSCommand();
    }

    /**
	 * Toggles the bot to spin the intake while stowed 
	 */
	public Command toggleStowSpin() {
		return new Command() {
			@Override
			public void initialize() {
				IntakeSubsystem.stowSpin = true;
			}

			@Override
			public void end(boolean interrupted) {
				IntakeSubsystem.stowSpin = false;
			}
		};
	} // TODO: just a test for next time, as it wasn't turning off before
}