package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ManipSubsystem extends SubsystemBase {
    IntakeSubsystem intakeSubsystem;
    IndexSubsystem indexSubsystem;
    ShooterArmSubsystem shooterArmSubsystem;
    SpitterSubsystem spitterSubsystem;

    public ManipSubsystem() {
        intakeSubsystem = new IntakeSubsystem();
        indexSubsystem = new IndexSubsystem();
        shooterArmSubsystem = new ShooterArmSubsystem();
        spitterSubsystem = new SpitterSubsystem();
    }
    /**Command that runs intaking
     * 
     * @return Prints Intaking then it intakes and waits 2 seconds
     */
    public Command intake() {
        return runOnce(() -> System.out.println("Intaking..."))
        .andThen(intakeSubsystem.intake())
        .andThen(Commands.waitTime(Seconds.of(2)))
        .andThen(indexSubsystem.index(indexSubsystem.forwards)); // might not be needed
    }

    public Command stow() {
        return runOnce(() -> System.out.println("SSTOOOOOOOOWWWWWWWWWWWWWW"))
        .andThen(intakeSubsystem.rotateTo(IntakeSubsystem.HOME_ANGLE));

    }
    /**Extends storage
     * 
     * @return Runs the extend storage command
     */
    public Command extendStorage() {
        return runOnce(() -> intakeSubsystem.extendIntake());
    }
    /**Spins the shooter wheel
     * 
     * @return Runs the Spit command from
     */
    public Command spinUp() {
        return spitterSubsystem.spinUp(25 /*placeholder will be replaced with distance*/);
    }
    /**moves the fuel forward and then takes it into the shooter
     * 
     * @return moves the index and then runs the intake motor on the shooter
     */
    public Command shoot() {
        return(spitterSubsystem.shoot(25, 25))
        .andThen(indexSubsystem.index(indexSubsystem.forwards))
        .andThen(intakeSubsystem.agitateFuel());
    }
    /** reverses the intake to blast out balls from intake
     * 
     * @return Runs the extake command
     */
    public Command extake() {
        return runOnce(() -> intakeSubsystem.extake())
        .andThen(indexSubsystem.index(indexSubsystem.reverse));
    }
    //TODO: Find and change double value later
   
    
    public Command lowPass() {
        return(spitterSubsystem.lowPass())
        .andThen(indexSubsystem.index(indexSubsystem.forwards));
    }

    public Command midPass() {
        return(spitterSubsystem.midPass())
        .andThen(indexSubsystem.index(indexSubsystem.forwards));
    }

    public Command highPass() {
        return(spitterSubsystem.highPass())
        .andThen(indexSubsystem.index(indexSubsystem.forwards));
    }
}