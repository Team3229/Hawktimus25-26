package frc.robot.subsystems.manipSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipSubsystem extends SubsystemBase {
    IntakeSubsystem intakeSubsystem;
    IndexSubsystem indexSubsystem;
    SpitterSubsystem spitterSubsystem;
    
    public ManipSubsystem() {
        intakeSubsystem = new IntakeSubsystem();
        indexSubsystem = new IndexSubsystem();
        spitterSubsystem = new SpitterSubsystem();
    }
    /**Command that runs intaking
     * 
     * @return Prints Intaking then it intakes and waits 2 seconds
     */
    public Command intake() {
        return runOnce(() -> System.out.println("Intaking..."))
        .andThen(intakeSubsystem.intake());
    }

    public Command stow() {
        return runOnce(() -> System.out.println("SSTOOOOOOOOWWWWWWWWWWWWWW"))
        .andThen(intakeSubsystem.emergencyStow())
        .andThen(runOnce(() -> System.out.println("WE ARE DONE WITH STOWING")));
    }

    /**Extends storage
     * 
     * @return Runs the extend storage command
     */
    public Command extendStorage() {
        return runOnce(() -> System.out.println("WE HAVE BEGUN THE PROCESS OF EXTENDING"))
        .andThen(intakeSubsystem.extendIntake())
        .andThen(runOnce(() -> System.out.println("WE ARE SO DONE WITH EXTENDING")));
    }

    /**Spins the shooter wheel
     * 
     * @return Runs the Spit command from
     */
    public Command spinUp() {
        return spitterSubsystem.shoot();
    }

    /**moves the fuel forward and then takes it into the shooter
     * 
     * @return moves the index and then runs the intake motor on the shooter
     */
    public Command shoot() {
        return(spitterSubsystem.shoot())
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
   
    public Command lowPass() {
        return(new ParallelCommandGroup(
            spitterSubsystem.lowPass(),
            indexSubsystem.index(indexSubsystem.forwards)
        ));
        // .andThen(intakeSubsystem.agitateFuel());
    }

    public Command midPass() {
        return(spitterSubsystem.midPass())
        .andThen(indexSubsystem.index(indexSubsystem.forwards));
        // .andThen(intakeSubsystem.agitateFuel());
    }

    public Command highPass() {
        return(spitterSubsystem.highPass())
        .andThen(indexSubsystem.index(indexSubsystem.forwards));
        // .andThen(intakeSubsystem.agitateFuel());
    }
}