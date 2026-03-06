package frc.robot.subsystems.manipSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ManipSubsystem extends SubsystemBase {
    IntakeSubsystem intakeSubsystem;
    IndexSubsystem indexSubsystem;
    SpitterSubsystem spitterSubsystem;
    
    public ManipSubsystem(DriveSubsystem drive) {
        intakeSubsystem = new IntakeSubsystem();
        spitterSubsystem = new SpitterSubsystem(drive);
        indexSubsystem = new IndexSubsystem(spitterSubsystem);
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

    public Command home() {
        return runOnce(() -> System.out.println("Returning to home"))
        .andThen(intakeSubsystem.goHome())
        .andThen(runOnce(() -> System.out.println("At home!")));
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
        return spitterSubsystem.setSpitterSpeed()
        .andThen(indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1))
        .andThen(new ParallelCommandGroup(
            spitterSubsystem.shoot(),
            indexSubsystem.index(indexSubsystem.forwards)
        ));
        // intakeSubsystem.agitateFuel();

        //     Command out = new Command() {
        //         @Override
        //         public void initialize() {
        //         }
                
        //         @Override 
        //         public void execute() {
        //         System.out.println("is executing");

        //         if(shooterIsReady()) {
        //             System.out.println("shooter is ready and index should start");
        //         }
        //     }
        // };
        // out.addRequirements(this);
        // return out;
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
        return indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1)
        .andThen(new ParallelCommandGroup(
            spitterSubsystem.lowPass(),
            indexSubsystem.index(indexSubsystem.forwards)
        ));
    }

    public Command midPass() {
     return indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1)
        .andThen(new ParallelCommandGroup(
            spitterSubsystem.midPass(),
            indexSubsystem.index(indexSubsystem.forwards)
        ));
    }

    public Command highPass() {
     return indexSubsystem.index(indexSubsystem.reverse).withTimeout(0.1)
        .andThen(new ParallelCommandGroup(
            spitterSubsystem.highPass(),
            indexSubsystem.index(indexSubsystem.forwards)
        ));
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

    public Command setSpitterSpeed() {
        return spitterSubsystem.setSpitterSpeed();
    }
}