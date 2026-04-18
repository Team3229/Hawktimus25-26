
package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Gladiator {
    
    private CommandGenericHID controller;

    private static final double kControllerDeadzone = 0.1;

    public FlightStick(int port) {
        controller = new CommandGenericHID(port);
    }

    public Trigger b_Trigger() {return controller.button(1);}
    public Trigger b_FullTrigger() {return controller.button(2);}
    public Trigger b_3() {return controller.button(3);}

}
