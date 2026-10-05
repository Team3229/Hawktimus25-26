package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Gladiator {
    
    private CommandGenericHID controller;

    // private static final double kControllerDeadzone = 0.1;

    public Gladiator(int port) {
        controller = new CommandGenericHID(port);
    }
    public double a_X() {return (controller.getRawAxis(0));}
    public double a_Y() {return (controller.getRawAxis(1));}
    public double a_Z() {return (-controller.getRawAxis(5));}

    
    // public double a_X() {return applyDeadzone(controller.getRawAxis(0));}
    // public double a_Y() {return applyDeadzone(controller.getRawAxis(1));}
    // public double a_Z() {return applyDeadzone(controller.getRawAxis(5));}
    public double a_Throttle() {return controller.getRawAxis(2);}
    public double a_Switch() {return controller.getRawAxis(3);}
    public double a_Dial() {return controller.getRawAxis(4);}

    public Trigger b_Trigger() {return controller.button(1);}
    public Trigger b_FullTrigger() {return controller.button(2).and(controller.button(1));}
    public Trigger b_A2() {return controller.button(3);}
    public Trigger b_B1() {return controller.button(4);}
    public Trigger b_D1() {return controller.button(5);}
    public Trigger b_A3Up() {return controller.button(6);}
    public Trigger b_A3Right() {return controller.button(7);}
    public Trigger b_A3Down() {return controller.button(8);}
    public Trigger b_A3Left() {return controller.button(9);}
    public Trigger b_A3Pushed() {return controller.button(10);}
    public Trigger b_A4Up() {return controller.button(11);}
    public Trigger b_A4Right() {return controller.button(12);}
    public Trigger b_A4Down() {return controller.button(13);}
    public Trigger b_A4Left() {return controller.button(14);}
    public Trigger b_A4Pushed() {return controller.button(15);}
    public Trigger b_C1Up() {return controller.button(16);}
    public Trigger b_C1Right() {return controller.button(17);}
    public Trigger b_C1Down() {return controller.button(18);}
    public Trigger b_C1Left() {return controller.button(19);}
    public Trigger b_C1Pushed() {return controller.button(20);}
    public Trigger b_TopTriggerDown() {return controller.button(22);}
    public Trigger b_TopTriggerUp() {return controller.button(21);}

    public Trigger p_A1_up() {return controller.povUp();}
    public Trigger p_A1_down() {return controller.povDown();}
    public Trigger p_A1_left() {return controller.povLeft();}
    public Trigger p_A1_right() {return controller.povRight();}
    public Trigger p_A1_any() {return controller.povCenter().negate();}


    // private static double applyDeadzone(double input) {
    //     return (Math.abs(input) > kControllerDeadzone) ? 
    //     input
    //     : 0;
    // }
}
