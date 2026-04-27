
package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Gladiator {
    
    private CommandGenericHID controller;

    private static final double kControllerDeadzone = 0.1;

    public Gladiator(int port) {
        controller = new CommandGenericHID(port);
    }

    public double a_X() {return applyDeadzone(controller.getRawAxis(0));}
    public double a_Y() {return applyDeadzone(controller.getRawAxis(1));}
    public double a_Z() {return applyDeadzone(controller.getRawAxis(2));}
    public double a_Throttle() {return controller.getRawAxis(3);}
    public double a_Switch() {return controller.getRawAxis(4);}
    public double a_Dial() {return controller.getRawAxis(5);}

    public Trigger b_Trigger() {return controller.button(1);}
    public Trigger b_FullTrigger() {return controller.button(2).and(controller.button(1));}
    public Trigger b_3() {return controller.button(3);}
    public Trigger b_4() {return controller.button(4);}
    public Trigger b_5() {return controller.button(5);}
    public Trigger b_POV0Up() {return controller.button(6);}
    public Trigger b_POV0Right() {return controller.button(7);}
    public Trigger b_POV0Down() {return controller.button(8);}
    public Trigger b_POV0Left() {return controller.button(9);}
    public Trigger b_POV0Pushed() {return controller.button(10);}
    public Trigger b_POV1Up() {return controller.button(11);}
    public Trigger b_POV1Right() {return controller.button(12);}
    public Trigger b_POV1Down() {return controller.button(13);}
    public Trigger b_POV1Left() {return controller.button(14);}
    public Trigger b_POV1Pushed() {return controller.button(15);}
    public Trigger b_POV2Up() {return controller.button(16);}
    public Trigger b_POV2Right() {return controller.button(17);}
    public Trigger b_POV2Down() {return controller.button(18);}
    public Trigger b_POV2Left() {return controller.button(19);}
    public Trigger b_POV2Pushed() {return controller.button(20);}
    public Trigger b_TopTriggerDown() {return controller.button(22);}
    public Trigger b_TopTriggerUp() {return controller.button(21);}

    public Trigger p_Up() {return controller.povUp();}
    public Trigger p_Down() {return controller.povDown();}
    public Trigger p_Left() {return controller.povLeft();}
    public Trigger p_Right() {return controller.povRight();}
    public Trigger p_Any() {return controller.povCenter().negate();}


    private static double applyDeadzone(double input) {
        return (Math.abs(input) > kControllerDeadzone) ? 
        input
        : 0;
    }
}
