package frc.robot.subsystems.manipSubsystems;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private LEDPattern pattern;

    private final Color purple = Color.packRGB(128,0,128);
    private final Color yellow = Color.packRGB(255,255,0);

    private final int LEDPortNumber = 3;
    private final int LEDBuffer = 180;

    public LEDSubsystem() {
        led = new AddressableLED(LEDPortNumber);
        ledBuffer = new AddressableLEDBuffer(LEDBuffer);

        led.setLength(ledBuffer.getLength()); 

        led.setData(ledBuffer);
        led.start();

        // this.setDefaultCommand(defaultRainbow());

    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }

    public void setRed() {
        setColor(Color.kRed);
    }

    public void setColor(Color color) {
        pattern = LEDPattern.solid(color);
        pattern.applyTo(ledBuffer);
    }

    public void setColorsContinuous(Color c1, Color c2) {
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, c1, c2);
        pattern.applyTo(ledBuffer);
    }

    public void setPurpleAndYellowContinuous() {
        setColorsContinuous(purple, yellow);
    }

    public void setPurpleAndYellowDiscontinuous() {
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, purple, yellow);
        pattern.applyTo(ledBuffer);
    }

    public void setColorsSteps(Color colorsAreCoolAndTheyMakeRobotLookBetterBecauseColorsYouKnowLikeWhyNot, Color colorsAreCoolAndTheyMakeRobotLookBetterBecauseColorsYouKnowLikeWhyNotAndThenWeHaveSecondColorWhichMakesEvenMoreCoolnessOnRobotBecauseThenWeYesWeAreCool) {
        pattern = LEDPattern.steps(Map.of(0, colorsAreCoolAndTheyMakeRobotLookBetterBecauseColorsYouKnowLikeWhyNot, 0.5, colorsAreCoolAndTheyMakeRobotLookBetterBecauseColorsYouKnowLikeWhyNotAndThenWeHaveSecondColorWhichMakesEvenMoreCoolnessOnRobotBecauseThenWeYesWeAreCool));
        pattern.applyTo(ledBuffer);
    }
    
    public void setColorProgressMask() {
        pattern = LEDPattern.progressMaskLayer(() -> elevator.getHeight() / elevator.getMaxHeight());
        pattern.applyTo(ledBuffer);
    }

    public void setColorOffset(Color colorForOffset, Color secondColorForOffset) {
        pattern = LEDPattern.discontinuousGradient(colorForOffset, secondColorForOffset);
        pattern = base.offsetBy(40);
        pattern = base.offsetBy(-20);
        pattern.applyTo(ledBuffer);
    }

    public void setColorReverse(Color colorForReverse, Color secondColorForReverse) {
        pattern = LEDPattern.discontinuousGradient(colorForReverse, secondColorForReverse);
        pattern = base.reversed();
        pattern.applyTo(ledBuffer);
    }

    public void setColorScroll(Color colorForScroll, Color secondColorForScroll) {
        pattern = LEDPattern.discontinuousGradient(colorForScroll, secondColorForScroll);
        pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
        pattern = base.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);
        pattern.applyTo(ledBuffer);
    }

    public void setColorBreathe(Color colorForBreathe, Color secondColorForBreathe) {
        pattern = LEDPattern.discontinuousGradient(colorForBreathe, secondColorForBreathe);
        pattern = base.breathe(Seconds.of(2));
        pattern.applyTo(ledBuffer);
    }

    public void setColorBlink(Color colorForBlink, Color secondColorForBlink) {
        pattern = LEDPattern.discontinuousGradient(colorForBlink, secondColorForBlink);
        pattern = base.blink(Seconds.of(1.5));
        pattern = base.blink(Seconds.of(2), Seconds.of(1));
        pattern = base.synchronizedBlink(RobotController::getRSLState);
        pattern.applyTo(ledBuffer);
    }


    // public Command hasFuel() {
    //      Command out = new Command() {
    //         @Override public void execute() {
    //             setAllLEDs(new Color(255, 0, 0));
    //         }

    //         @Override public String getName() {
    //             return "ledNote";
    //         }
    //     };

    //     out.addRequirements(this);

    //     return out;
    // }
}
