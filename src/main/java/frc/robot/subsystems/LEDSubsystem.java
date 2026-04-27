package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.FrequencyUnit;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.drive.GameState;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem instance;
	public static LEDSubsystem getInstance() {
		if (instance == null) {
			instance = new LEDSubsystem();
		}
		return instance;
	}
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    
    private final Color purple = RGBPacker(230, 0, 255);
    private final Color yellow = RGBPacker(255, 255, 0);
    private final Color red = RGBPacker(255, 0, 0);
    private final Color green = RGBPacker(0, 255, 0);
    private final Color blue = RGBPacker(0, 0, 225);

    private final int LEDPortNumber = 0;
    private final int LEDBuffer = 776;

    private SendableChooser<Color> colorChooser;

    private SendableChooser<LEDPattern> paternChooser;

    private LEDSubsystem() {
        led = new AddressableLED(LEDPortNumber);

        ledBuffer = new AddressableLEDBuffer(LEDBuffer);    

        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        // led.setColorOrder(ColorOrder.kGRB);
       
        led.start();
       // new led chooser for the LEDPatterns, command that runs the pattern we choose with the colors we choose
        colorChooser = new SendableChooser<Color>();
        colorChooser.setDefaultOption("Purple", purple);
        colorChooser.addOption("Yellow", yellow);
        colorChooser.addOption("Blue", blue);
        colorChooser.addOption("Green", green);
        colorChooser.addOption("Red", red);
        colorChooser.addOption("None :(", Color.kBlack);
        SmartDashboard.putData("LED Color Chooser", colorChooser);

        paternChooser = new SendableChooser<LEDPattern>();
        paternChooser.setDefaultOption("Solid", setColor(selectColor()));
        paternChooser.addOption("Breath", setColorBreathe(selectColor()));
        paternChooser.addOption("Blink", setColorBlink(selectColor()));
        paternChooser.addOption("Scroll", setColorScroll(selectColor(), Color.kBlack));
        SmartDashboard.putData("LED Pattern Chooser", paternChooser);


        
        this.setDefaultCommand(runPatternWithSendable());

    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }

    /*
     * Takes in RGB values and makes a new Color with the BGR values for the LEDs to use.
     */
    public static Color RGBPacker(int red, int green, int blue) {
        return new Color(red, green, blue);
    }

    public Command hubActive() {
        return run(() -> {
            if (GameState.isMyHubActive()) {
                setGreen().applyTo(ledBuffer);
            } else {
                setRed().applyTo(ledBuffer);
            }  
        }).ignoringDisable(true);
        
    }

    public Command runPatternWithSendable() {
        return run(() -> paternChooser.getSelected().applyTo(ledBuffer)).ignoringDisable(true);
    }

    public Color selectColor() {
        return colorChooser.getSelected();
    }

    public Command runColorWithSendable() {
        return run(() -> setColor(colorChooser.getSelected()).applyTo(ledBuffer))
            .ignoringDisable(true);
    }

    public Command runWithAlliance() {
        return run(() -> {
            if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                setColor(red).applyTo(ledBuffer);
            } else {
                setColor(blue).applyTo(ledBuffer);
            }
        }).ignoringDisable(true);
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(ledBuffer)).ignoringDisable(true);
    }

    public LEDPattern setColor(Color color) {
        return LEDPattern.solid(color).atBrightness(Percent.of(50));
    }

    public LEDPattern setPurple() {
        return setColor(purple);
    }

    public LEDPattern setYellow() {
        return setColor(yellow);
    }

    public LEDPattern setRed() {
        return setColor(red);
    }

    public LEDPattern setGreen() {
        return setColor(green);
    }

    public LEDPattern setColorsContinuous(Color c1, Color c2) {
        return LEDPattern.gradient(GradientType.kContinuous, c1, c2);
    }

    public LEDPattern setHawkContinuous() {
        return setColorsContinuous(purple, yellow);
    }

    public LEDPattern setColorsSteps(Color c1, Color c2) {
       return LEDPattern.steps(Map.of(0, c1, 0.5, c2));
    }

    public LEDPattern setHawkSteps() {
        return setColorsSteps(purple, yellow);
    }
   
    public LEDPattern setColorProgressMask(Color forProgressMask, Color secondForProgressMask, double currentHeight, double maxHeight) {
        return LEDPattern.progressMaskLayer(() -> {return (currentHeight / maxHeight);});
    }

    public LEDPattern setHawkMask(double currentHeight, double maxHeight) {
        return setColorProgressMask(purple, yellow, currentHeight, maxHeight);
    }

    public LEDPattern setColorOffset(Color colorForOffset, Color secondColorForOffset) {
        return setDiscontGradient(colorForOffset, secondColorForOffset).offsetBy(LEDBuffer/10);
    }

    public LEDPattern setHawkOffset() {
        return setColorOffset(purple, yellow);
    }

    public LEDPattern setColorReverse(Color colorForReverse, Color secondColorForReverse) {
        return setDiscontGradient(colorForReverse, secondColorForReverse).reversed();
    }

    public LEDPattern setHawkReverse() {
        return setColorReverse(purple, yellow);
    }

    public LEDPattern setColorScroll(Color colorForScroll, Color secondColorForScroll) {
        return setDiscontGradient(colorForScroll, secondColorForScroll).scrollAtRelativeSpeed(Frequency.ofBaseUnits(0.25, Hertz));
    }

    public LEDPattern setHawkScroll() {
        return setColorScroll(purple, yellow);
    }

    public LEDPattern setColorBreathe(Color colorForBreathe) {
        return setColor(colorForBreathe).breathe(Seconds.of(3)).atBrightness(Percent.of(50));
    }

    public LEDPattern setColorBlink(Color colorForBlink) {
        return setColor(colorForBlink).blink(Seconds.of(0.4));
    }

    public LEDPattern setColorBrightness(Color colorForBrightness, Color secondColorForBrightness, double constantBrightnessLevelIsCool) {
        return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,colorForBrightness, secondColorForBrightness);
    }

    public LEDPattern setHawkBrightness(double brightnessLevel) {
        return setColorBrightness(purple, yellow, brightnessLevel);
    }

    public LEDPattern setColorMask(Color colorForMask, Color secondColorForMask) {
        return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colorForMask, secondColorForMask);
    }

    public LEDPattern setHawkMask() {
        return setColorMask(purple, yellow);
    }

    public LEDPattern setDiscontGradient(Color c1, Color c2) {
        return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, c1, c2);
    }

    public LEDPattern setDiscontGradientHawk() {
        return setDiscontGradient(purple, yellow);
    }
}