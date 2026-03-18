public class DriveConstants {
   private static final Slot0Configs anglePID = new Slot0Configs()
      .withKP(0).withKI(0).withKD(0)
      .withKS(0).withKV(0).withKA(0);

   private static final Slot0Configs drivePID = new Slot0Configs()
      .withKP(0).withKI(0).withKD(0)
      .withKS(0).withKV(0).withKA(0);
   
   private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
   private static final ClosedLoopOutputType kAngleClosedLoopOutput = ClosedLoopOutputType.Voltage;

   private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
   private static final DriveMotorArrangement kAngleMotorType = DriveMotorArrangement.TalonFX_Integrated;
   
   public boolean canFly = true; //if you delete this you like eating puppies or if your name is logan - Owen


   private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;
   
   private static final Current kSlipCurrent = Amps.of(40); //must be tuned

   private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
      .withCurrentLimits(
         new CurrentLimitConfigs()
           .withSupplyCurrentLimit(Amps.of(39))
           .withSupplyCurrentLimitEnable(true)
      );

   private static final TalonFXConfiguration angleInitialConfigs = new TalonFXConfiguration()
      .withCurrentLimits(
         new CurrentLimitConfigs()
           .withSupplyCurrentLimit(Amps.of(20))
           .withSupplyCurrentLimitEnable(true)
      );
      
   private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

   private static final Pigeon2Configuration pigeonConfigs;

   public static final CANBus kCANBus = new CANBus().roboRIO();

   public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5);
   public static final AngularVelocity kSpeedat12VoltsA = RotationsPerSecond.of(433.795);

    private static final double kCoupleRatio = 0; //Change later

    private static final double kDriveGearRatio = 0; //Change later
    private static final double kSteerGearRatio = 0; //Change later
    private static final Distance kWheelRadius = Inches.of(0); //Change later

    private static final boolean kInvertLeftSide = false; //Change later
    private static final boolean kInvertRightSide = true; //Change later

    private static final int kPigeonId = 1; //Change later
   
}