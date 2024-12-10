package robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Ports;

public class Drive extends SubsystemBase {
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF.kS, FF.kV);
    private final CANSparkMax leftLeader = new CANSparkMax(Ports.Drive.LEFT_LEADER, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(Ports.Drive.LEFT_FOLLOWER, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(Ports.Drive.RIGHT_LEADER, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(Ports.Drive.RIGHT_FOLLOWER, MotorType.kBrushless);


     private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
     private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

     private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);

     private final DifferentialDriveOdometry odometry;
    public Drive() {
      gyro.reset();
      for (CANSparkMax spark : List.of(leftLeader, leftFollower, rightLeader, rightFollower)) {
	    spark.restoreFactoryDefaults();
	    spark.setIdleMode(IdleMode.kBrake);
      }
      leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
      rightEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);

      leftEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
      rightEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);

      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
    }
    
    rightFollower.follow(rightLeader);
    leftFollower.follow(leftLeader);

    leftLeader.setInverted(true);

    odometry = new DifferentialDriveOdometry(
            new Rotation2d(), 
            0, 
            0, 
            new Pose2d());

    private void drive(double leftSpeed, double rightSpeed) {
      leftLeader.set(leftSpeed);
      rightLeader.set(rightSpeed);
    }
    private void updateOdometry(Rotation2d rotation) {
      odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
    }
    public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight) {
        return run(() -> drive(vLeft.getAsDouble(), vRight.getAsDouble()));
    }
    @Override 
    public void periodic() {
      updateOdometry(gyro.getRotation2d());
    }

    public Pose2d pose() {
      return odometry.getPoseMeters();
    }
}
