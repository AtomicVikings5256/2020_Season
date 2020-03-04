package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

//Drive
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//Output
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//Input
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
//NetworkTable
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//Motor Controllers
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

//Pnuematics 
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase;
//Auto Things
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
  //Variables
  private boolean search;

  //Motor Controllers
  private WPI_TalonFX frontLeft, frontRight, rearLeft, rearRight, leftShooter, rightShooter;
  private CANSparkMax turret, preroller, vertConvey, indexer, intake, climber;
  private TalonFXSensorCollection shooterSensor, leftSensor, rightSensor;
  private DifferentialDrive drive;
  private SpeedControllerGroup leftDrivey, rightDrivey, shooters;
  private VictorSPX funnel;
  
  //Misc
  private Timer time;
  private Joystick driver, mechanic;

  //Auto Things
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;
  private AHRS gyro;
  private Pose2d pose;

  //NetworkTable
  private NetworkTable table;
  private double tx;
  private double tv;
  private double ty;

  //Turret sTUFF
  private double kp;
  private double min_command;
  private double turret_adjust;
  private double state;

  //Sensors
  private DigitalInput bottomCell, middleCell, topCell;

  //Solenoids
  private Solenoid shifter1, shifter2, leftIntake, rightIntake;
  private Compressor compressor;
 
  @Override
  public void robotInit() {
    //Auto Stuff
    time     = new Timer();
    driver = new Joystick(0);
    mechanic = new Joystick(1);
    gyro = new AHRS(I2C.Port.kMXP);
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28.5));
    odometry = new DifferentialDriveOdometry(getHeading());

    //Shooters
    leftShooter = new WPI_TalonFX(9);
    rightShooter = new WPI_TalonFX(10);
    leftShooter.setInverted(true);
    rightShooter.setInverted(false);
    shooters = new SpeedControllerGroup(leftShooter, rightShooter);
    
    //Intake and Conveyors
    turret = new CANSparkMax(13, MotorType.kBrushless);
    preroller = new CANSparkMax(11, MotorType.kBrushless);
    indexer = new CANSparkMax(14, MotorType.kBrushless);
    vertConvey = new CANSparkMax(12, MotorType.kBrushless);
    intake = new CANSparkMax(15, MotorType.kBrushless);
    funnel = new VictorSPX(16);
    climber = new CANSparkMax(17, MotorType.kBrushless);

    //Drive
    frontLeft = new WPI_TalonFX(5);
    frontRight = new WPI_TalonFX(6);
    rearLeft = new WPI_TalonFX(7);
    rearRight = new WPI_TalonFX(8);
    leftDrivey  = new SpeedControllerGroup(frontLeft, rearLeft);
    rightDrivey = new SpeedControllerGroup(frontRight, rearRight);
    drive = new DifferentialDrive(leftDrivey, rightDrivey);

    //NetworkTable stuffs
    table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("ledMode").forceSetNumber(1);
    tx = table.getEntry("tx").getDouble(0.0);
    tv = table.getEntry("tv").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);

    //Turret sTUFFs
    kp = -0.1;
    min_command = 0.05;
    turret_adjust = 0.0;
    state = 0;
    search =  true;

    //Sensors
    bottomCell = new DigitalInput(0);
    middleCell = new DigitalInput(1);
    topCell = new DigitalInput(2);

    //Encoders
    leftSensor = new TalonFXSensorCollection(frontLeft);
    rightSensor = new TalonFXSensorCollection(frontRight);

    //pneumatics
    shifter1 = new Solenoid(0);
    shifter2 = new Solenoid(1);
    leftIntake = new Solenoid(2);
    rightIntake = new Solenoid(3);
    compressor = new Compressor();
  }
  
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Top ", topCell.get());
    SmartDashboard.putBoolean("Mid ", middleCell.get());
    SmartDashboard.putBoolean("Bot ", bottomCell.get());
    SmartDashboard.putNumber("Velocity ", shooterSensor.getIntegratedSensorVelocity());
    //Update the odometry in the periodic block
    pose = odometry.update(
      getHeading(), 
      leftSensor.getIntegratedSensorVelocity() / 7.29 * 2 * Math.PI  * Units.inchesToMeters(3.5) / 60, 
      rightSensor.getIntegratedSensorVelocity() / 7.29 * 2 * Math.PI  * Units.inchesToMeters(3.5) / 60);
    
  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
     
  }

  @Override
  public void teleopInit() {
    turret.getEncoder().setPosition(0);
    turret.setOpenLoopRampRate(.8);
    leftIntake.set(true);
    rightIntake.set(true);
  }

  @Override
  public void teleopPeriodic() {
    //Drive
    drive.arcadeDrive(driver.getRawAxis(1), driver.getRawAxis(4));

    //Shooter
    if (driver.getRawButton(7)) {
      shooters.set(1);
    } else {
      shooters.set(0.0);
    }

    //Shifting
    if (driver.getRawButton(2)) {
      shifter1.set(true);
      shifter2.set(true);
    } else if (driver.getRawButton(3)) {
      shifter1.set(false);
      shifter2.set(false);
    }


    //Turret
    if (driver.getRawButtonPressed(9)) {
      search = !search;
    } 

    if (search) {
      limelight();
    }

    if (!search) {
      if (mechanic.getRawAxis(2) == 1) {
        turret.set(-1);
      } else if (mechanic.getRawAxis(2) == -1) {
        turret.set(1);
      }
    }

    SmartDashboard.putBoolean("Auto Search ", search);

    //Preroller
    if (driver.getRawButton(8)) {
      preroller.set(1);
    } else {
      preroller.set(0);
    }
      
    //Intake
    if (driver.getRawButton(5)) {
      intake.set(-1);
    } else if (driver.getRawButton(6)) {
      intake.set(1);
    } else {
      intake.set(0);
    }

    //Mechanics
    if (mechanic.getPOV() == 0) {
      vertConvey.set(1);
    } else if (mechanic.getPOV() == 4) {
      vertConvey.set(-1);
    } else {
      vertConvey.set(0);
    }

    if (mechanic.getRawButton(3)) {
      indexer.set(-1);
    } else if (mechanic.getRawButton(1)) {
      indexer.set(1);
    } else {
      indexer.set(0);
    }

    if (mechanic.getRawButton(2)) {
      funnel.set(ControlMode.PercentOutput, 100);
    } else {
      funnel.set(ControlMode.PercentOutput, 0);
    }
      
    if(mechanic.getRawButton(5)) {
      leftIntake.set(true);
      rightIntake.set(true);
    } else if (mechanic.getRawButton(6)) {
      leftIntake.set(false);
      rightIntake.set(false);
    }

    if(mechanic.getRawButton(4)) {
      climber.set(0.5);
    } else {
      climber.set(0.0);
    }
  }

  @Override
  public void testPeriodic() {

  }

  public void limelight() {
    tx = table.getEntry("tx").getDouble(0.0);
    tv = table.getEntry("tv").getDouble(0.0);
    turret_adjust = 0.0;
    double heading_error = -tx;
    double Eposition = turret.getEncoder().getPosition();
    boolean timeToSearch = false;
    boolean oneTime = true;
    boolean twoTime = true;
    time.reset();
    time.start();
    if (tv == 1) {
      oneTime = true;
      twoTime = true;
      timeToSearch = false;
      SmartDashboard.putBoolean("Status of target ", true);
      if (tx > 1.0) {
        turret_adjust = kp*heading_error + min_command;
      } else if (tx < 1.0) {
        turret_adjust = kp*heading_error - min_command;
      }
    } else if (tv == 0) {
      SmartDashboard.putBoolean("Status of target ", false);
      if (oneTime) {
        oneTime = false;
        state = time.get();
      } else if (twoTime && time.get() > state + 5) {
        twoTime = false;
        timeToSearch = true;
      }
      if (timeToSearch) {
        if (Eposition == 5) {
          turret_adjust = -0.1;
        } else if (Eposition == -5) {
          turret_adjust = 0.1;
        }
      }
    }
    turret.set(turret_adjust);
    SmartDashboard.putNumber("Turret position ", Eposition);
  }
  
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }


}