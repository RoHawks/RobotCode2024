package universalSwerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import universalSwerve.components.Wheel;
import universalSwerve.components.WheelLabel;
import universalSwerve.components.implementations.FalconTranslationSystem;
import universalSwerve.components.implementations.SparkMaxAndLampreyRotationSystem;
import universalSwerve.components.implementations.SparkMaxUtilizingAbsoluteEncoderRotationSystemContinuously;
import universalSwerve.components.implementations.TalonFXTranslationSystem;
import universalSwerve.components.implementations.TalonSRXAndAbsoluteEncoderRotationSystem;
import universalSwerve.components.implementations.TalonSRXOpenLoopTranslationSystem;
import universalSwerve.hardware.implementations.ADIS16470Gyro;
import universalSwerve.hardware.implementations.LampreyWheelAngleReader;
import universalSwerve.hardware.implementations.NavX;
import universalSwerve.hardware.implementations.Pigeon2Imu;
import universalSwerve.hardware.implementations.ZeroGyro;
import universalSwerve.utilities.PIDFConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class SwerveFactory 
{
     public static SwerveDrive Create2024Swerve()
    {        

        double[] leftRightLocationsRelativeToCenter = new double[] {12.25, 12.25, -12.25, -12.25 };
        double[] frontBackLocationsRelativeToCenter = new double[] {15.0625-3.75, -(15.0625-1.875), -(15.0625-1.875), 15.0625-3.75 };


        //Wheel details:
        double WHEEL_DIAMETER = 3.0;
        double DRIVE_GEAR_RATIO = 14.0/22.0*15.0/45.0;

        double MAX_LINEAR_SPEED = 15.0 * 12.0; //was 15 at hofstra
        double MAX_ROTATIONAL_SPEED = 120; //degrees per second
        double NUDGING_SPEED = 0.1;// * 0.15 / 0.17; //scaled down to keep actual nudging speed consistent

        //ADIS16470Gyro gyro = new ADIS16470Gyro(edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kZ, true);
        Pigeon2Imu gyro = new Pigeon2Imu(true);
                //Drive FPID Configurations:
        PIDFConfiguration neDriveFPIDCongiruation = new PIDFConfiguration(0.000, 0, 0,  0.12);
        PIDFConfiguration seDriveFPIDCongiruation = new PIDFConfiguration(0.000, 0, 0,  0.12);
        PIDFConfiguration swDriveFPIDCongiruation = new PIDFConfiguration(0.000, 0, 0,  0.12);
        PIDFConfiguration nwDriveFPIDCongiruation = new PIDFConfiguration(0.00, 0, 0,  0.12);
        PIDFConfiguration[] driveFPIDConfigurations = new PIDFConfiguration[]{ neDriveFPIDCongiruation, seDriveFPIDCongiruation, swDriveFPIDCongiruation, nwDriveFPIDCongiruation};



        //Turn FPID Configurations:
        PIDFConfiguration neTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration seTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration swTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration nwTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration[] turnFPIDConfigurations= new PIDFConfiguration[]{neTurnFPIDCongiruation, seTurnFPIDCongiruation, swTurnFPIDCongiruation, nwTurnFPIDCongiruation };
        

        //Drive Ports:
        int neDriveChannel = 1;
        int seDriveChannel = 62;
        int swDriveChannel = 19;
        int nwDriveChannel = 18;

        //Turn Ports:
        int neTurnChannel = 3;
        int seTurnChannel = 2;
        int swTurnChannel = 17;
        int nwTurnChannel = 16;

        CurrentLimitsConfigs driveCurrentLimitConfig = new CurrentLimitsConfigs();
        driveCurrentLimitConfig.SupplyCurrentLimit = 40;
        driveCurrentLimitConfig.SupplyCurrentThreshold = 60;
        driveCurrentLimitConfig.SupplyCurrentLimitEnable = true;
        
        MotorOutputConfigs driveMotorOutputConfig = new MotorOutputConfigs();
        //driveMotorOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        driveMotorOutputConfig.NeutralMode = NeutralModeValue.Brake;        

        FeedbackConfigs driveMotorfeedbackConfigs = new FeedbackConfigs();
        driveMotorfeedbackConfigs.FeedbackSensorSource  = FeedbackSensorSourceValue.RotorSensor;

        ClosedLoopRampsConfigs driveMotorClosedLoopRampConfigs = new ClosedLoopRampsConfigs();
        driveMotorClosedLoopRampConfigs.DutyCycleClosedLoopRampPeriod = 0.05;
        driveMotorClosedLoopRampConfigs.VoltageClosedLoopRampPeriod= 0.05;
        


        class DriveFalconCreator { TalonFX CreateDriveFalcon(int pDriveChannel) 
            { 
                TalonFX returnValue = new TalonFX(pDriveChannel);
                returnValue.getConfigurator().apply(driveCurrentLimitConfig);                                
                returnValue.getConfigurator().apply(driveMotorOutputConfig);                
                returnValue.getConfigurator().apply(driveMotorfeedbackConfigs);                
                returnValue.getConfigurator().apply(driveMotorClosedLoopRampConfigs);
                
                returnValue.setInverted(false);
                                                
                return returnValue;
            }}

        //Drive Falcons:
        DriveFalconCreator driveFalconCreator = new DriveFalconCreator();
        TalonFX neDriveFalcon = driveFalconCreator.CreateDriveFalcon(neDriveChannel);
        TalonFX seDriveFalcon = driveFalconCreator.CreateDriveFalcon(seDriveChannel);
        TalonFX swDriveFalcon = driveFalconCreator.CreateDriveFalcon(swDriveChannel);
        TalonFX nwDriveFalcon = driveFalconCreator.CreateDriveFalcon(nwDriveChannel);
        TalonFX[] driveFalcons = new TalonFX[] {neDriveFalcon, seDriveFalcon, swDriveFalcon, nwDriveFalcon};


        class TurnSparkMaxCreator { CANSparkMax CreateTurnSparkMax(int pTurnChannel, double pZeroOffset) {             
            CANSparkMax returnValue = new CANSparkMax(pTurnChannel, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);            
            returnValue.restoreFactoryDefaults();            
            returnValue.setSmartCurrentLimit(20);                    
            SparkAbsoluteEncoder absoluteEncoder = returnValue.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
            absoluteEncoder.setInverted(true);            
            absoluteEncoder.setPositionConversionFactor(1);            
            absoluteEncoder.setZeroOffset(pZeroOffset);            
            returnValue.getPIDController().setFeedbackDevice(absoluteEncoder);            
            return returnValue;
        }}

        //Turn Spak Maxes        
        TurnSparkMaxCreator turnSparkMaxCreator = new TurnSparkMaxCreator();        
        CANSparkMax neTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(neTurnChannel, 0.3444+0.500);
        CANSparkMax seTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(seTurnChannel, 0.2546-0.250);
        CANSparkMax swTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(swTurnChannel, 0.5122);
        CANSparkMax nwTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(nwTurnChannel, 0.9434 -0.750);        
        CANSparkMax[] turnSparkMaxes = new CANSparkMax[] { neTurnSparkMax,seTurnSparkMax, swTurnSparkMax, nwTurnSparkMax};

        Wheel[] wheels = new Wheel[4];

      
        for(int i = 0; i < 4; i++)
        {
            wheels[i] = new Wheel(WheelLabel.FromInt(i), 
                   leftRightLocationsRelativeToCenter[i],
                   frontBackLocationsRelativeToCenter[i],
            new TalonFXTranslationSystem(driveFalcons[i], driveFPIDConfigurations[i], WHEEL_DIAMETER, DRIVE_GEAR_RATIO),
            new SparkMaxUtilizingAbsoluteEncoderRotationSystemContinuously(turnSparkMaxes[i], turnFPIDConfigurations[i]));
        }        
        SwerveDrive returnValue = new SwerveDrive(wheels[0], wheels[1], wheels[2], wheels[3], gyro, MAX_LINEAR_SPEED, MAX_ROTATIONAL_SPEED, NUDGING_SPEED);
    

       neTurnSparkMax.burnFlash();
       seTurnSparkMax.burnFlash();
       swTurnSparkMax.burnFlash();
       nwTurnSparkMax.burnFlash();

       return returnValue;

    }

    public static SwerveDrive Create2023Swerve()
    {        

        
        //Locations:
        double WHEELS_LEFT_RIGHT_FROM_CENTER = 23.75/2.0;
        double WHEELS_FRONT_BACK_FROM_CENTER = 26.5/2.0;

        //Wheel details:
        double WHEEL_DIAMETER = 3.0;
        double DRIVE_GEAR_RATIO = 14.0/22.0*15.0/45.0;

        double MAX_LINEAR_SPEED = 15.0 * 12.0; //was 15 at hofstra
        double MAX_ROTATIONAL_SPEED = 120; //degrees per second
        double NUDGING_SPEED = 0.1;// * 0.15 / 0.17; //scaled down to keep actual nudging speed consistent

        ADIS16470Gyro gyro = new ADIS16470Gyro(edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kZ, true);

        //Drive FPID Configurations:
        PIDFConfiguration neDriveFPIDCongiruation = new PIDFConfiguration(0.083, 0, 0,  0.046);
        PIDFConfiguration seDriveFPIDCongiruation = new PIDFConfiguration(0.083, 0, 0,  0.046);
        PIDFConfiguration swDriveFPIDCongiruation = new PIDFConfiguration(0.083, 0, 0,  0.046);
        PIDFConfiguration nwDriveFPIDCongiruation = new PIDFConfiguration(0.083, 0, 0,  0.046);
        PIDFConfiguration[] driveFPIDConfigurations = new PIDFConfiguration[]{ neDriveFPIDCongiruation, seDriveFPIDCongiruation, swDriveFPIDCongiruation, nwDriveFPIDCongiruation};



        //Turn FPID Configurations:
        PIDFConfiguration neTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration seTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration swTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration nwTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration[] turnFPIDConfigurations= new PIDFConfiguration[]{neTurnFPIDCongiruation, seTurnFPIDCongiruation, swTurnFPIDCongiruation, nwTurnFPIDCongiruation };
        

        //Drive Ports:
        int neDriveChannel = 19;
        int seDriveChannel = 18;
        int swDriveChannel = 1;
        int nwDriveChannel = 62;

        //Turn Ports:
        int neTurnChannel = 17;
        int seTurnChannel = 15;
        int swTurnChannel = 4;
        int nwTurnChannel = 2;

        class DriveFalconCreator { WPI_TalonFX CreateDriveFalcon(int pDriveChannel) 
            { 
                WPI_TalonFX returnValue = new WPI_TalonFX(pDriveChannel);
                returnValue.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 2));
                returnValue.setInverted(true);    
                returnValue.setNeutralMode(NeutralMode.Coast);
                returnValue.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,0);
                returnValue.configClosedloopRamp(0.05);
                return returnValue;
            }}

        //Drive Falcons:
        DriveFalconCreator driveFalconCreator = new DriveFalconCreator();
        WPI_TalonFX neDriveFalcon = driveFalconCreator.CreateDriveFalcon(neDriveChannel);
        WPI_TalonFX seDriveFalcon = driveFalconCreator.CreateDriveFalcon(seDriveChannel);
        WPI_TalonFX swDriveFalcon = driveFalconCreator.CreateDriveFalcon(swDriveChannel);
        WPI_TalonFX nwDriveFalcon = driveFalconCreator.CreateDriveFalcon(nwDriveChannel);
        WPI_TalonFX[] driveFalcons = new WPI_TalonFX[] {neDriveFalcon, seDriveFalcon, swDriveFalcon, nwDriveFalcon};


        class TurnSparkMaxCreator { CANSparkMax CreateTurnSparkMax(int pTurnChannel, double pZeroOffset) {             
            CANSparkMax returnValue = new CANSparkMax(pTurnChannel, MotorType.kBrushless);            
            returnValue.restoreFactoryDefaults();            
            returnValue.setSmartCurrentLimit(20);            
            SparkMaxAbsoluteEncoder absoluteEncoder = returnValue.getAbsoluteEncoder(Type.kDutyCycle);            
            absoluteEncoder.setInverted(true);            
            absoluteEncoder.setPositionConversionFactor(1);            
            absoluteEncoder.setZeroOffset(pZeroOffset);            
            returnValue.getPIDController().setFeedbackDevice(absoluteEncoder);            
            return returnValue;
        }}

        //Turn Spak Maxes        
        TurnSparkMaxCreator turnSparkMaxCreator = new TurnSparkMaxCreator();        
        CANSparkMax neTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(neTurnChannel, 0.3444+0.500);
        CANSparkMax seTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(seTurnChannel, 0.2546-0.250);
        CANSparkMax swTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(swTurnChannel, 0.5122);
        CANSparkMax nwTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(nwTurnChannel, 0.9434 -0.750);        
        CANSparkMax[] turnSparkMaxes = new CANSparkMax[] { neTurnSparkMax,seTurnSparkMax, swTurnSparkMax, nwTurnSparkMax};

        Wheel[] wheels = new Wheel[4];

      
        for(int i = 0; i < 4; i++)
        {
            wheels[i] = new Wheel(WheelLabel.FromInt(i), 
                ((i == 0 || i == 1) ? 1.0 : -1.0) * WHEELS_LEFT_RIGHT_FROM_CENTER,
                ((i == 0 || i == 3) ? 1.0 : -1.0) * WHEELS_FRONT_BACK_FROM_CENTER,                
            new FalconTranslationSystem(driveFalcons[i], driveFPIDConfigurations[i], WHEEL_DIAMETER, DRIVE_GEAR_RATIO),
            new SparkMaxUtilizingAbsoluteEncoderRotationSystemContinuously(turnSparkMaxes[i], turnFPIDConfigurations[i]));
        }        
        SwerveDrive returnValue = new SwerveDrive(wheels[0], wheels[1], wheels[2], wheels[3], gyro, MAX_LINEAR_SPEED, MAX_ROTATIONAL_SPEED, NUDGING_SPEED);
    

       neTurnSparkMax.burnFlash();
       seTurnSparkMax.burnFlash();
       swTurnSparkMax.burnFlash();
       nwTurnSparkMax.burnFlash();

       return returnValue;

    }





    public static SwerveDrive Create2022Swerve()
    {        
        //Locations:
        double WHEELS_LEFT_RIGHT_FROM_CENTER = 10.625;
        double WHEELS_FRONT_BACK_FROM_CENTER = 12.1875;

        //Wheel details:
        double WHEEL_DIAMETER = 3.25;
        double DRIVE_GEAR_RATIO = 9.0 / 30.0 * 24.0 / 36.0 * 34.0 / 54.0;

        double TURN_GEAR_RATIO = (29.0/84.0) * (21.0/76.0) * (13.0/68.0) * (20.0/32.0);;

        double MAX_LINEAR_SPEED = 3 * 12; //inches per second //original was 9 * 12
        double MAX_ROTATIONAL_SPEED = 120; //degrees per second
        double NUDGING_SPEED = 2;

        ADIS16470Gyro gyro = new ADIS16470Gyro(edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kX, true);

        //Drive FPID Configurations:
        PIDFConfiguration neDriveFPIDCongiruation = new PIDFConfiguration(0.083, 0, 0, 0.04856 * 1.026* 1.1);
        PIDFConfiguration seDriveFPIDCongiruation = new PIDFConfiguration(0.063, 0, 0, 0.045 * 1.037 * 1.021 * 1.3);
        PIDFConfiguration swDriveFPIDCongiruation = new PIDFConfiguration(0.073, 0, 0, 0.0455 * 1.1);
        PIDFConfiguration nwDriveFPIDCongiruation = new PIDFConfiguration(0.083, 0, 0,  0.046);
        PIDFConfiguration[] driveFPIDConfigurations = new PIDFConfiguration[]{ neDriveFPIDCongiruation, seDriveFPIDCongiruation, swDriveFPIDCongiruation, nwDriveFPIDCongiruation};



        //Turn FPID Configurations:
        PIDFConfiguration neTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration seTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration swTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration nwTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration[] turnFPIDConfigurations= new PIDFConfiguration[]{neTurnFPIDCongiruation, seTurnFPIDCongiruation, swTurnFPIDCongiruation, nwTurnFPIDCongiruation };
        

        //Drive Ports:
        int neDriveChannel = 12;
        int seDriveChannel = 17;
        int swDriveChannel = 2;
        int nwDriveChannel = 7;

        //Turn Ports:
        int neTurnChannel = 19;
        int seTurnChannel = 18;
        int swTurnChannel = 62;
        int nwTurnChannel = 1;

        class DriveFalconCreator { WPI_TalonFX CreateDriveFalcon(int pDriveChannel) { return new WPI_TalonFX(pDriveChannel); }}

        //Drive Falcons:
        DriveFalconCreator driveFalconCreator = new DriveFalconCreator();
        WPI_TalonFX neDriveFalcon = driveFalconCreator.CreateDriveFalcon(neDriveChannel);
        WPI_TalonFX seDriveFalcon = driveFalconCreator.CreateDriveFalcon(seDriveChannel);
        WPI_TalonFX swDriveFalcon = driveFalconCreator.CreateDriveFalcon(swDriveChannel);
        WPI_TalonFX nwDriveFalcon = driveFalconCreator.CreateDriveFalcon(nwDriveChannel);
        WPI_TalonFX[] driveFalcons = new WPI_TalonFX[] {neDriveFalcon, seDriveFalcon, swDriveFalcon, nwDriveFalcon};


        class TurnSparkMaxCreator { CANSparkMax CreateTurnSparkMax(int pTurnChannel) { 
            CANSparkMax returnValue = new CANSparkMax(pTurnChannel, MotorType.kBrushless); 
            returnValue.setSmartCurrentLimit(20);
            return returnValue;
        }}

        //Turn Spak Maxes
        TurnSparkMaxCreator turnSparkMaxCreator = new TurnSparkMaxCreator();
        CANSparkMax neTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(neTurnChannel);
        CANSparkMax seTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(seTurnChannel);
        CANSparkMax swTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(swTurnChannel);
        CANSparkMax nwTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(nwTurnChannel);
        CANSparkMax[] turnSparkMaxes = new CANSparkMax[] { neTurnSparkMax,seTurnSparkMax, swTurnSparkMax, nwTurnSparkMax};

        

        //Lampreys:
        LampreyWheelAngleReader neLampreyAngleReader = new LampreyWheelAngleReader(4, 0);
        LampreyWheelAngleReader seLampreyAngleReader = new LampreyWheelAngleReader(1, 0);
        LampreyWheelAngleReader swLampreyAngleReader = new LampreyWheelAngleReader(2, 0);
        LampreyWheelAngleReader nwLampreyAngleReader = new LampreyWheelAngleReader(3, 0);
        LampreyWheelAngleReader[] lampreyAngleReaders = new LampreyWheelAngleReader[] {neLampreyAngleReader, seLampreyAngleReader, swLampreyAngleReader, nwLampreyAngleReader };


        Wheel[] wheels = new Wheel[4];

      
        for(int i = 0; i < 4; i++)
        {
            wheels[i] = new Wheel(WheelLabel.FromInt(i), 
                ((i == 0 || i == 1) ? 1.0 : -1.0) * WHEELS_LEFT_RIGHT_FROM_CENTER,
                ((i == 0 || i == 3) ? 1.0 : -1.0) * WHEELS_FRONT_BACK_FROM_CENTER,                
            new FalconTranslationSystem(driveFalcons[i], driveFPIDConfigurations[i], WHEEL_DIAMETER, DRIVE_GEAR_RATIO),
            new SparkMaxAndLampreyRotationSystem(turnSparkMaxes[i], TURN_GEAR_RATIO, turnFPIDConfigurations[i], lampreyAngleReaders[i]));
        }

       SwerveDrive returnValue = new SwerveDrive(wheels[0], wheels[1], wheels[2], wheels[3], gyro, MAX_LINEAR_SPEED, MAX_ROTATIONAL_SPEED, NUDGING_SPEED);

       return returnValue;

    }


    public static SwerveDrive CreateDedicatedTestChassisSwerve()
    {        

        
        //Locations:
        double WHEELS_LEFT_RIGHT_FROM_CENTER = 29.25/2.0;
        double WHEELS_FRONT_BACK_FROM_CENTER = 27.5/2.0;

        //Wheel details:
        double WHEEL_DIAMETER = 3.25;
        double DRIVE_GEAR_RATIO = 12.0/60.0*1.25/2.72*35.0/65.0;  //Rough, since I'm not sure I have the CAD and don't want to count gear teeth!

        double MAX_LINEAR_SPEED = 3.0 * 12.0; //This guy needs to drive awfully slow because he has the 3DP wheels 
        double MAX_ROTATIONAL_SPEED = 60; //degrees per second, //This guy needs to drive awfully slow because he has the 3DP wheels
        double NUDGING_SPEED = 0.7;// * 0.15 / 0.17; //scaled down to keep actual nudging speed consistent

        //ADIS16470Gyro gyro = new ADIS16470Gyro(edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kZ, true);
        //Need to import NavX library

      
        //Turn FPID Configurations:
        PIDFConfiguration neTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration seTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration swTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration nwTurnFPIDCongiruation = new PIDFConfiguration(10, 0, 0, 0);
        PIDFConfiguration[] turnFPIDConfigurations= new PIDFConfiguration[]{neTurnFPIDCongiruation, seTurnFPIDCongiruation, swTurnFPIDCongiruation, nwTurnFPIDCongiruation };
        
        //determine these
        //Drive Ports:
        int neDriveChannel = 9;
        int seDriveChannel = 16;
        int swDriveChannel = 6;
        int nwDriveChannel = 7;


        //determine these
        //Turn Ports:
        int neTurnChannel = 10;
        int seTurnChannel = 1;
        int swTurnChannel = 2;
        int nwTurnChannel = 3;

        class Drive775ProCreator { TalonSRX CreateDriveMotor(int pDriveChannel) 
            { 
                TalonSRX returnValue = new TalonSRX(pDriveChannel);
                returnValue.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 2));
                returnValue.setInverted(false);    
                returnValue.setNeutralMode(NeutralMode.Coast);
                returnValue.configClosedloopRamp(0.05);
                return returnValue;
            }}

        //Drive Motors:
        Drive775ProCreator driveMotorCreator = new Drive775ProCreator();
        TalonSRX neDriveMotor = driveMotorCreator.CreateDriveMotor(neDriveChannel);
        TalonSRX seDriveMotor = driveMotorCreator.CreateDriveMotor(seDriveChannel);
        TalonSRX swDriveMotor = driveMotorCreator.CreateDriveMotor(swDriveChannel);
        TalonSRX nwDriveMotor = driveMotorCreator.CreateDriveMotor(nwDriveChannel);
        TalonSRX[] driveMotors = new TalonSRX[] {neDriveMotor, seDriveMotor, swDriveMotor, nwDriveMotor};


        class TurnMotorCreator { TalonSRX CreateTurnMotor(int pTurnChannel) {             
            TalonSRX returnValue = new TalonSRX(pTurnChannel);
            returnValue.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 2));
            returnValue.setInverted(true);    
            returnValue.setNeutralMode(NeutralMode.Brake);
            //Probably need to set feedback device here somehow?  I forget how this works
            return returnValue;
        }}

        //Turn Spak Maxes        
        TurnMotorCreator turnMotorCreator = new TurnMotorCreator();        
        TalonSRX neTurnMotor = turnMotorCreator.CreateTurnMotor(neTurnChannel);
        TalonSRX seTurnMotor = turnMotorCreator.CreateTurnMotor(seTurnChannel);
        TalonSRX swTurnMotor = turnMotorCreator.CreateTurnMotor(swTurnChannel);
        TalonSRX nwTurnMotor = turnMotorCreator.CreateTurnMotor(nwTurnChannel);        
        TalonSRX[] turnMotors = new TalonSRX[] { neTurnMotor,seTurnMotor, swTurnMotor, nwTurnMotor};

        double[] turnEncoderOffsets = new double[4];
        turnEncoderOffsets[0] = 2873;
        turnEncoderOffsets[1] = 2584;
        turnEncoderOffsets[2] = 3422;
        turnEncoderOffsets[3] = 1883;

        Wheel[] wheels = new Wheel[4];

      
        for(int i = 0; i < 4; i++)
        {
            wheels[i] = new Wheel(WheelLabel.FromInt(i), 
                ((i == 0 || i == 1) ? 1.0 : -1.0) * WHEELS_LEFT_RIGHT_FROM_CENTER,
                ((i == 0 || i == 3) ? 1.0 : -1.0) * WHEELS_FRONT_BACK_FROM_CENTER,                
            new TalonSRXOpenLoopTranslationSystem(driveMotors[i], 20000, DRIVE_GEAR_RATIO, WHEEL_DIAMETER),
            new TalonSRXAndAbsoluteEncoderRotationSystem(turnMotors[i], new PIDFConfiguration(0.6, 0.0002, 0, 0), turnEncoderOffsets[i], true));
        }        
        SwerveDrive returnValue = new SwerveDrive(wheels[0], wheels[1], wheels[2], wheels[3], new NavX(true, NavX.Axis.Yaw), MAX_LINEAR_SPEED, MAX_ROTATIONAL_SPEED, NUDGING_SPEED);
      

       return returnValue;

    }

    
}
