package universalSwerve;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import universalSwerve.components.Wheel;
import universalSwerve.components.WheelLabel;
import universalSwerve.components.implementations.SparkMaxUtilizingAbsoluteEncoderRotationSystemContinuously;
import universalSwerve.components.implementations.TalonFXTranslationSystem;
import universalSwerve.hardware.implementations.Pigeon2Imu;
import universalSwerve.utilities.PIDFConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

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
        /* Townsend config */
        /*
        PIDFConfiguration neDriveFPIDCongiruation = new PIDFConfiguration(0.000, 0, 0,  0.12 * 13.5/14.15);
        PIDFConfiguration seDriveFPIDCongiruation = new PIDFConfiguration(0.000, 0, 0,  0.12 * 13.5/14.4);
        PIDFConfiguration swDriveFPIDCongiruation = new PIDFConfiguration(0.000, 0, 0,  0.12);
        PIDFConfiguration nwDriveFPIDCongiruation = new PIDFConfiguration(0.00, 0, 0,  0.12 * 13.5/13.9);
        */

        //SysID Config
        
        PIDFConfiguration neDriveFPIDCongiruation = new PIDFConfiguration(0.13478, 0, 0,  0.11939, 0.11044);//p = 0.12701, a = 0.027155
        PIDFConfiguration seDriveFPIDCongiruation = new PIDFConfiguration(0.10966, 0, 0,  0.12152, 0.2147);//p = 0.13894, a = 0.020701
        PIDFConfiguration swDriveFPIDCongiruation = new PIDFConfiguration(0.14748, 0, 0,  0.12003, 0.20957);//p 0.1199, a 0.0265
        PIDFConfiguration nwDriveFPIDCongiruation = new PIDFConfiguration(0.10088, 0, 0,  0.12085,  0.26914);//p 0.1219, a 0013223

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
        CANSparkMax neTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(neTurnChannel, 0.3454+0.500);
        CANSparkMax seTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(seTurnChannel, 0.2536-0.250);
        CANSparkMax swTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(swTurnChannel, 0.671);
        CANSparkMax nwTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(nwTurnChannel, 0.9414 -0.750);        
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


    
}
