package universalSwerve.components.implementations;

import javax.swing.text.html.FormSubmitEvent.MethodType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import universalSwerve.components.ITranslationSystem;
import universalSwerve.utilities.Conversions;
import universalSwerve.utilities.PIDFConfiguration;

public class TalonFXTranslationSystem implements ITranslationSystem{

    private TalonFX mFalcon;
    private double mWheelDiameter;
    private double mGearRatio;


    private final VelocityVoltage mVoltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
  


    /*
        Diameter is in inches
        Gear ratio is a fraction, so a 10:1 should be passed in as 0.1
    */
    public TalonFXTranslationSystem(TalonFX pFalcon, PIDFConfiguration pPidfConfiguration, double pWheelDiameter, double pGearRatio)
    {
        mFalcon = pFalcon;
        InitializeFalconPID(pPidfConfiguration);
        mWheelDiameter = pWheelDiameter;
        mGearRatio = pGearRatio;
    }

    @Override
    public void Initialize() {


        
    }
    public void ResetDistanceTravelled()
    {
        ResetDriveEncodersToZero();
    }

    public void ResetDriveEncodersToZero()
	{
        throw new RuntimeException("ResetDriveEncodersToZero not implemented yet for TalonFXTranslationSystem");        
	}

	private double ENCODER_TICKS_PER_REVOLUTION_OF_MOTOR = 1;
	
	private double ConvertDriveEncoderToDistanceTravelled(double pEncoderTicks)
	{
		double numberOfMotorRotations = pEncoderTicks / ENCODER_TICKS_PER_REVOLUTION_OF_MOTOR ;
		double numberOfWheelRotations = numberOfMotorRotations  * mGearRatio;
		double distanceTravelled = Math.abs(numberOfWheelRotations * Math.PI * mWheelDiameter);
		return  distanceTravelled;
		

	}

	public double GetDistanceTravelled()
	{
		return ConvertDriveEncoderToDistanceTravelled(this.mFalcon.getRotorPosition().getValue());
	}


    @Override
    public void SetVelocity(double pVelocity) {
        //pVelocity is in inches per second
        //Let's convert that to RPMS
        //SmartDashboard.putNumber("VelocityInchesPerSecond", pVelocity);
        double rpms = Conversions.InchesPerSecondToRPMS(pVelocity, this.mWheelDiameter);        
        //SmartDashboard.putNumber("rpms", rpms);
        double rpmsAtMotor = rpms / mGearRatio;
        //SmartDashboard.putNumber("rpmsAtMotor", rpmsAtMotor);
        double targetVelocity = Conversions.RPMsToTalonFXVelocityUnit(rpmsAtMotor);
        //SmartDashboard.putNumber("TargetVelocity", targetVelocity);
        mFalcon.setControl(mVoltageVelocity.withVelocity(-1.0 * targetVelocity));//why do we need a negative here?  It seems like none of the invert options work....
        
    }

    public double GetVelocity()
    {
        
        double motorRPMs = Conversions.TalonFXVelocityUnitToRPMS(mFalcon.getVelocity().getValue());        
        double wheelRPMs = motorRPMs * mGearRatio;        
        return Conversions.RPMstoInchesPerSecond(wheelRPMs, mWheelDiameter);        
    }

    public void SetToBreakMode()
    {
        //to do, you probably need to do something like this:
        /*
        MotorOutputConfigs driveMotorOutputConfig = new MotorOutputConfigs();
        driveMotorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
        driveMotorOutputConfig.NeutralMode = NeutralModeValue.Coast;  
        */
        //But it seems like you can only set all of these parameters together, so make sure not to undo anything.
        throw new RuntimeException("SetToBreakMode not implemented for TalonFXTranslationSystem");
    }
    public void SetToCoastMode()
    {
        //See SetToBreakMode
        throw new RuntimeException("SetToCoastMode not implemented for TalonFXTranslationSystem");
    }
    
    private void InitializeFalconPID(PIDFConfiguration pPidfConfiguration)
    {
        TalonFXConfiguration pidConfiguration = new TalonFXConfiguration();
        pidConfiguration.Slot0.kP = pPidfConfiguration.P();
        pidConfiguration.Slot0.kP = pPidfConfiguration.I();
        pidConfiguration.Slot0.kP = pPidfConfiguration.D();
        pidConfiguration.Slot0.kV = pPidfConfiguration.F();
        mFalcon.getConfigurator().apply(pidConfiguration);
    }

    public void StopEverything()
    {
        mFalcon.stopMotor();
    }
  
    public double GetPercentOutput()
    {
        return mFalcon.get();
    }
}