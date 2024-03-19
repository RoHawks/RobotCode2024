package universalSwerve.components.implementations;

import java.util.Arrays;

import javax.swing.text.html.FormSubmitEvent.MethodType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
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


    private final VelocityVoltage mVoltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, true, false, false);
    private final VelocityDutyCycle mVelocityDutyCycle = new VelocityDutyCycle(0, 0, true, 0, 0, true, false, false);
  


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



    private double mRotorValueToExcludeFromReset = 0;
    public void ResetDriveEncodersToZero()
	{
        mRotorValueToExcludeFromReset = this.mFalcon.getRotorPosition().getValue() ;
	}

	private double ENCODER_TICKS_PER_REVOLUTION_OF_MOTOR = 1;
	
	private double ConvertDriveEncoderToDistanceTravelled(double pEncoderTicks)
	{
		double numberOfMotorRotations = pEncoderTicks / ENCODER_TICKS_PER_REVOLUTION_OF_MOTOR ;
		double numberOfWheelRotations = numberOfMotorRotations  * mGearRatio;
        //ATS 3/2/2024
        //Gromble, gromble, gromble.
        //This ABS really shouldn't be here
        //I suspect it was in here from previous, simple autnomouses where we just pointed the wheels in a direction and drove straight
        //But this doesn't work well when combined with the SwerveDriveOdometry
        //Becase what happens is that sometimes we point the wheels 180 degrees from where they are supposed to be
        //And run the motor backwards
        //By doing the ABS here it looks like we are running the motor forwards
        //And as a result the odometry thinks that this wheel is doing the opposte of what it actualy is
		//double distanceTravelled = Math.abs(numberOfWheelRotations * Math.PI * mWheelDiameter);
        double distanceTravelled = numberOfWheelRotations * Math.PI * mWheelDiameter;
		return  distanceTravelled;
		

	}

	public double GetDistanceTravelled()
	{        
        

		return ConvertDriveEncoderToDistanceTravelled(this.mFalcon.getRotorPosition().getValue() - mRotorValueToExcludeFromReset);
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
    
        
        StatusCode sc = mFalcon.setControl(mVoltageVelocity.withVelocity(-1.0 * targetVelocity));//why do we need a negative here?  It seems like none of the invert options work....
        //StatusCode sc = mFalcon.setControl(mVelocityDutyCycle.withVelocity(-1.0 * targetVelocity));//why do we need a negative here?  It seems like none of the invert options work....
        
        //SmartDashboard.putString("SetControl cs", sc.toString());
        
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
        //pidConfiguration.Slot0.kS = pPidfConfiguration.S();
        //SmartDashboard.putNumber("KV", pPidfConfiguration.F());
        StatusCode statusCode = mFalcon.getConfigurator().apply(pidConfiguration);
        //SmartDashboard.putString("StatusCode", statusCode.toString());
        
    }

    public void StopEverything()
    {
        //SmartDashboard.putNumber("StopEcverythingCalledAt", System.currentTimeMillis());
        mFalcon.stopMotor();
    }
  
    public double GetPercentOutput()
    {
        return mFalcon.get();
    }

    public java.util.List<TalonFX> GetSpeakers()
    {
        return Arrays.asList(mFalcon);
    }

    public double GetCurrentDraw()
    {
        return mFalcon.getSupplyCurrent().getValueAsDouble();
    }
}
