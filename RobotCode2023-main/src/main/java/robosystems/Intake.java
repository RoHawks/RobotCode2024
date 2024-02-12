package robosystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*; //import static class representing the value of the DoubleSolenoid

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import states.JoystickControls;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
// import com.revrobotics.CANSparkMax.SparkMaxLimitSwitch.Type;

public class Intake {   

    private CANSparkMax mIntakeMotorTop;
    private CANSparkMax mIntakeMotorBottom;
    private DoubleSolenoid mIntakeDeploy;
    private boolean mPreviousCheckWasStalled = false;
    private long mStallBeginTime = 0;
    private JoystickControls mControls;
    private AddressableLED mLEDs;
    private AddressableLEDBuffer mLEDBuffer;

    public enum IntakeMode
    {
        NormalIntaking,
        Holding,
        Ejecting,
        GracefulEjecting
    }

    private IntakeMode mIntakeMode;

    public Intake(CANSparkMax pIntakeMotorTop, CANSparkMax pIntakeMotorBottom, DoubleSolenoid pIntakeDeploy, AddressableLED pLEDs, AddressableLEDBuffer pLEDBuffer) 
    {
        mIntakeMotorTop = pIntakeMotorTop;
        mIntakeMotorBottom = pIntakeMotorBottom;
        mIntakeDeploy = pIntakeDeploy;
        mIntakeMode = IntakeMode.Holding;
        mControls = new JoystickControls(new XboxController(0), new XboxController(1));
        mLEDs = pLEDs;
        mLEDBuffer = pLEDBuffer;

        configureSparkMax(mIntakeMotorTop);
        configureSparkMax(mIntakeMotorBottom);        
    }



    public static void configureSparkMax(CANSparkMax pSparkMax){
        pSparkMax.restoreFactoryDefaults(); 
        pSparkMax.setSmartCurrentLimit(20);    
        pSparkMax.setOpenLoopRampRate(0.5);
        pSparkMax.burnFlash();
    }


    private boolean IsMotorStalled(CANSparkMax pMotor)
    {
        return Math.abs(pMotor.getEncoder().getVelocity()) < 100
            && pMotor.getOutputCurrent() > 10;

    }

    private boolean CheckStall()
    {
        return IsMotorStalled(mIntakeMotorTop)
        || IsMotorStalled(mIntakeMotorBottom);
            
    }

    long mModeEntryTime = 0;
    public void SetMode(IntakeMode pIntakeMode)
    {
        if(pIntakeMode != mIntakeMode)
        {
            mModeEntryTime = System.currentTimeMillis();
            mIntakeMode = pIntakeMode;

            if(mIntakeMode == IntakeMode.NormalIntaking || mIntakeMode == IntakeMode.Holding )
            {
                this.mIntakeMotorBottom.setSmartCurrentLimit(20);
                this.mIntakeMotorTop.setSmartCurrentLimit(20);
            }
            else //ejecting
            {
                this.mIntakeMotorBottom.setSmartCurrentLimit(40);
                this.mIntakeMotorTop.setSmartCurrentLimit(40);
                
            }
        }
    }
    
    private long mControllerVibratingTime = 1000; // ms
    private void VibrateIfJustTookInLowPiece()
    {
        if (mIntakeMode == IntakeMode.Holding && (System.currentTimeMillis() - mVibrateEntryTime) < mControllerVibratingTime)
        {
            mControls.TurnOnVibrate();
        }
        else
        {
            mControls.TurnOffVibrate();
        }
    }

    private void SetLEDColor(int pR, int pG, int pB)
    {
        for(int i = 0; i < mLEDBuffer.getLength(); i++)
        {
            mLEDBuffer.setRGB(i, pR, pG, pB);
        }
        mLEDs.setData(mLEDBuffer);
    }

    private void SetLEDStatus()
    {
        if(mIntakeMode == IntakeMode.NormalIntaking)
        {
            SetLEDColor(0,0,0);
        }
        else if(mIntakeMode == IntakeMode.Holding)
        {
            long timeSinceEntry = System.currentTimeMillis() - mVibrateEntryTime;
            if(timeSinceEntry < 250 || (timeSinceEntry > 500 & timeSinceEntry < 750))
            {
                SetLEDColor(255, 255, 255);
            }
            else if(timeSinceEntry < 1000)
            {
                SetLEDColor(0,0,0);
            }
            else
            {
                SetLEDColor(0,255,0);
            }                        
        } 
        else if(mIntakeMode == IntakeMode.GracefulEjecting)
        {
            
            if(GetTimeSinceEnteredCurrentMode() > GRACEFUL_EJECT_TIME_TO_MAX_SPEED)
            {
                long blinkTime = 400;
                boolean switchValue = (GetTimeSinceEnteredCurrentMode() % blinkTime)  > (blinkTime / 2);
                if(switchValue)
                {
                    SetLEDColor(0, 0, 255);
                }
                else
                {
                    SetLEDColor(0, 0, 0);
                }
            }
            else
            {
                double percentBrightness = ((double)GetTimeSinceEnteredCurrentMode())/ ((double)GRACEFUL_EJECT_TIME_TO_MAX_SPEED);
                if(percentBrightness > 1)
                {
                    percentBrightness = 1;
                }
                SetLEDColor(0, 0, (int)(percentBrightness * 255.0));
            }
        }            
        else //ejecting
        {
            SetLEDColor(255, 0, 0);
        }
    }


    private long GetTimeSinceEnteredCurrentMode()
    {
        return System.currentTimeMillis() - mModeEntryTime;
    }

    private long mVibrateEntryTime = 0;
    public void Run(boolean pIsLowGoalMode)
    {
        VibrateIfJustTookInLowPiece();
        SetLEDStatus();

        String intakeMode = "Intaking";
        if(mIntakeMode == IntakeMode.Holding)
        {            
            intakeMode = "Holding";
        }
        else if (mIntakeMode == IntakeMode.Ejecting)
        {
            intakeMode = "Ejecting";
        }
        else if (mIntakeMode == IntakeMode.GracefulEjecting)
        {
            intakeMode = "GracefulEjecting";
        }
        SmartDashboard.putString("IntakeMode", intakeMode);
        if(mIntakeMode == IntakeMode.NormalIntaking)
        {
            boolean isStalled = CheckStall();
            if(isStalled)
            {
                if(!mPreviousCheckWasStalled)
                {
                    mStallBeginTime = System.currentTimeMillis();
                    mPreviousCheckWasStalled  = true;
                }
                if(System.currentTimeMillis() - mStallBeginTime > 400) //ms, arbitrary check
                {
                    SetMode(IntakeMode.Holding);  
                    mVibrateEntryTime = System.currentTimeMillis();
                }

            }
            else
            {
                mPreviousCheckWasStalled = false;
            }

            turnOn(pIsLowGoalMode);
        }
        else if(mIntakeMode == IntakeMode.Holding)
        {
            turnOff();
        }
        else if(mIntakeMode == IntakeMode.GracefulEjecting)
        {
            gracefulEject();
        }
        else //mIntake is ejecting
        {
            eject();
        }
    }


    private void turnOn(boolean pIsLowGoalMode){ //need to actually supply power using the CANSparkMax before any of this can happen
       
        /*from nyc, adjusted for switched motor can IDs 
        mIntakeMotorBottom.set(-1.0);
        mIntakeMotorTop.set(-0.5);
        */
        //we may want to have different speeds here for normal intaking or low goal intaking
        if(pIsLowGoalMode)
        {
            mIntakeMotorBottom.set(-0.75);
            mIntakeMotorTop.set(-0.50);
        }
        else
        {
            mIntakeMotorBottom.set(-1.0);
            mIntakeMotorTop.set(-0.5);
        }
        
    }

    private void turnOff(){
        mIntakeMotorTop.set(0);
        mIntakeMotorBottom.set(0);
    }

    private void eject()
    {
        mIntakeMotorBottom.set(1.0);
        mIntakeMotorTop.set(1.0);
    }

    private long GRACEFUL_EJECT_TIME_TO_MAX_SPEED = 2000; //ms
    private double MAX_GRACEFUL_EJECT_SPEED = 1.0;
    private void gracefulEject()
    {
        double percentThrottle = ((double)GetTimeSinceEnteredCurrentMode())/ ((double)GRACEFUL_EJECT_TIME_TO_MAX_SPEED);
        if(percentThrottle > 1.0)
        {
            percentThrottle = 1.0;
        }
        percentThrottle *= MAX_GRACEFUL_EJECT_SPEED;
        mIntakeMotorBottom.set(percentThrottle);
        mIntakeMotorTop.set(percentThrottle);
        
    }


    public void deploy() {
        mIntakeDeploy.set(kForward);
    }
    public void retract() {
        mIntakeDeploy.set(kReverse);
    }
}
