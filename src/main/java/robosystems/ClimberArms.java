package robosystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class ClimberArms {
    private CANSparkMax mLeftDrumMotor;
    private CANSparkMax mRightDrumMotor;
    private RelativeEncoder mLeftEncoder;
    private RelativeEncoder mRightEncoder;

    private final int LEFT_CAN_ID = 0;
    private final int RIGHT_CAN_ID = 1;

    private double EXTEND_TARGET_POSITION = 0;
    private double RETRACT_TARGET_POSITION = 0;
   
    // Constructor
    public ClimberArms() {
        CANSparkMax mLeftDrumMotor = new CANSparkMax(LEFT_CAN_ID, MotorType.kBrushless);
        CANSparkMax mRightDrumMotor = new CANSparkMax(RIGHT_CAN_ID, MotorType.kBrushless);

        mLeftEncoder = mLeftDrumMotor.getEncoder();
        mRightEncoder = mRightDrumMotor.getEncoder();

        configureSparkMax(mLeftDrumMotor);
        configureSparkMax(mRightDrumMotor);
    }

    // Configure Drum Motors
    public static void configureSparkMax(CANSparkMax pDrumMotor) {
        pDrumMotor.restoreFactoryDefaults(); 
        pDrumMotor.setSmartCurrentLimit(20);
        //Various other configs
    }

    // Extends arms until both reach EXTEND_TARGET_POSITION
    public void extend() {
        if (mLeftEncoder.getPosition() < EXTEND_TARGET_POSITION && mRightEncoder.getPosition() < EXTEND_TARGET_POSITION){
          mLeftDrumMotor.set(0.1);
          mRightDrumMotor.set(0.1);} 
        else {
          mLeftDrumMotor.set(0);
          mRightDrumMotor.set(0);}
    }

    // Retracts arms until both reach RETRACT_TARGET_POSITION
    public void retract() {
        if (mLeftEncoder.getPosition() > RETRACT_TARGET_POSITION && mRightEncoder.getPosition() > RETRACT_TARGET_POSITION) {
            mLeftDrumMotor.set(-0.1);
            mRightDrumMotor.set(-0.1);
        } else {
            mLeftDrumMotor.set(0);
            mRightDrumMotor.set(0);
        }
    }
}