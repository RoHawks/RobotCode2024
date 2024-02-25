package robosystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class ClimberArms {
    private CANSparkMax mLeftDrumMotor;
    private CANSparkMax mRightDrumMotor;
    private RelativeEncoder mLeftEncoder;
    private RelativeEncoder mRightEncoder;

    private final int LEFT_CAN_ID = 11;
    private final int RIGHT_CAN_ID = 12;

    private double EXTEND_TARGET_POSITION = 0;
    private double RETRACT_TARGET_POSITION = 0;
    private double BASE_SPEED;
   
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

        double leftSpeed = BASE_SPEED;
        double rightSpeed = BASE_SPEED;
        if (mLeftEncoder.getPosition() > EXTEND_TARGET_POSITION * 0.75)
        {
            leftSpeed = BASE_SPEED * 0.25;
        }
        if (mRightEncoder.getPosition() > EXTEND_TARGET_POSITION * 0.75)
        {
            leftSpeed = BASE_SPEED * 0.25;
        }


        if (mLeftEncoder.getPosition() < EXTEND_TARGET_POSITION)
        {
          mLeftDrumMotor.set(leftSpeed);
        }
        else 
        {
          mLeftDrumMotor.set(0);
        }

        if (mRightEncoder.getPosition() < EXTEND_TARGET_POSITION)
        {
          mRightDrumMotor.set(rightSpeed);
        }
        else 
        {
          mRightDrumMotor.set(0);
        }
    }

    // Retracts arms until both reach RETRACT_TARGET_POSITION
    public void retract() {
        double leftSpeed = BASE_SPEED;
        double rightSpeed = BASE_SPEED;
        if (mLeftEncoder.getPosition() < 
            (EXTEND_TARGET_POSITION - RETRACT_TARGET_POSITION) * 0.25 + RETRACT_TARGET_POSITION)
        {
            leftSpeed = BASE_SPEED * 0.25;
        }
        if (mRightEncoder.getPosition() < 
            (EXTEND_TARGET_POSITION - RETRACT_TARGET_POSITION) * 0.25 + RETRACT_TARGET_POSITION)
        {
            leftSpeed = BASE_SPEED * 0.25;
        }


        if (mLeftEncoder.getPosition() > RETRACT_TARGET_POSITION)
        {
          mLeftDrumMotor.set(-leftSpeed);
        }
        else 
        {
          mLeftDrumMotor.set(0);
        }

        if (mRightEncoder.getPosition() > RETRACT_TARGET_POSITION)
        {
          mRightDrumMotor.set(-rightSpeed);
        }
        else 
        {
          mRightDrumMotor.set(0);
        }
    }
}