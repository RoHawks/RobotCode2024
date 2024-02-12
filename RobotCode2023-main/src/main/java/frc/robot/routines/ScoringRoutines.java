package frc.robot.routines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.arm.LowerJointConstants;
import frc.robot.arm.UpperJointConstants;
import states.GamePieceLocation;
import states.GamePieces;
import states.ScoringInstructions;

public class ScoringRoutines 
{

    private Routine mHighCone;
    private Routine mMidCone;
    private Routine mLowConeOrCube;
    private Routine mHighCube;
    private Routine mMidCube;

    private Routine mHighConeAuto;
    private Routine mMidConeAuto;
    private Routine mLowConeOrCubeAuto;
    private Routine mHighCubeAuto;
    private Routine mMidCubeAuto;

    public void CreateRoutines(RoutineFactory pRoutineFactory)
    {
        //mHighCone = pRoutineFactory.CreateSmoothRoutineParameterized(true, LowerJointConstants.TOP_CONE, UpperJointConstants.TOP_CONE);
        //mMidCone = pRoutineFactory.CreateSmoothRoutineParameterized(true, LowerJointConstants.MID_CONE, UpperJointConstants.MID_CONE);
        mHighCone = pRoutineFactory.CreateConeOrCubeRoutineNewStyle(true, LowerJointConstants.TOP_CONE, UpperJointConstants.TOP_CONE, true);
        mMidCone = pRoutineFactory.CreateConeOrCubeRoutineNewStyle(true, LowerJointConstants.MID_CONE, UpperJointConstants.MID_CONE, true);

        //These are working well as of Monday morning:
        //mHighCube = pRoutineFactory.CreateCubeOnlyRoutineNewStyle(true, LowerJointConstants.TOP_CUBE, UpperJointConstants.TOP_CUBE, false);
        //mMidCube = pRoutineFactory.CreateCubeOnlyRoutineNewStyle(true, LowerJointConstants.MID_CUBE, UpperJointConstants.MID_CUBE, false);
        mHighCube = pRoutineFactory.CreateConeOrCubeRoutineNewStyle(true, LowerJointConstants.TOP_CUBE, UpperJointConstants.TOP_CUBE, false);
        mMidCube = pRoutineFactory.CreateConeOrCubeRoutineNewStyle(true, LowerJointConstants.MID_CUBE, UpperJointConstants.MID_CUBE, false);
        

        mLowConeOrCube = pRoutineFactory.CreateConeOrCubeRoutineNewStyle(true, LowerJointConstants.LOW_GOAL, UpperJointConstants.LOW_GOAL, true);
       
        //mHighConeAuto = pRoutineFactory.CreateSmoothRoutineParameterized(false, LowerJointConstants.TOP_CONE, UpperJointConstants.TOP_CONE);
        mHighConeAuto = pRoutineFactory.CreateConeOrCubeRoutineNewStyle(false, LowerJointConstants.TOP_CONE, UpperJointConstants.TOP_CONE, true);
        mMidConeAuto = pRoutineFactory.CreateConeOrCubeRoutineNewStyle(false, LowerJointConstants.MID_CONE, UpperJointConstants.MID_CONE, true);
        mHighCubeAuto = pRoutineFactory.CreateConeOrCubeRoutineNewStyle(false, LowerJointConstants.TOP_CUBE, UpperJointConstants.TOP_CUBE, false);
        mMidCubeAuto = pRoutineFactory.CreateConeOrCubeRoutineNewStyle(false, LowerJointConstants.MID_CUBE, UpperJointConstants.MID_CUBE, false);
        
        mLowConeOrCubeAuto = pRoutineFactory.CreateConeOrCubeRoutineNewStyle(false, LowerJointConstants.LOW_GOAL, UpperJointConstants.LOW_GOAL, true);
       
    }

    private ScoringRoutines()
    {
        
    }



    private static ScoringRoutines mInstance = new ScoringRoutines();

    public static ScoringRoutines GetInstance()
    {
        return mInstance;
    }
    
    public Routine GetRoutineForInstructions(ScoringInstructions pInstructions)
    {
        return GetRoutineForInstructions(pInstructions, false);
    }

    public Routine GetRoutineForInstructions(ScoringInstructions pInstructions, boolean pIsAuto)
    {
        if(pInstructions.GetGamePiece() == GamePieces.CONE)
        {
            if(pInstructions.GetGamePieceLocation() == GamePieceLocation.HIGH)
            {
                return pIsAuto ? mHighConeAuto : mHighCone;
            }
            else if(pInstructions.GetGamePieceLocation() == GamePieceLocation.MID)
            {
                return pIsAuto ? mMidConeAuto : mMidCone;
            }
            else
            {
                return pIsAuto ? mLowConeOrCubeAuto : mLowConeOrCube;
            }

        }
        else //Cube
        {
            if(pInstructions.GetGamePieceLocation() == GamePieceLocation.HIGH)
            {
                return pIsAuto ? mHighCubeAuto : mHighCube;
            }
            else if(pInstructions.GetGamePieceLocation() == GamePieceLocation.MID)
            {
                return pIsAuto ? mMidCubeAuto : mMidCube;
            }
            else
            {
                return pIsAuto ? mLowConeOrCubeAuto : mLowConeOrCube;
            }
        }
    }
}
