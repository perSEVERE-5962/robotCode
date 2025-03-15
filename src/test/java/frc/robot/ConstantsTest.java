package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import frc.robot.Constants.ScoringConstants;

public class ConstantsTest {

  @Test
  public void testScorePositionConstants() {
    double[][] positions = ScoringConstants.postions;
    int kL1 = ScoringConstants.kL1;
    int kL2 = ScoringConstants.kL2;
    int kL3 = ScoringConstants.kL3;
    int kL4 = ScoringConstants.kL4;
    int kStation = ScoringConstants.kStation;
    int kPivot = ScoringConstants.kPivot;
    int kWrist = ScoringConstants.kWrist;
    int kReach = ScoringConstants.kReach;
    

    // assertEquals(0.0, positions[kL1][kReach]);
    // assertEquals(-25.0, positions[kL1][kWrist]);
    // assertEquals(0.69, positions[kL1][kPivot]);

    // assertEquals(0, positions[kL2][kReach]);
    // assertEquals(-25.0, positions[kL2][kWrist]);
    // assertEquals(0.69, positions[kL2][kPivot]);

    // assertEquals(0, positions[kL3][kReach]);
    // assertEquals(0, positions[kL3][kWrist]);
    // assertEquals(0.69, positions[kL3][kPivot]);

    // assertEquals(0, positions[kL4][kReach]);
    // assertEquals(0, positions[kL4][kWrist]);
    // assertEquals(0.69, positions[kL4][kPivot]);

    // assertEquals(-13.1, positions[kStation][kReach]);
    // assertEquals(-11.74, positions[kStation][kWrist]);
    // assertEquals(0.836, positions[kStation][kPivot]);
  }
}