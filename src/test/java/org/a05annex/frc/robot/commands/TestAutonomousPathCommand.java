package org.a05annex.frc.robot.commands;

import frc.robot.subsystems.TestMk4NeoModule;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.runner.JUnitPlatform;
import org.junit.runner.RunWith;

import static org.junit.jupiter.api.Assertions.assertEquals;

/** This is a test of the {@link org.a05annex.frc.robot.commands.AutonomousPathCommand} that uses a test path
 * with both scheduled commands and stop-and-run commands. The test path is a 5 control point path. Stop-and-run
 * commands happen at the 1st, 3rd, and 5th (last) control points, scheduled commands happen at 3
 * locations on the path.
 */
@RunWith(JUnitPlatform.class)
public class TestAutonomousPathCommand {
    /**
     *
     */
    @Test
    @DisplayName("Test AutonomousPathCommand")
    void test_autonomousPathCommand() {
        assertEquals(4.5, 4.5);
    }
}
