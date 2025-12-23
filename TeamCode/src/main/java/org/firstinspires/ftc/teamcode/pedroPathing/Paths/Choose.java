package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auto Selector")
public class Choose {
    public enum Alliance { RED, BLUE, NONE }
    public Alliance selectedAlliance = Alliance.NONE;
    private boolean allianceConfirmed = false;

    public enum Auto { FAR, CLOSE, NONE }
    private Auto selectedAuto = Auto.NONE;
    private boolean autoConfirmed = false;

    private int mark = 0;
    private boolean numConfirmed = false;

    // Button debouncing variables - CRITICAL for proper button handling
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean aPressed = false;

    public void initPrint() {
        telemetry.addLine("Autonomous Selector Initialized");
        telemetry.addLine("Ready to configure auto routine");
        telemetry.update();
    }

    public void init_loop() {
        if (!autoConfirmed) {
            handleAutoSelection();
            displayAutoSelectionMenu();
        }

        else if (!numConfirmed) {
            handleAutoNum();
            displayNumSelectionMenu();
        }

        else if (!allianceConfirmed) {
            handleAllianceSelection();
            displayAllianceSelectionMenu();
        }
        // STEP 4: Ready to start
        else {
            displayReadyScreen();
        }

        telemetry.update();
    }

    private void handleAutoSelection() {
        // Dpad Up - Select FAR
        if (gamepad1.dpad_up && !dpadUpPressed) {
            selectedAuto = Auto.FAR;
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        // Dpad Down - Select CLOSE
        if (gamepad1.dpad_down && !dpadDownPressed) {
            selectedAuto = Auto.CLOSE;
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // A button - Confirm selection
        if (gamepad1.a && !aPressed && selectedAuto != Auto.NONE) {
            autoConfirmed = true;
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }
    }

    private void handleAutoNum() {
        // Dpad Down - Decrease trips
        if (gamepad1.dpad_down && !dpadDownPressed) {
            if (mark > 0) {
                mark -= 1;
            }
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // Dpad Up - Increase trips
        if (gamepad1.dpad_up && !dpadUpPressed) {
            if (mark < 5) {
                mark += 1;
            }
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        // A button - Confirm selection
        if (gamepad1.a && !aPressed) {
            numConfirmed = true;
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }
    }

    private void handleAllianceSelection() {
        // Dpad Up - Select RED
        if (gamepad1.dpad_up && !dpadUpPressed) {
            selectedAlliance = Alliance.RED;
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        // Dpad Down - Select BLUE
        if (gamepad1.dpad_down && !dpadDownPressed) {
            selectedAlliance = Alliance.BLUE;
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // A button - Confirm selection
        if (gamepad1.a && !aPressed && selectedAlliance != Alliance.NONE) {
            allianceConfirmed = true;
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }
    }

    private void displayAutoSelectionMenu() {
        telemetry.addLine("STEP 1: SELECT AUTO TYPE");
        telemetry.addLine("=================================");
        telemetry.addLine("");
        telemetry.addLine((selectedAuto == Auto.FAR ? ">>> FAR AUTO <<<" : "    Far Auto"));
        telemetry.addLine((selectedAuto == Auto.CLOSE ? ">>> CLOSE AUTO <<<" : "    Close Auto"));
        telemetry.addLine("");
        telemetry.addLine("---------------------------------");
        telemetry.addData("Current Selection", selectedAuto);
        telemetry.addData("Confirmed", autoConfirmed ? "YES ✓" : "NO");
        telemetry.addLine("---------------------------------");

        if (selectedAuto != Auto.NONE && !autoConfirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press A to confirm and continue");
        }
    }

    private void displayNumSelectionMenu() {
        telemetry.addLine("=================================");
        telemetry.addLine("NUMBER OF TRIPS");
        telemetry.addLine("=================================");
        telemetry.addLine("");
        telemetry.addLine("Use D-Pad Up/Down to adjust");
        telemetry.addLine("");
        telemetry.addData(">>> Trips", mark + " <<<");
        telemetry.addLine("");
        telemetry.addLine("---------------------------------");
        telemetry.addData("Confirmed", numConfirmed ? "YES ✓" : "NO");
        telemetry.addLine("---------------------------------");

        if (!numConfirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press A to confirm and continue");
        }
    }

    private void displayAllianceSelectionMenu() {
        telemetry.addLine("=================================");
        telemetry.addLine("STEP 3: SELECT ALLIANCE");
        telemetry.addLine("=================================");
        telemetry.addLine("");
        telemetry.addLine((selectedAlliance == Alliance.RED ? ">>> RED ALLIANCE <<<" : "    Red Alliance"));
        telemetry.addLine((selectedAlliance == Alliance.BLUE ? ">>> BLUE ALLIANCE <<<" : "    Blue Alliance"));
        telemetry.addLine("");
        telemetry.addLine("---------------------------------");
        telemetry.addData("Current Selection", selectedAlliance);
        telemetry.addData("Confirmed", allianceConfirmed ? "YES ✓" : "NO");
        telemetry.addLine("---------------------------------");

        if (selectedAlliance != Alliance.NONE && !allianceConfirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press A to confirm selection");
        }
    }

    private void displayReadyScreen() {
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        telemetry.addData("Auto Type", selectedAuto);
        telemetry.addData("Number of Trips", mark);
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addLine("");
    }

    public void startPrint() {
        // Provide warnings if selections weren't made
        if (selectedAuto == Auto.NONE) {
            telemetry.addLine("WARNING: No auto version selected!");
            telemetry.addLine("Defaulting to CLOSE auto");
            selectedAuto = Auto.CLOSE;
        }

        if (selectedAlliance == Alliance.NONE) {
            telemetry.addLine("WARNING: No alliance selected!");
            telemetry.addLine("Defaulting to RED alliance");
            selectedAlliance = Alliance.RED;
        }

        telemetry.addLine("");
        telemetry.addData("Starting Auto Type", selectedAuto);
        telemetry.addData("Starting with Trips", mark);
        telemetry.addData("Starting Alliance", selectedAlliance);
        telemetry.update();
    }

    public void stopPrint() {
        telemetry.addLine("Auto selector stopped");
        telemetry.update();
    }

    public Auto getSelectedAuto() {
        return selectedAuto;
    }

    public Alliance getSelectedAlliance() {
        return selectedAlliance;
    }

    public int getMark() {
        return mark;
    }
}