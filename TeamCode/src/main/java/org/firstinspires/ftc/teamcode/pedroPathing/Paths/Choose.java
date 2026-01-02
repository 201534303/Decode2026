package org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Choose {
    protected Gamepad gamepad1;
    protected Telemetry telemetry;
    public enum Alliance { RED, BLUE, NONE }
    public Alliance selectedAlliance = Alliance.RED;
    private boolean allianceConfirmed = false;
    public enum Auto { FAR, CLOSE, NONE }
    private Auto selectedAuto = Auto.NONE;
    private boolean autoConfirmed = false;

    private int mark = 4;
    private boolean numConfirmed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean aPressed = false;

    private boolean wolfpack = false;
    private boolean wolfpackConfirmed = false;

    public Choose(Gamepad g1, Telemetry t) {
        telemetry = t;
        gamepad1 = g1;
    }

    public boolean tripsInit(){
         if (!numConfirmed) {
            handleAutoNum();
            displayNumSelectionMenu();
            return false;
        } else {
             displayReadyTripsScreen();
             return true;
         }
    }

    public boolean allianceInit(){
        if (!allianceConfirmed) {
            handleAllianceSelection();
            displayAllianceSelectionMenu();
            return false;
        } else {
            displayReadyTeleScreen();
            return true;
        }
    }

    public boolean wolfpackInit(){
        if (!wolfpackConfirmed) {
            handleWolfpackSelection();
            displayWolfpackSelectionMenu();
            return false;
        } else {
            displayReadyCloseScreen();
            return true;
        }
    }

    private void handleWolfpackSelection() {
        if (gamepad1.dpad_up && !dpadUpPressed) {
            wolfpack = true;
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            wolfpack = false;
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        if (gamepad1.a && !aPressed) {
            wolfpackConfirmed = true;
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }
    }

    private void handleAutoNum() {
        if (gamepad1.dpad_down && !dpadDownPressed) {
            if (mark > 0) {
                mark -= 1;
            }
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        if (gamepad1.dpad_up && !dpadUpPressed) {
            if (mark < 4) {
                mark += 1;
            }
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.a && !aPressed) {
            numConfirmed = true;
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }
    }

    private void handleAllianceSelection() {
        if (gamepad1.dpad_up && !dpadUpPressed) {
            selectedAlliance = Alliance.RED;
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            selectedAlliance = Alliance.BLUE;
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        if (gamepad1.a && !aPressed && selectedAlliance != Alliance.NONE) {
            allianceConfirmed = true;
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }
    }

    private void displayNumSelectionMenu() {
        telemetry.addLine("=================================");
        telemetry.addLine("NUMBER OF TRIPS");
        telemetry.addLine("=================================");
        telemetry.addLine("");
        telemetry.addLine("Use D-Pad Up/Down to adjust");
        telemetry.addData("Trips:", mark);
        telemetry.addLine("");
        telemetry.addLine("---------------------------------");
        telemetry.addData("Current Selection", mark);
        telemetry.addData("Confirmed", numConfirmed ? "YES" : "NO");
        telemetry.addLine("---------------------------------");

        if (!numConfirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press X to confirm");
        }
    }

    private void displayWolfpackSelectionMenu() {
        telemetry.addLine("=================================");
        telemetry.addLine("WOLFPACK AUTO");
        telemetry.addLine("=================================");
        telemetry.addLine("");
        telemetry.addLine("Use D-Pad Up/Down to adjust");
        telemetry.addData("Wolfpack:", wolfpack);
        telemetry.addLine("");
        telemetry.addLine("---------------------------------");
        telemetry.addData("Current Selection", wolfpack);
        telemetry.addData("Confirmed", wolfpackConfirmed ? "YES" : "NO");
        telemetry.addLine("---------------------------------");

        if (!numConfirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press X to confirm");
        }
    }

    private void displayAllianceSelectionMenu() {
        telemetry.addLine("=================================");
        telemetry.addLine("SELECT ALLIANCE");
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
            telemetry.addLine("Press X to confirm selection");
        }
    }

    private void displayReadyTeleScreen() {
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addLine("");
    }

    private void displayReadyTripsScreen() {
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        telemetry.addData("NUMBER OF TRIPS:", mark);
        telemetry.addLine("");
    }

    public void displayReadyCloseScreen() {
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Number of Trips", mark);
        telemetry.addData("Wolfpack", wolfpack);
        telemetry.addLine("");
    }

    public Alliance getSelectedAlliance() { return selectedAlliance; }
    public Boolean getSelectedWolfpack() { return wolfpack; }
    public int getMark() {
        return mark;
    }
}