package org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Choose {
    protected Gamepad gamepad1;
    protected Telemetry telemetry;
    public enum Alliance { RED, BLUE, NONE }
    public Alliance selectedAlliance = Alliance.RED;
    private boolean allianceConfirmed = false;

    public enum Choices {
        RED, BLUE
    }

    private int mark = 5;
    private boolean numConfirmed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean aPressed = false;

    private boolean wolfpack = false;
    private boolean wolfpackConfirmed = false;
    private boolean fill = false;
    private boolean fillConfirmed = false;

    private boolean thirdSpike = false;
    private boolean spikeConfirmed = false;

    public Choose(Gamepad g1, Telemetry t) {
        telemetry = t;
        gamepad1 = g1;
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

    int returnNum = 0;

    private int handleAutoNum(int lowerLimit, int upperLimit) {
        if(returnNum < lowerLimit){
            returnNum = lowerLimit;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            if (returnNum > lowerLimit) {
                returnNum -= 1;
            }
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        if (gamepad1.dpad_up && !dpadUpPressed) {
            if (returnNum < upperLimit) {
                returnNum += 1;
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

        return returnNum;
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

    boolean trueFalse = false;
    private boolean dPadUpDown() {
        if (gamepad1.dpad_up && !dpadUpPressed) {
            trueFalse = true;
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            trueFalse = false;
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
        return trueFalse;
    }
    private void displayUpDownMenu(String title, String title2) {
        telemetry.addLine("=================================");
        telemetry.addLine(title);
        telemetry.addLine("=================================");
        telemetry.addLine("");
        telemetry.addLine("Use D-Pad Up/Down to adjust");
        telemetry.addData(title2 + " :", mark);
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
    private void displayFillSelectionMenu() {
        telemetry.addLine("=================================");
        telemetry.addLine("FILL CLASSIFIER");
        telemetry.addLine("=================================");
        telemetry.addLine("");
        telemetry.addLine("Use D-Pad Up/Down to adjust");
        telemetry.addData("Fill:", fill);
        telemetry.addLine("");
        telemetry.addLine("---------------------------------");
        telemetry.addData("Current Selection", fill);
        telemetry.addData("Confirmed", fillConfirmed ? "YES" : "NO");
        telemetry.addLine("---------------------------------");

        if (!fillConfirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press X to confirm selection");
        }
    }

    private void displaySpikeSelectionMenu() {
        telemetry.addLine("=================================");
        telemetry.addLine("THIRD SPIKE");
        telemetry.addLine("=================================");
        telemetry.addLine("");
        telemetry.addLine("Use D-Pad Up/Down to adjust");
        telemetry.addData("Third Spike:", thirdSpike);
        telemetry.addLine("");
        telemetry.addLine("---------------------------------");
        telemetry.addData("Current Selection", thirdSpike);
        telemetry.addData("Confirmed", spikeConfirmed ? "YES" : "NO");
        telemetry.addLine("---------------------------------");

        if (!spikeConfirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press X to confirm selection");
        }
    }

    public void displayReady(Choices choices){
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        telemetry.addData("Alliance", choices);
        telemetry.addLine("");
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

    public void displayReadyCloseScreen2() {
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Number of Trips", mark);
        telemetry.addLine("");
    }

    public void displayReadyCloseScreen3() {
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Fill Clasifier", fill);
        telemetry.addLine("");
    }

    public void displayReadyCloseScreen4() {
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("3rd Spike", fill);
        telemetry.addLine("");
    }

    public Alliance getSelectedAlliance() { return selectedAlliance; }
    public Boolean getSelectedWolfpack() { return wolfpack; }
    public Boolean getFill() { return fill; }
    public Boolean getSpike() { return thirdSpike; }

    public int getMark() {
        return mark;
    }
}