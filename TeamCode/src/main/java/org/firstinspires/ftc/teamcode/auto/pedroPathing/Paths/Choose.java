package org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class Choose {
    protected Gamepad gamepad1;
    protected Telemetry telemetry;

    public enum Alliance { RED, BLUE, NONE }
    public enum Choices { RED, BLUE }

    // ---- Selected values ----
    public Alliance selectedAlliance = Alliance.RED;
    private int mark = 5;
    private boolean wolfpack = false;
    private boolean fill = false;
    private boolean thirdSpike = false;

    // ---- Confirmation flags ----
    private boolean allianceConfirmed = false;
    private boolean numConfirmed = false;
    private boolean wolfpackConfirmed = false;
    private boolean fillConfirmed = false;
    private boolean spikeConfirmed = false;

    // ---- Shared button edge-detection state (one set, reused by every screen) ----
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean aPressed = false;

    public Choose(Gamepad g1, Telemetry t) {
        telemetry = t;
        gamepad1 = g1;
    }

    // =====================================================================
    //  PUBLIC "INIT" METHODS - call once per loop() from init_loop().
    //  Each just plugs its own state/handler/display into the shared
    //  selectionInit() driver below instead of repeating the same
    //  if(!confirmed){...}else{...} block five times.
    // =====================================================================

    public boolean tripsInit() {
        return selectionInit(numConfirmed, this::handleAutoNum, this::displayNumSelectionMenu, this::displayReadyTripsScreen);
    }

    public boolean allianceInit() {
        return selectionInit(allianceConfirmed, this::handleAllianceSelection, this::displayAllianceSelectionMenu, this::displayReadyTeleScreen);
    }

    public boolean fillInit() {
        return selectionInit(fillConfirmed, this::handleFillSelection, this::displayFillSelectionMenu, this::displayReadyCloseScreen3);
    }

    public boolean spikeInit() {
        return selectionInit(spikeConfirmed, this::handleUpDownSelection, this::displaySpikeSelectionMenu, this::displayReadyCloseScreen4);
    }

    public boolean wolfpackInit() {
        return selectionInit(wolfpackConfirmed, this::handleWolfpackSelection, this::displayWolfpackSelectionMenu, this::displayReadyCloseScreen);
    }

    private boolean selectionInit(boolean confirmed, Runnable handle, Runnable display, Runnable displayReady) {
        if (!confirmed) {
            handle.run();
            display.run();
            return false;
        } else {
            displayReady.run();
            return true;
        }
    }

    // =====================================================================
    //  INPUT HANDLING
    //  One generic dpad-up / dpad-down / confirm handler with debounce,
    //  reused by every field instead of five near-identical copies.
    // =====================================================================

    private void handleSelection(Runnable onUp, Runnable onDown, BooleanSupplier canConfirm, Runnable onConfirm) {
        if (gamepad1.dpad_up && !dpadUpPressed) {
            onUp.run();
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            onDown.run();
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        if (gamepad1.a && !aPressed && canConfirm.getAsBoolean()) {
            onConfirm.run();
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }
    }

    private void handleAutoNum() {
        handleSelection(
                () -> { if (mark < 4) mark += 1; },
                () -> { if (mark > 0) mark -= 1; },
                () -> true,
                () -> numConfirmed = true
        );
    }

    private void handleAllianceSelection() {
        handleSelection(
                () -> selectedAlliance = Alliance.RED,
                () -> selectedAlliance = Alliance.BLUE,
                () -> selectedAlliance != Alliance.NONE,
                () -> allianceConfirmed = true
        );
    }

    private void handleWolfpackSelection() {
        handleSelection(
                () -> wolfpack = true,
                () -> wolfpack = false,
                () -> true,
                () -> wolfpackConfirmed = true
        );
    }

    private void handleFillSelection() {
        handleSelection(
                () -> fill = true,
                () -> fill = false,
                () -> true,
                () -> fillConfirmed = true
        );
    }

    private void handleUpDownSelection() {
        handleSelection(
                () -> thirdSpike = true,
                () -> thirdSpike = false,
                () -> true,
                () -> spikeConfirmed = true
        );
    }

    // =====================================================================
    //  TELEMETRY - SELECTION MENUS
    //  One generic menu renderer, reused by every field instead of four
    //  near-identical copies.
    // =====================================================================

    private void displaySelectionMenu(String title, String valueLabel, Object value, boolean confirmed) {
        telemetry.addLine("=================================");
        telemetry.addLine(title);
        telemetry.addLine("=================================");
        telemetry.addLine("");
        telemetry.addLine("Use D-Pad Up/Down to adjust");
        telemetry.addData(valueLabel, value);
        telemetry.addLine("");
        telemetry.addLine("---------------------------------");
        telemetry.addData("Current Selection", value);
        telemetry.addData("Confirmed", confirmed ? "YES" : "NO");
        telemetry.addLine("---------------------------------");

        if (!confirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press X to confirm selection");
        }
    }

    private void displayNumSelectionMenu() {
        displaySelectionMenu("NUMBER OF TRIPS", "Trips:", mark, numConfirmed);
    }

    private void displayWolfpackSelectionMenu() {
        displaySelectionMenu("WOLFPACK AUTO", "Wolfpack:", wolfpack, wolfpackConfirmed);
    }

    private void displayFillSelectionMenu() {
        displaySelectionMenu("FILL CLASSIFIER", "Fill:", fill, fillConfirmed);
    }

    private void displaySpikeSelectionMenu() {
        displaySelectionMenu("THIRD SPIKE", "Third Spike:", thirdSpike, spikeConfirmed);
    }

    // Alliance keeps its own layout since it shows RED/BLUE arrows instead
    // of a single value line, so it doesn't fit the generic menu shape.
    private void displayAllianceSelectionMenu() {
        telemetry.addLine("=================================");
        telemetry.addLine("SELECT ALLIANCE");
        telemetry.addLine("=================================");
        telemetry.addLine("");
        telemetry.addLine(selectedAlliance == Alliance.RED ? ">>> RED ALLIANCE <<<" : "    Red Alliance");
        telemetry.addLine(selectedAlliance == Alliance.BLUE ? ">>> BLUE ALLIANCE <<<" : "    Blue Alliance");
        telemetry.addLine("");
        telemetry.addLine("---------------------------------");
        telemetry.addData("Current Selection", selectedAlliance);
        telemetry.addData("Confirmed", allianceConfirmed ? "YES \u2713" : "NO");
        telemetry.addLine("---------------------------------");

        if (selectedAlliance != Alliance.NONE && !allianceConfirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press X to confirm selection");
        }
    }

    // =====================================================================
    //  TELEMETRY - "READY" SUMMARY SCREENS
    //  One generic (label, value, label, value...) renderer instead of six
    //  near-identical copies.
    // =====================================================================

    private void displayReadySummary(Object... labelsAndValues) {
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        for (int i = 0; i + 1 < labelsAndValues.length; i += 2) {
            telemetry.addData((String) labelsAndValues[i], labelsAndValues[i + 1]);
        }
        telemetry.addLine("");
    }

    private void displayReadyTeleScreen() {
        displayReadySummary("Alliance", selectedAlliance);
    }

    private void displayReadyTripsScreen() {
        displayReadySummary("NUMBER OF TRIPS:", mark);
    }

    public void displayReadyCloseScreen() {
        displayReadySummary("Alliance", selectedAlliance, "Number of Trips", mark, "Wolfpack", wolfpack);
    }

    public void displayReadyCloseScreen2() {
        displayReadySummary("Alliance", selectedAlliance, "Number of Trips", mark);
    }

    public void displayReadyCloseScreen3() {
        displayReadySummary("Alliance", selectedAlliance, "Fill Clasifier", fill);
    }

    public void displayReadyCloseScreen4() {
        displayReadySummary("Alliance", selectedAlliance, "3rd Spike", thirdSpike);
    }

    public void displayReady(Choices choices) {
        displayReadySummary("Alliance", choices);
    }

    // =====================================================================
    //  GETTERS
    // =====================================================================

    public Alliance getSelectedAlliance() { return selectedAlliance; }
    public Boolean getSelectedWolfpack() { return wolfpack; }
    public Boolean getFill() { return fill; }
    public Boolean getSpike() { return thirdSpike; }
    public int getMark() { return mark; }
}