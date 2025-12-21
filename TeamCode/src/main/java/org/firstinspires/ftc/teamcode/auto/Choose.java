package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auto Selector")
public class Choose extends OpMode {
    public enum Alliance { RED, BLUE, NONE }
    private Alliance selectedAlliance = Alliance.NONE;
    private boolean allianceConfirmed = false;

    public enum Auto { FAR, CLOSE, NONE }
    private Auto selectedAuto = Auto.NONE;
    private boolean autoConfirmed = false;

    private int mark = 0;
    private boolean numConfirmed = false;

    @Override
    public void init() {
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (!autoConfirmed) {
            handleAutoSelection();
            displayAutoSelectionMenu();
        } else if (!numConfirmed) {
            handleAutoNum();
            displayNumSelectionMenu();
        } else if (!allianceConfirmed) {
            handleAllianceSelection();
            displayAllianceSelectionMenu();
        }
        else {
            displayReadyScreen();
        }

        telemetry.update();
    }

    private void handleAutoSelection() {
        if (gamepad1.dpad_up) {
            selectedAuto = Auto.FAR;
        } if (gamepad1.dpad_down) {
            selectedAuto = Auto.CLOSE;
        } if (gamepad1.a && selectedAuto != Auto.NONE) {
            autoConfirmed = true;
        }
    }

    private void handleAllianceSelection() {
        if (gamepad1.dpad_down) {
            selectedAlliance = Alliance.BLUE;
        } if (gamepad1.dpad_up) {
            selectedAlliance = Alliance.RED;
        } if (gamepad1.a && selectedAlliance != Alliance.NONE) {
            allianceConfirmed = true;
        }
    }

    private void handleAutoNum() {
        if (gamepad1.dpad_down) {
            if (mark > 0){
                mark -= 1;
            } else{
                mark = 0;
            }

        } if (gamepad1.dpad_up) {
            if (mark < 5){
                mark += 1;
            } else{
                mark = 5;
            }
        } if (gamepad1.a) {
            numConfirmed = true;
        }
    }

    private void displayAutoSelectionMenu() {
        telemetry.addLine("AUTO TYPE");
        telemetry.addLine( (selectedAuto == Auto.FAR ? "> FAR AUTO" : "Far Auto"));
        telemetry.addLine((selectedAuto == Auto.CLOSE ? "> CLOSE AUTO" : "Close Auto"));
        telemetry.addLine("---------------------------------");
        telemetry.addData("Current Selection", selectedAuto);
        telemetry.addData("Confirmed", autoConfirmed ? "YES" : "NO - Press X to confirm");
        telemetry.addLine("---------------------------------");

        if (selectedAuto != Auto.NONE && !autoConfirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press X to confirm and continue");
        }
    }

    private void displayNumSelectionMenu() {
        telemetry.addLine("AUTO VERSION");
        telemetry.addData("Trips:", mark);
        telemetry.addLine("---------------------------------");
        telemetry.addData("Confirmed", numConfirmed ? "YES" : "NO - Press X to confirm");
        telemetry.addLine("---------------------------------");

        if (!numConfirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press X to confirm and continue");
        }
    }

    private void displayAllianceSelectionMenu() {
        telemetry.addLine("ALLIANCE");
        telemetry.addLine((selectedAlliance == Alliance.BLUE ? "> BLUE ALLIANCE" : "Blue Alliance"));
        telemetry.addLine((selectedAlliance == Alliance.RED ? "> RED ALLIANCE" : "Red Alliance"));
        telemetry.addLine("---------------------------------");
        telemetry.addData("Current Selection", selectedAlliance);
        telemetry.addData("Confirmed", allianceConfirmed ? "YES " : "NO - Press X to confirm");
        telemetry.addLine("---------------------------------");

        if (selectedAlliance != Alliance.NONE && !allianceConfirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press A to confirm selection");
        }
    }

    private void displayReadyScreen() {
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        telemetry.addData("Auto Version:", selectedAuto);
        telemetry.addData("Alliance:", selectedAlliance);
        telemetry.addLine("");
        telemetry.addLine("READY TO START");
    }

    @Override
    public void start() {
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
        telemetry.addData("Starting Auto", selectedAuto);
        telemetry.addData("Starting Alliance", selectedAlliance);
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Running Auto", selectedAuto);
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Auto selector stopped");
        telemetry.update();
    }

    public Auto getSelectedAuto() {
        return selectedAuto;
    }

    public Alliance getSelectedAlliance() {
        return selectedAlliance;
    }
}