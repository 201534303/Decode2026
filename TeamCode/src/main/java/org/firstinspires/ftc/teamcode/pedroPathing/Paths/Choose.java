package org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Choose {
    protected Gamepad gamepad1;
    protected Telemetry telemetry;
    private boolean confirmed = false;
    public enum Choices {
        RED, BLUE
    }

    public Choose(Gamepad g1, Telemetry t) {
        telemetry = t;
        gamepad1 = g1;
    }

    /*private void displayAllianceSelectionMenu() {
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
    }*/

    public void displayReady(Choices choices){
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        telemetry.addData("Alliance", choices);
        telemetry.addLine("");
    }
    private String choicePrint(Choices choice){
        if (choice == Choices.RED || choice == Choices.BLUE){
            return "Alliance";
        } return "";
    }
}