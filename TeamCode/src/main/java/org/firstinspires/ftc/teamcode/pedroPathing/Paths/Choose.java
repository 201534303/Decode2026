package org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import static org.firstinspires.ftc.teamcode.pedroPathing.Paths.Choose.Choices.NONE;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Choose {
    protected Gamepad gamepad1;
    protected Telemetry telemetry;
    private boolean confirmed = false;
    public enum Choices {
        RED, BLUE, NONE
    }

    public Choices alliance = NONE;

    public Choose(Gamepad g1, Telemetry t) {
        telemetry = t;
        gamepad1 = g1;
    }

    private void displaySelectionMenu() {
        telemetry.addLine("=================================");
        telemetry.addLine("SELECT ALLIANCE");
        telemetry.addLine("=================================");
        telemetry.addLine("");
        telemetry.addLine((alliance == Choices.RED ? ">>> RED ALLIANCE <<<" : "    Red Alliance"));
        telemetry.addLine((alliance == Choices.BLUE ? ">>> BLUE ALLIANCE <<<" : "    Blue Alliance"));
        telemetry.addLine("");
        telemetry.addLine("---------------------------------");
        telemetry.addData("Current Selection", alliance);
        telemetry.addData("Confirmed", confirmed ? "YES ✓" : "NO");
        telemetry.addLine("---------------------------------");

        if (alliance != Choices.NONE && !confirmed) {
            telemetry.addLine("");
            telemetry.addLine("Press X to confirm selection");
        }
    }

    public void displayReady(Choices choices){
        telemetry.addLine("CONFIGURATION COMPLETE");
        telemetry.addLine("");
        telemetry.addData(choicePrint(choices), choices);
        telemetry.addLine("");
    }
    private String choicePrint(Choices choice){
        if (choice == Choices.RED || choice == Choices.BLUE){
            return "Alliance";
        } return "";
    }
}