package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * ArtemisDrive — identical to ApolloDrive except for the goal field
 * coordinate and the AprilTag ID it tracks. Everything else (drive,
 * intake, flywheel, blocker, turret, telemetry, etc.) is inherited.
 *
 * To tweak shared behaviour, edit ApolloDrive — both opmodes get the
 * change automatically. Only change this file if Artemis's goal moves
 * or its target tag changes.
 */
@TeleOp(name = "ArtemisDrive", group = "Swerve")
public class ArtemisDrive extends ApolloDrive {

    @Override protected double getGoalFieldX()  { return  128.0; } // inches, +X
    @Override protected double getGoalFieldY()  { return -128.0; } // inches, -Y
    @Override protected int    getTargetTagId() { return  20;    }
}