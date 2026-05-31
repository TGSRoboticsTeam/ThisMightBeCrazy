package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * ArtemisDrive — identical to {@link ApolloDrive} in every way except the
 * AprilTag it targets. Apollo reads tag 24; Artemis reads tag 20. Everything
 * else (drive, turret, flywheel distance charts, exposure control, etc.) is
 * inherited unchanged.
 */
@Disabled
@TeleOp(name = "ArtemisDrive", group = "Swerve")
public class ArtemisDrive extends ApolloDrive {

   //
   //@Override
    protected int getTargetTagId() { return 20; }
}