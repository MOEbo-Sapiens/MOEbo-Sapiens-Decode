package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class VisionResult {

    public final Pose3D pose;
    public final int tagCount;

    public VisionResult() {
        this(null, 0);
    }

    public VisionResult(Pose3D pose, int tagCount) {
        this.pose = pose;
        this.tagCount = tagCount;
    }

    public boolean hasPose() {
        return pose != null && tagCount > 0;
    }
}
