package org.firstinspires.ftc.teamcode.positionTracker;

import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;

public final class Pose2DP {
    public final Vector translation;
    public final float rotation;

    public static final Pose2DP zero = new Pose2DP(Vector.zero(), 0);

    public Pose2DP(final Vector position, final float rotation) {
        this.translation = position;
        this.rotation = rotation;
    }

    public Pose2DP subtract(final Pose2DP other) {
        return new Pose2DP(this.translation.subtract(other.translation),
                Angle.wrapAnglePlusMinusPI(this.rotation - other.rotation));
    }

    @Override
    public String toString() {
        return translation.toString() + " dir: " + Angle.radToDeg(rotation);
    }
}
