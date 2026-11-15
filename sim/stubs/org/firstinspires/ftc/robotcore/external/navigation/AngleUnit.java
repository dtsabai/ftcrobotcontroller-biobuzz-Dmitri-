package org.firstinspires.ftc.robotcore.external.navigation;
public enum AngleUnit {
    DEGREES, RADIANS;
    public static double normalizeRadians(double a) { while (a > Math.PI) a -= 2*Math.PI; while (a <= -Math.PI) a += 2*Math.PI; return a; }
    public static double normalizeDegrees(double a) { while (a > 180) a -= 360; while (a <= -180) a += 360; return a; }
}
