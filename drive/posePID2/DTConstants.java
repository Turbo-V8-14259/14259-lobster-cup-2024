package org.firstinspires.ftc.teamcode.drive.posePID2;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;

@Config
public class DTConstants {
    public static double NxyP = 1, NxyI = 0, NxyD = 0;
    public static double xyP = 1, xyI = 0, xyD = 0;

    public static double rP = 0.35, rI = 0, rD = 0;

    public static double pPrP = 1, pPrI = 0, pPrD = 0;

    public static double RBasePower = .1;
    public static double XYBasePower = .07;

    public static double maxAxialPower = 1;
    public static double maxAngularPower = 1;

    public static double allowedAxialError = 1;
    public static double allowedAngularError = M.toRadians(1);
}
