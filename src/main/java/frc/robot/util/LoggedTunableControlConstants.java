package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedTunableControlConstants {
    private final String baseKey;

    private final LoggedNetworkNumber loggedKP;
    private final LoggedNetworkNumber loggedKI;
    private final LoggedNetworkNumber loggedKD;
    private final LoggedNetworkNumber loggedKS;
    private final LoggedNetworkNumber loggedKV;
    private final LoggedNetworkNumber loggedKCos;

    private double lastKP;
    private double lastKI;
    private double lastKD;
    private double lastKS;
    private double lastKV;
    private double lastKCos;

    private UpdateCallback callback;

    public LoggedTunableControlConstants(String baseKey) {
        if (baseKey.endsWith("/")) {
            baseKey = baseKey.substring(0, baseKey.length() - 1);
        }

        this.baseKey = baseKey;

        loggedKP = new LoggedNetworkNumber(baseKey + "/Tuning/kP");
        loggedKI = new LoggedNetworkNumber(baseKey + "/Tuning/kI");
        loggedKD = new LoggedNetworkNumber(baseKey + "/Tuning/kD");
        loggedKS = new LoggedNetworkNumber(baseKey + "/Tuning/kS");
        loggedKV = new LoggedNetworkNumber(baseKey + "/Tuning/kV");
        loggedKCos = new LoggedNetworkNumber(baseKey + "/Tuning/kCos");

        System.out.println("Control Constants created for " + baseKey);
    }

    public LoggedTunableControlConstants setP(double setValue) {
        loggedKP.set(setValue);
        lastKP = setValue;
        return this;
    }

    public LoggedTunableControlConstants setI(double setValue) {
        loggedKI.set(setValue);
        lastKI = setValue;
        return this;
    }

    public LoggedTunableControlConstants setD(double setValue) {
        loggedKD.set(setValue);
        lastKD = setValue;
        return this;
    }

    public LoggedTunableControlConstants setS(double setValue) {
        loggedKS.set(setValue);
        lastKS = setValue;
        return this;
    }

    public LoggedTunableControlConstants setV(double setValue) {
        loggedKV.set(setValue);
        lastKV = setValue;
        return this;
    }

    public LoggedTunableControlConstants setCos(double setValue) {
        loggedKCos.set(setValue);
        lastKCos = setValue;
        return this;
    }

    public void setCallback(UpdateCallback callback) {
        this.callback = callback;
    }

    public void periodic() {
        double currentKP = loggedKP.get();
        double currentKI = loggedKI.get();
        double currentKD = loggedKD.get();
        double currentKS = loggedKS.get();
        double currentKV = loggedKV.get();
        double currentKCos = loggedKCos.get();

        if (
            currentKP != lastKP ||
            currentKI != lastKI ||
            currentKD != lastKD ||
            currentKS != lastKS ||
            currentKV != lastKV ||
            currentKCos != lastKCos
        ) {
            lastKP = currentKP;
            lastKI = currentKI;
            lastKD = currentKD;
            lastKS = currentKS;
            lastKV = currentKV;
            lastKCos = currentKCos;

            if (callback == null) {
                throw new Error("LoggedTunableControlConstants changed, but no callback was set.");
            } else {
                callback.execute(currentKP, currentKI, currentKD, currentKS, currentKV, currentKCos);
                System.out.println("Control Constants for " + baseKey + " have updated.");
            }
        }
    }

    @FunctionalInterface
    public static interface UpdateCallback {
        void execute(double kP, double kI, double kD, double kS, double kV, double kCos);
    }
}