package org.firstinspires.ftc.teamcode.util;

public class Constants {
    public enum ArmPID implements PIDConst {
        P(0.05),
        I(0.6),
        D(0.001);

        private final double val;

        ArmPID(double val) {
            this.val = val;
        }

        public double getVal() {
            return val;
        }
    }

    public enum BatteryPID implements PIDConst {
        P(0.015),
        I(0),
        D(0.001);

        private final double val;

        BatteryPID(double val) {
            this.val = val;
        }

        public double getVal() {
            return val;
        }
    }

    public enum ArmConst implements MotorConst {
        BASE(0),
        LOW(250),
        MIDDLE(550),
        HIGH(1150);

        private final int pos;

        ArmConst(int pos) {
            this.pos = pos;
        }

        @Override
        public int getPos() {
            return pos;
        }
    }

    public enum BatteryConst implements MotorConst {
        LOW(0),
        MIDDLE(145),
        HIGH(300);

        private final int pos;

        BatteryConst(int pos) {
            this.pos = pos;
        }

        @Override
        public int getPos() {
            return pos;
        }
    }

    public enum ServoState {
        OPEN,
        CLOSE
    }

    public interface MotorConst {
        int getPos();
    }

    public interface PIDConst {
        double getVal();
    }
}
