package org.firstinspires.ftc.teamcode.teleop.configurations;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.OrcaV3;

import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;

@TeleOp
@Config
@Mercurial.Attach
@BulkRead.Attach
@OrcaV3.Attach
public class VSlidesPID extends OpMode {
    public static double p,i,d,f,target;

    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void loop(){
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        OrcaV3.deposit().PIDTuning(p,i,d,f,target);
        telemetry.addData("(Deposit) Current Pos: ", OrcaV3.deposit().getCurrentSlidePosition());
        telemetry.addData("(Deposit) Target Pos: ", target);
        telemetry.update();

    }
}
