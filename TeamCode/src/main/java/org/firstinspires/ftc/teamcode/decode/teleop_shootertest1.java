package org.firstinspires.ftc.teamcode.decode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name="shooter Test1")
public class teleop_shootertest1 extends LinearOpMode {
    DecodeRobotHardware robot = new DecodeRobotHardware(this);

    DcMotorEx shooterTop;
    DcMotorEx shooterBottom;

    public static double shooterPower = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
        shooterTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");
        shooterBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom.setDirection(DcMotorEx.Direction.REVERSE);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        waitForStart();
        while (opModeIsActive()) {
            shooterTop.setPower(shooterPower);
            shooterBottom.setPower(shooterPower);
            dashboardTelemetry.addData("velocity", shooterTop.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.update();
            telemetry.addData("velocity", shooterTop.getVelocity(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}

