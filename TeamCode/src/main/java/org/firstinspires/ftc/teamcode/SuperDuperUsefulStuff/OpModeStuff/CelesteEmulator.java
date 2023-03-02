package org.firstinspires.ftc.teamcode.SuperDuperUsefulStuff.OpModeStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.SuperDuperUsefulStuff.CelesteClassic.Celeste;
//import org.firstinspires.ftc.teamcode.SuperDuperUsefulStuff.CelesteClassic.Pico8;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SuperDuperUsefulStuff.CelesteClassic.Celeste;
import org.firstinspires.ftc.teamcode.SuperDuperUsefulStuff.CelesteClassic.Pico8;

@Disabled
@TeleOp
public class CelesteEmulator extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pico8 p8 = new Pico8();
        Celeste celeste = new Celeste(p8);
        p8.load_game(celeste);
        p8.set_btn_state(0);
        celeste.load_room(0 % 8, 0);
        telemetry.setMsTransmissionInterval(15);
        //telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE)
        while (!isStarted()) {
            double start = System.currentTimeMillis();
            p8.set_btn_state(0);
            p8.set_inputs(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.a || gamepad1.y, gamepad1.x || gamepad1.b);
            p8.step();
            String print = celeste.getString().replace("\n", "<br>");
            telemetry.addLine(print);
            telemetry.update();
            while (System.currentTimeMillis() - start < 33);
        }
    }
}
