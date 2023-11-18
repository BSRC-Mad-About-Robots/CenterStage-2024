package src.main.java.org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
public class tp extends OpMode {

    private DcMotor T;
    @Override
    public void init (){
        T = hardwareMap.dcMotor.get("t");
    }

    @Override
    public void loop() {
        T.setPower(-1);
    }
}


