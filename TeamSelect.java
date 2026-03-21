package org.firstinspires.ftc.teamcode.mechanisms;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class teamSelect(){

    private boolean BlueAlliance = false;

    public GetAlliance(){
        return (BlueAlliance ? "BLUE" : "RED");
    }
}