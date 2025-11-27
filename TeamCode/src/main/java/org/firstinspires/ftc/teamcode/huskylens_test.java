package org.firstinspires.ftc.teamcode;

import static java.util.Collections.sort;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "Huskylens test")
public class huskylens_test extends LinearOpMode {

    private HuskyLens huskylens;
    String processHuskylens(List<HuskyLens.Block> data) {
        //1-green,2-purple
        data.sort((block1,block2)->block1.x-block2.x);
        if(data.size()==1) {
            if(data.get(0).id==1)
                return "GGG";
            else
                return "PPP";
        }
        else if(data.size()==2) {
                //0 is wider
                if(data.get(0).width>=1.5*data.get(1).width){
                    if(data.get(0).id==2)
                        return "PPG";
                    else
                        return "GGP";
                }
                //1 is wider
                else if(data.get(1).width>=1.5*data.get(0).width){
                    if(data.get(0).id==2)
                        return "PGG";
                    else
                        return "GPP";
                }
                else{
                    if(data.get(0).x<100) {
                        if(data.get(0).id==1)
                            return "GPG";
                        else
                            return "PGP";
                    }
                    else {
                        if(data.get(0).id==1)
                            return "PGP";
                        else
                            return "GPG";
                    }
                }
        }
        else if(data.size()==3){
            String result="";
            for(int i=0;i<3;i++)
                result+=(data.get(i).id==1?"G":"P");
            return result;
        }
        return "";
    }
    @Override
    public void runOpMode() {
        ElapsedTime myElapsedTime;
        List<HuskyLens.Block> HuskyLensBlocks;
        HuskyLens.Block HuskyLensBlock;

        huskylens = hardwareMap.get(HuskyLens.class, "huskylens");

        // Put initialization blocks here.
        telemetry.addData(">>", huskylens.knock() ? "Touch start to continue" : "Problem communicating with HuskyLens");
        huskylens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                    HuskyLensBlocks = Arrays.asList(huskylens.blocks());
//                    telemetry.addData("state","block 0 x"+HuskyLensBlocks.get(0).x+"y"+HuskyLensBlocks.get(0).y+"height"+HuskyLensBlocks.get(0).height+"width"+HuskyLensBlocks.get(0).width);
//                    telemetry.addData("Block count", JavaUtil.listLength(HuskyLensBlocks));
                    String s= processHuskylens(HuskyLensBlocks);
                    telemetry.addData("order",s);
                    telemetry.update();
            }
        }
    }
}