package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Indexer {
    RevColorSensorV3 slotOne, slotTwo, slotThree;

    Servo kickerOne, kickerTwo, kickerThree;
    ElapsedTime timer = new ElapsedTime();
    String[] order;

    public Indexer(RevColorSensorV3 slotOne, RevColorSensorV3 slotTwo, RevColorSensorV3 slotThree, Servo kickerOne, Servo kickerTwo, Servo kickerThree, String[] order, ElapsedTime timer){
        this.slotOne = slotOne;
        this.slotTwo = slotTwo;
        this.slotThree = slotThree;

        this.kickerOne = kickerOne;
        this.kickerTwo = kickerTwo;
        this.kickerThree = kickerThree;

        this.order = order;
        this.timer = timer;
    }

    public String [] getReadings(){
        String[] colors = new String[3];
        colors[0] = getStatus(slotOne);
        colors[1] = getStatus(slotTwo);
        colors[2] = getStatus(slotThree);

        return colors;
    }

    public void fire(){
        String[] slots = getReadings();

        if(slots[0] == "Empty" || slots[1] == "Empty" || slots[2] == "Empty"){
            kickerOne.setPosition(1);
            timer.reset();
        }

    }

    public String getStatus(RevColorSensorV3 color){
        if(color.getDistance(DistanceUnit.CM) < 6){

            if(color.green() > 200){
                return "Green";
            }else{
                return "Purple";
            }

        }else{
            return "Empty";
        }
    }




}
