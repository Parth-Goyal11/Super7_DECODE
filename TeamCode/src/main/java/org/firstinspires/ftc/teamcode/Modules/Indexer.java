package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Indexer {
    RevColorSensorV3 slotOne, slotTwo, slotThree;

    Servo kickerOne, kickerTwo, kickerThree;
    ElapsedTime timer = new ElapsedTime();
    String[] order;

    Servo[] kickers;


    public Indexer(RevColorSensorV3 slotOne, RevColorSensorV3 slotTwo, RevColorSensorV3 slotThree, Servo kickerOne, Servo kickerTwo, Servo kickerThree, String[] order, ElapsedTime timer){
        this.slotOne = slotOne;
        this.slotTwo = slotTwo;
        this.slotThree = slotThree;

        this.kickerOne = kickerOne;
        this.kickerTwo = kickerTwo;
        this.kickerThree = kickerThree;

        kickers = new Servo[]{kickerOne, kickerTwo, kickerThree};

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



    //TODO: Replace sleep waits with resetting timer to preserve Tele-Op Thread
    public void fire() throws InterruptedException {
        String[] slots = getReadings();

        if(slots[0].equals("Empty") || slots[1].equals("Empty") || slots[2].equals("Empty")){
            kickerOne.setPosition(1);
            sleep(500);
            kickerOne.setPosition(0);
            kickerTwo.setPosition(1);
            sleep(500);
            kickerTwo.setPosition(0);
            kickerThree.setPosition(1);
        }else{
            boolean[] fired = {false, false, false};

            for(int i = 0; i<order.length; i++){
                for(int j = 0; j<slots.length; j++){
                    if(slots[j].equals(order[i]) && !fired[j]){
                       kickers[j].setPosition(1);
                       fired[i] = true;
                       sleep(200);
                       kickers[j].setPosition(0);

                       break;
                    }
                }
            }
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
