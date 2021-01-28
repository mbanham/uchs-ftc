package org.firstinspires.ftc.teamcode.Abstracts;

public class HandledException extends Throwable {
    private String message;
    public HandledException(String message){
        this.message = message;
    }
    public String what(){
        return message;
    }
}
