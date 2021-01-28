package org.firstinspires.ftc.teamcode.Abstracts;

public class RingReturnObject{
    public int ring_count;
    public int listsize;
    public int maxlistsize;
    private String why = "";

    public RingReturnObject(int ring_count, int listsize, int maxlistsize){
        this.ring_count = ring_count;
        this.listsize = listsize;
    }
    public RingReturnObject(String message){
        this.why = message;
    }
    public String why(){
        return why;
    }
}
