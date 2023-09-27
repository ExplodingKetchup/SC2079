package com.example.mdp40;

import java.util.Observable;

public class MySubject extends Observable {
    public void changeInstruction() {
        setChanged();
        notifyObservers();
    }
}