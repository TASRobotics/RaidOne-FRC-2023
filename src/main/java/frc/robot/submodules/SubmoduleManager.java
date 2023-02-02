package frc.robot.submodules;

import java.util.Arrays;
import java.util.List;

public class SubmoduleManager {
    
    private static SubmoduleManager instance = null;

    /**
     * singleton
     * 
     * @return the only instance of this SubmoduleManager class
     */
    public static SubmoduleManager getInstance(){
        if(instance==null){
            instance = new SubmoduleManager();
        }
        return instance;
    }

    private SubmoduleManager(){
    }

    private List<Submodule> submodules;

    /**
     * get the list of submodules
     * 
     * @param submodules varargs list of submodules
     */
    public void setSubmodules(Submodule... submodules){
        this.submodules = Arrays.asList(submodules);
    }

    /**
     * calls the {@link Submodule#onInit()} method for all submodules
     */
    public void onInit(){
        submodules.forEach(o -> o.onInit());
    }

    public void onStart(double timestamp){
        submodules.forEach(o -> o.onStart(timestamp));
    }

    public void onStop(double timestamp){
        submodules.forEach(Submodule::stop);
    }

    public void onLoop(double timestamp){
        submodules.forEach(o -> o.update(timestamp));
        submodules.forEach(Submodule::run);
    }
}
