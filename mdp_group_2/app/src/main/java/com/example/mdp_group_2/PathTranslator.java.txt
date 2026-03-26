package com.example.mdp_group_14;

import android.app.Activity;
import android.util.Log;
import android.widget.Toast;

public class PathTranslator {
    private static final String TAG = "PathTranslator";
    private static GridMap gridMap;
    private static final int CELL_LENGTH = 10; //length of each cell in cm
    private static final int MILLI_DELAY = 50;    // delay between movement commands

    private static final int LEFT_TURNING_RADIUS = 40;
    private static final int RIGHT_TURNING_RADIUS = 41;
    private static final int BLEFT_TURNING_RADIUS = 37;
    private static final int BRIGHT_TURNING_RADIUS = 41;

    private int curX, curY;
    private String dir;

    public PathTranslator() {
        this.gridMap = Home.getGridMap();
    }

    public PathTranslator(GridMap gridMap) {
        this.gridMap = gridMap;
        this.curX = 2;
        this.curY = 1;
        this.dir = "up";
    }

    public void translatePath(final String stmCommand) {
        showLog("Entered translatePath: " + stmCommand);
        
        // Run in a background thread to allow UI updates and animation
        new Thread(new Runnable() {
            @Override
            public void run() {
                char commandType = 'z';
                int commandValue = 0;

                if(stmCommand.contains("MOVE")){
                    String[] parts = stmCommand.split(",");
                    if (parts.length >= 3) {
                        try {
                            commandValue = Integer.parseInt(parts[1].trim());
                            String direction = parts[2].trim().toUpperCase();
                            showLog("Parsed MOVE: val=" + commandValue + ", dir=" + direction);
                            
                            if(direction.equals("FORWARD")){
                                commandType = 'f';
                            }
                            else if (direction.equals("BACKWARD")){
                                commandType = 'b';
                            }
                        } catch(Exception e) {
                            showLog("Error parsing MOVE: " + e.getMessage());
                        }
                    }
                }
                else if (stmCommand.contains("TURN")){
                    String[] parts = stmCommand.split(",");
                    if (parts.length >= 2) {
                        String direction = parts[1].trim().toUpperCase();
                        if(direction.equals("FORWARD_RIGHT")) commandType = 'd';
                        else if(direction.equals("FORWARD_LEFT")) commandType = 'a';
                        else if (direction.equals("BACKWARD_RIGHT")) commandType = 'e';
                        else if (direction.equals("BACKWARD_LEFT")) commandType = 'q';
                    }
                }

                int moves = 0;
                if (commandType == 'f' || commandType == 'b') {
                    moves = commandValue / CELL_LENGTH;
                }

                switch(commandType) {
                    case 'f':
                        for(int i = 0; i < moves; i++) {
                            updateUI("forward");
                            //Home.printMessage("MOVE,10,FORWARD");
                            try { Thread.sleep(MILLI_DELAY); } catch(InterruptedException e) {}
                        }
                        break;
                    case 'b':
                        for(int i = 0; i < moves; i++) {
                            updateUI("back");
                            //Home.printMessage("MOVE,10,BACKWARD");
                            try { Thread.sleep(MILLI_DELAY); } catch(InterruptedException e) {}
                        }
                        break;
                    case 'd':
                        updateUI("right");
                        break;
                    case 'a':
                        updateUI("left");
                        break;
                    case 'q':
                        updateUI("backleft");
                        break;
                    case 'e':
                        updateUI("backright");
                        break;
                    default:
                        showLog("Invalid or unhandled commandType: " + commandType);
                }
            }

            private void updateUI(final String direction) {
                if (gridMap.getContext() instanceof Activity) {
                    ((Activity) gridMap.getContext()).runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            gridMap.moveRobot(direction);
                            Home.refreshLabel();
                        }
                    });
                }
            }
        }).start();
    }

    private static void showLog(String message) {
        Log.d(TAG, message);
    }
}
