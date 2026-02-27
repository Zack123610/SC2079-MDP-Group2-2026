package com.example.mdp_group_14;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.ClipData;
import android.content.Context;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.util.AttributeSet;
import android.util.Log;
import android.view.DragEvent;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.view.WindowManager;
import android.widget.ArrayAdapter;
import android.widget.ImageButton;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.UUID;

public class GridMap extends View {
    public GridMap(Context c) {
        super(c);
        initMap();
        setWillNotDraw(false);
    }

    SharedPreferences sharedPreferences;

    private final Paint blackPaint = new Paint();
    private final Paint whitePaint = new Paint();
    private final Paint maroonPaint = new Paint();
    private final Paint obstacleColor = new Paint();
    private final Paint imageColor = new Paint();
    private final Paint robotColor = new Paint();
    private final Paint endColor = new Paint();
    private final Paint startColor = new Paint();
    private final Paint waypointColor = new Paint();
    private final Paint unexploredColor = new Paint();
    private final Paint exploredColor = new Paint();
    private final Paint arrowColor = new Paint();
    private final Paint fastestPathColor = new Paint();

    private static String robotDirection = "None";
    private static int[] startCoord = new int[]{-1, -1};
    private static int[] curCoord = new int[]{-1, -1};
    private static int[] oldCoord = new int[]{-1, -1};
    private static ArrayList<int[]> obstacleCoord = new ArrayList<>();
    
    public static boolean canDrawRobot = false;
    private static boolean startCoordStatus = false;
    private static boolean setObstacleStatus = false;
    private static final boolean unSetCellStatus = false;
    private static final boolean setExploredStatus = false;
    private static boolean validPosition = false;
    private static final String TAG = "GridMap";
    private static final int COL = 20;
    private static final int ROW = 20;
    private static float cellSize;
    private static Cell[][] cells;
    Map<String, String> val2IdxMap;

    private boolean mapDrawn = false;
    private static final int CELL_LENGTH = 5; 
    
    // Robot Configuration: 3x3 footprint
    private static final int ROBOT_SIZE = 3;

    public int movesRx = 0;
    public int moves = 0;

    public ArrayList<String[]> ITEM_LIST = new ArrayList<>(Arrays.asList(
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20]
    ));
    public static ArrayList<String[]> imageBearings = new ArrayList<>(Arrays.asList(
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20]
    ));

    static ClipData clipData;
    static Object localState;
    int initialColumn, initialRow;
    public Canvas canvas;

    public GridMap(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        initMap();
        blackPaint.setStyle(Paint.Style.FILL_AND_STROKE);
        whitePaint.setColor(Color.WHITE);
        whitePaint.setTextSize(17);
        whitePaint.setTextAlign(Paint.Align.CENTER);
        maroonPaint.setColor(getResources().getColor(R.color.brightRed));
        maroonPaint.setStrokeWidth(8);
        obstacleColor.setColor(getResources().getColor(R.color.black));
        imageColor.setColor(getResources().getColor(R.color.rockColor));
        robotColor.setColor(getResources().getColor(R.color.pikaYellow));
        robotColor.setStrokeWidth(2);
        endColor.setColor(Color.RED);
        startColor.setColor(Color.CYAN);
        waypointColor.setColor(Color.GREEN);
        unexploredColor.setColor(getResources().getColor(R.color.skyBlue));
        exploredColor.setColor(getResources().getColor(R.color.lighterYellow));
        arrowColor.setColor(Color.BLACK);
        fastestPathColor.setColor(Color.MAGENTA);

        sharedPreferences = getContext().getSharedPreferences("Shared Preferences", Context.MODE_PRIVATE);
        this.val2IdxMap = new HashMap<>();
    }

    private void initMap() {
        setWillNotDraw(false);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (!mapDrawn) {
            mapDrawn = true;
            this.createCell();
        }
        drawIndividualCell(canvas);
        drawHorizontalLines(canvas);
        drawVerticalLines(canvas);
        drawGridNumber(canvas);
        if (getCanDrawRobot())
            drawRobot(canvas, curCoord);
        drawObstacles(canvas);
    }

    private void drawObstacles(Canvas canvas) {
        for (int i = 0; i < obstacleCoord.size(); i++) {
            int col = obstacleCoord.get(i)[0];
            int row = obstacleCoord.get(i)[1];
            if (ITEM_LIST.get(row)[col] == null || ITEM_LIST.get(row)[col].equals("") || ITEM_LIST.get(row)[col].equals("Nil")) {
                whitePaint.setTextSize(15);
                canvas.drawText(String.valueOf(i + 1), cells[col + 1][19 - row].startX + (cellSize / 2), cells[col + 1][19 - row].startY + (cellSize / 2) + 5, whitePaint);
            } else {
                whitePaint.setTextSize(17);
                canvas.drawText(ITEM_LIST.get(row)[col], cells[col + 1][19 - row].startX + (cellSize / 2), cells[col + 1][19 - row].startY + (cellSize / 2) + 10, whitePaint);
            }

            switch (imageBearings.get(row)[col]) {
                case "North":
                    canvas.drawLine(cells[col + 1][19 - row].startX, cells[col + 1][19 - row].startY, cells[col + 1][19 - row].endX, cells[col + 1][19 - row].startY, maroonPaint);
                    break;
                case "South":
                    canvas.drawLine(cells[col + 1][19 - row].startX, cells[col + 1][19 - row].startY + cellSize, cells[col + 1][19 - row].endX, cells[col + 1][19 - row].startY + cellSize, maroonPaint);
                    break;
                case "East":
                    canvas.drawLine(cells[col + 1][19 - row].startX + cellSize, cells[col + 1][19 - row].startY, cells[col + 1][19 - row].startX + cellSize, cells[col + 1][19 - row].endY, maroonPaint);
                    break;
                case "West":
                    canvas.drawLine(cells[col + 1][19 - row].startX, cells[col + 1][19 - row].startY, cells[col + 1][19 - row].startX, cells[col + 1][19 - row].endY, maroonPaint);
                    break;
            }
        }
    }

    private void drawIndividualCell(Canvas canvas) {
        for (int x = 1; x <= COL; x++)
            for (int y = 0; y < ROW; y++)
                canvas.drawRect(cells[x][y].startX, cells[x][y].startY, cells[x][y].endX, cells[x][y].endY, cells[x][y].paint);
    }

    private void drawHorizontalLines(Canvas canvas) {
        for (int y = 0; y <= ROW; y++)
            canvas.drawLine(cells[1][y].startX, cells[1][y].startY - (cellSize / 30), cells[20][y].endX, cells[20][y].startY - (cellSize / 30), whitePaint);
    }

    private void drawVerticalLines(Canvas canvas) {
        for (int x = 0; x <= COL; x++)
            canvas.drawLine(cells[x][0].startX - (cellSize / 30) + cellSize, cells[x][0].startY - (cellSize / 30), cells[x][0].startX - (cellSize / 30) + cellSize, cells[x][19].endY + (cellSize / 30), whitePaint);
    }

    private void drawGridNumber(Canvas canvas) {
        for (int x = 1; x <= COL; x++) {
            float xOffset = (x > 10) ? (cellSize / 5) : (cellSize / 3);
            canvas.drawText(Integer.toString(x - 1), cells[x][20].startX + xOffset, cells[x][20].startY + (cellSize / 1.5f), blackPaint);
        }
        for (int y = 0; y < ROW; y++) {
            float xOffset = ((20 - y) > 10) ? (cellSize / 4) : (cellSize / 2.5f);
            canvas.drawText(Integer.toString(ROW - 1 - y), cells[0][y].startX + xOffset, cells[0][y].startY + (cellSize / 1.5f), blackPaint);
        }
    }

    private void drawRobot(Canvas canvas, int[] curCoord) {
        float xCoord, yCoord;
        BitmapFactory.Options op = new BitmapFactory.Options();
        Bitmap bm, mapscalable;

        int androidRowCoord = curCoord[1];

        // Bounds check for 3x3 robot (allowed row index 1 to 18)
        if (androidRowCoord < 1 || androidRowCoord > 18 || curCoord[0] < 3 || curCoord[0] > 20) {
            return;
        }

        int convRow = convertRow(androidRowCoord);
        int startColIdx = curCoord[0] - 2;

        // Draw visual 3x3 boundary lines safely
        for (int i = 0; i <= ROBOT_SIZE; i++) {
            float yPos = cells[startColIdx][convRow - 2].startY + i * cellSize;
            canvas.drawLine(cells[startColIdx][convRow].startX, yPos, cells[curCoord[0]][convRow].endX, yPos, robotColor);
        }
        for (int i = 0; i <= ROBOT_SIZE; i++) {
            float xPos = cells[startColIdx][convRow].startX + i * cellSize;
            canvas.drawLine(xPos, cells[startColIdx][convRow - 2].startY, xPos, cells[curCoord[0]][convRow].endY, robotColor);
        }

        op.inMutable = true;
        int resId;
        switch (this.getRobotDirection()) {
            case "down": resId = R.drawable.crab_down; break;
            case "right": resId = R.drawable.crab_right; break;
            case "left": resId = R.drawable.crab_left; break;
            default: resId = R.drawable.crab_up; break;
        }
        bm = BitmapFactory.decodeResource(getResources(), resId, op);
        mapscalable = Bitmap.createScaledBitmap(bm, (int)(cellSize * ROBOT_SIZE), (int)(cellSize * ROBOT_SIZE), true);
        
        xCoord = cells[startColIdx][convRow].startX;
        yCoord = cells[startColIdx][convRow - 2].startY;
        canvas.drawBitmap(mapscalable, xCoord, yCoord, null);
    }

    public String getRobotDirection() { return robotDirection; }
    private void setValidPosition(boolean status) { validPosition = status; }
    public boolean getValidPosition() { return validPosition; }
    public void setSetObstacleStatus(boolean status) { setObstacleStatus = status; }
    public void setStartCoordStatus(boolean status) { startCoordStatus = status; }
    private boolean getStartCoordStatus() { return startCoordStatus; }
    public boolean getCanDrawRobot() { return canDrawRobot; }
    public boolean getSetObstacleStatus() { return setObstacleStatus; }

    private void createCell() {
        cells = new Cell[COL + 1][ROW + 1];
        this.calculateDimension();
        cellSize = this.getCellSize();
        for (int x = 0; x <= COL; x++)
            for (int y = 0; y <= ROW; y++)
                cells[x][y] = new Cell(x * cellSize + (cellSize / 30), y * cellSize + (cellSize / 30), (x + 1) * cellSize, (y + 1) * cellSize, unexploredColor, "unexplored");
    }

    public void setStartCoord(int col, int row) {
        startCoord[0] = col;
        startCoord[1] = row;
        String direction = getRobotDirection();
        if (direction.equals("None")) direction = "up";
        if (this.getStartCoordStatus()) this.setCurCoord(col, row, direction);

        String dir = (direction.equals("up")) ? "NORTH" : (direction.equals("down")) ? "SOUTH" : (direction.equals("left")) ? "WEST" : "EAST";
        if ((col - 3) >= 0 && (row - 1) >= 0) {
            Home.printMessage("ROBOT" + "," + (col - 3) * 5 + "," + (row - 1) * 5 + "," + dir.toUpperCase());
        }
        this.invalidate();
    }

    public void setCurCoord(int col, int row, String direction) {
        // Bounds check updated: allowed row starts from 1 (visual y=0)
        if (row < 1 || row > 18 || col < 3 || col > 20) {
            return;
        }
        
        int[] temp = {col, row};
        if (checkForObstacleCollision(temp, obstacleCoord)) {
            showLog("Cannot move: collision detected");
            return;
        }

        curCoord[0] = col;
        curCoord[1] = row;
        this.setRobotDirection(direction);
        this.updateRobotAxis(col, row, direction);
        int convertedRow = this.convertRow(row);
        
        for (int x = col - 2; x <= col; x++)
            for (int y = convertedRow - 2; y <= convertedRow; y++)
                cells[x][y].setType("robot");
    }

    public int[] getCurCoord() { return curCoord; }
    private void calculateDimension() { this.setCellSize(getWidth() / (COL + 1)); }
    public int convertRow(int row) { return (20 - row); }
    private void setCellSize(float cellSize) { GridMap.cellSize = cellSize; }
    public float getCellSize() { return cellSize; }

    private void setOldRobotCoord(int oldCol, int oldRow) {
        oldCoord[0] = oldCol;
        oldCoord[1] = oldRow;
        int convertedRow = this.convertRow(oldRow);
        if (convertedRow < 0) return;
        
        for (int x = oldCol - 2; x <= oldCol; x++)
            for (int y = convertedRow - 2; y <= convertedRow; y++)
                cells[x][y].setType("explored");
    }

    public void setRobotDirection(String direction) {
        SharedPreferences.Editor editor = sharedPreferences.edit();
        robotDirection = direction;
        editor.putString("direction", direction);
        editor.apply();
        this.invalidate();
    }

    public void updateRobotAxis(int col, int row, String direction) {
        TextView xAxis = ((Activity) getContext()).findViewById(R.id.xAxisTextView);
        TextView yAxis = ((Activity) getContext()).findViewById(R.id.yAxisTextView);
        TextView dirAxis = ((Activity) getContext()).findViewById(R.id.directionAxisTextView);
        if (xAxis != null) xAxis.setText(String.valueOf(col - 1));
        if (yAxis != null) yAxis.setText(String.valueOf(row - 1));
        if (dirAxis != null) dirAxis.setText(direction);
    }

    public void setObstacleCoord(int col, int row) {
        obstacleCoord.add(new int[]{col - 1, row - 1});
        cells[col][this.convertRow(row)].setType("obstacle");
        int num = obstacleCoord.size();
        Home.printMessage("OBSTACLE," + num + "," + (col - 1) * 10 + "," + (19 - convertRow(row)) * 10 + "," + (imageBearings.get(19 - convertRow(row))[col - 1]).toUpperCase() + "\n");
    }

    private class Cell {
        float startX, startY, endX, endY;
        Paint paint;
        String type;
        int id = -1;
        private Cell(float startX, float startY, float endX, float endY, Paint paint, String type) {
            this.startX = startX; this.startY = startY; this.endX = endX; this.endY = endY; this.paint = paint; this.type = type;
        }
        public void setType(String type) {
            this.type = type;
            if (type.equals("image")) paint = imageColor;
            else if (type.equals("obstacle")) paint = obstacleColor;
            else if (type.equals("robot")) paint = robotColor;
            else if (type.equals("explored")) paint = exploredColor;
            else paint = unexploredColor;
        }
        public int getId() { return id; }
    }

    @Override
    public boolean onDragEvent(DragEvent dragEvent) {
        clipData = dragEvent.getClipData();
        localState = dragEvent.getLocalState();
        int endColumn, endRow;
        
        if (dragEvent.getAction() == DragEvent.ACTION_DRAG_ENDED && !dragEvent.getResult()) {
            for (int i = 0; i < obstacleCoord.size(); i++) {
                if (Arrays.equals(obstacleCoord.get(i), new int[]{initialColumn - 1, initialRow - 1})) {
                    obstacleCoord.remove(i);
                    cells[initialColumn][20 - initialRow].setType("unexplored");
                    ITEM_LIST.get(initialRow - 1)[initialColumn - 1] = "";
                    imageBearings.get(initialRow - 1)[initialColumn - 1] = "";
                    Home.printMessage("OBSTACLE," + (i+1) + "," + (initialColumn)*10 + "," + (initialRow)*10 + ",-1");
                    break;
                }
            }
        } else if (dragEvent.getAction() == DragEvent.ACTION_DROP) {
            endColumn = (int) (dragEvent.getX() / cellSize);
            endRow = this.convertRow((int) (dragEvent.getY() / cellSize));

            if (endColumn >= 1 && endColumn <= 20 && endRow >= 1 && endRow <= 20) {
                String tempID = ITEM_LIST.get(initialRow - 1)[initialColumn - 1];
                String tempBearing = imageBearings.get(initialRow - 1)[initialColumn - 1];

                if (ITEM_LIST.get(endRow - 1)[endColumn - 1].equals("") && imageBearings.get(endRow - 1)[endColumn - 1].equals("")) {
                    ITEM_LIST.get(initialRow - 1)[initialColumn - 1] = "";
                    imageBearings.get(initialRow - 1)[initialColumn - 1] = "";
                    ITEM_LIST.get(endRow - 1)[endColumn - 1] = tempID;
                    imageBearings.get(endRow - 1)[endColumn - 1] = tempBearing;

                    for (int i = 0; i < obstacleCoord.size(); i++) {
                        if (Arrays.equals(obstacleCoord.get(i), new int[]{initialColumn - 1, initialRow - 1})) {
                            obstacleCoord.set(i, new int[]{endColumn - 1, endRow - 1});
                            cells[endColumn][20 - endRow].setType(cells[initialColumn][20 - initialRow].type);
                            cells[initialColumn][20 - initialRow].setType("unexplored");
                            Home.printMessage("OBSTACLE," + (i+1) + "," + (endColumn-1)*10 + "," + (endRow-1)*10 + "," + tempBearing.toUpperCase());
                            break;
                        }
                    }
                }
            }
        }
        this.invalidate();
        return true;
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (event.getAction() == MotionEvent.ACTION_DOWN) {
            int column = (int) (event.getX() / cellSize);
            int row = this.convertRow((int) (event.getY() / cellSize));
            initialColumn = column; initialRow = row;

            if (MappingFragment.dragStatus) {
                if (column >= 1 && column <= 20 && row >= 1 && row <= 20) {
                    if (!ITEM_LIST.get(row - 1)[column - 1].equals("") || !imageBearings.get(row - 1)[column - 1].equals("")) {
                        View.DragShadowBuilder dragShadowBuilder = new View.DragShadowBuilder(this);
                        this.startDrag(null, dragShadowBuilder, null, 0);
                        return true;
                    }
                }
            }

            if (MappingFragment.changeObstacleStatus) {
                if (column >= 1 && column <= 20 && row >= 1 && row <= 20) {
                    if (!ITEM_LIST.get(row - 1)[column - 1].equals("") || !imageBearings.get(row - 1)[column - 1].equals("")) {
                        final int tRow = row; final int tCol = column;
                        AlertDialog.Builder mBuilder = new AlertDialog.Builder(this.getContext());
                        View mView = ((Activity) this.getContext()).getLayoutInflater().inflate(R.layout.activity_dialog, null);
                        mBuilder.setTitle("Change Existing Bearing");
                        final Spinner mBearingSpinner = mView.findViewById(R.id.bearingSpinner2);
                        ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this.getContext(), R.array.imageBearing_array, android.R.layout.simple_spinner_item);
                        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
                        mBearingSpinner.setAdapter(adapter);

                        String currentBearing = imageBearings.get(row - 1)[column - 1];
                        if (currentBearing.equals("North")) mBearingSpinner.setSelection(0);
                        else if (currentBearing.equals("South")) mBearingSpinner.setSelection(1);
                        else if (currentBearing.equals("East")) mBearingSpinner.setSelection(2);
                        else if (currentBearing.equals("West")) mBearingSpinner.setSelection(3);

                        mBuilder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialogInterface, int i) {
                                String newBearing = mBearingSpinner.getSelectedItem().toString();
                                imageBearings.get(tRow - 1)[tCol - 1] = newBearing;
                                for (int m = 0; m < obstacleCoord.size(); m++) {
                                    if (Arrays.equals(obstacleCoord.get(m), new int[]{tCol - 1, tRow - 1})) {
                                        Home.printMessage("OBSTACLE," + (m+1) + "," + (tCol - 1)*10 + "," + (tRow - 1)*10 + "," + newBearing.toUpperCase());
                                        break;
                                    }
                                }
                                invalidate();
                            }
                        });
                        mBuilder.setNegativeButton("Cancel", null);
                        mBuilder.setView(mView);
                        mBuilder.show();
                        return true;
                    }
                }
            }

            if (startCoordStatus) {
                if (!canDrawRobot) canDrawRobot = true;
                else {
                    for (int i = 0; i <= COL; i++)
                        for (int j = 0; j <= ROW; j++)
                            if (cells[i][j].type.equals("robot")) cells[i][j].setType("unexplored");
                }
                setStartCoord(column, row);
                startCoordStatus = false;
                this.invalidate();
                return true;
            }
            if (setObstacleStatus) {
                if (column >= 1 && column <= 20 && row >= 1 && row <= 20) {
                    ITEM_LIST.get(row - 1)[column - 1] = (MappingFragment.imageID.equals("Nil")) ? "" : MappingFragment.imageID;
                    imageBearings.get(row - 1)[column - 1] = MappingFragment.imageBearing;
                    setObstacleCoord(column, row);
                }
                this.invalidate();
                return true;
            }
        }
        return false;
    }

    public void resetMap() {
        updateRobotAxis(1, 1, "None");
        startCoord = new int[]{-1, -1}; curCoord = new int[]{-1, -1}; robotDirection = "None";
        obstacleCoord = new ArrayList<>(); mapDrawn = false; canDrawRobot = false;
        for (int i = 0; i < 20; i++) {
            for (int j = 0; j < 20; j++) { ITEM_LIST.get(i)[j] = ""; imageBearings.get(i)[j] = ""; }
        }
        this.invalidate();
    }

    public void moveRobot(String direction) {
        setValidPosition(false);
        int[] cur = getCurCoord();
        setOldRobotCoord(cur[0], cur[1]);
        String backupDir = robotDirection;
        Integer[] newCoords = {cur[0], cur[1]};

        Map.Entry<String, ArrayList<Integer[]>> entry;
        try {
            switch (direction) {
                case "forward":
                case "back":
                    Integer[] lastS = Straight.straight(newCoords, robotDirection, direction);
                    if (isValid(lastS[0], lastS[1])) { curCoord[0] = lastS[0]; curCoord[1] = lastS[1]; setValidPosition(true); }
                    break;
                case "left":
                case "right":
                case "backleft":
                case "backright":
                    entry = Turn.turn(newCoords, robotDirection, direction);
                    Integer[] lastT = entry.getValue().get(entry.getValue().size() - 1);
                    if (isValid(lastT[0], lastT[1])) { curCoord[0] = lastT[0]; curCoord[1] = lastT[1]; robotDirection = entry.getKey(); setValidPosition(true); }
                    break;
            }
        } catch (Exception e) { robotDirection = backupDir; }

        if (getValidPosition() && checkForObstacleCollision(curCoord, obstacleCoord)) {
            setValidPosition(false); 
            robotDirection = backupDir;
        }

        if (getValidPosition()) setCurCoord(curCoord[0], curCoord[1], robotDirection);
        else setCurCoord(oldCoord[0], oldCoord[1], robotDirection);
        
        this.invalidate();
    }

    private boolean isValid(int col, int row) {
        // Correct bounds for 3x3 robot allowed at visual y=0 (row=1)
        return col >= 3 && col <= 20 && row >= 1 && row <= 18;
    }

    public boolean checkForObstacleCollision(int[] coord, List<int[]> obstacles) {
        // 3x3 footprint check in visual coordinates
        for (int vCol = coord[0] - 3; vCol <= coord[0] - 1; vCol++) {
            for (int vRow = coord[1] - 1; vRow <= coord[1] + 1; vRow++) {
                for (int[] obs : obstacles) {
                    if (obs[0] == vCol && obs[1] == vRow) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    public boolean updateIDFromRpi(String obstacleID, String imageID) {
        int idx = Integer.parseInt(obstacleID);
        if (idx < obstacleCoord.size()) {
            int x = obstacleCoord.get(idx)[0];
            int y = obstacleCoord.get(idx)[1];
            ITEM_LIST.get(y)[x] = (imageID.equals("-1")) ? "NA" : imageID;
            this.invalidate();
            return true;
        }
        return false;
    }

    public static String saveObstacleList(){    
        String message ="";
        for (int i = 0; i < obstacleCoord.size(); i++) {
            message += ((obstacleCoord.get(i)[0]) + "," 
                    + (obstacleCoord.get(i)[1]) + ","   
                    + imageBearings.get(obstacleCoord.get(i)[1])[obstacleCoord.get(i)[0]].charAt(0));  
            if(i < obstacleCoord.size() - 1) message += "\n";    
        }
        return message;
    }

    public String getObstacles() {
        String msg = "ALG|";
        int obstId = 0;
        for (int i = 0; i < obstacleCoord.size(); i++) {
            int col = obstacleCoord.get(i)[0];
            int row = obstacleCoord.get(i)[1];
            msg += (col + "," + row + "," + imageBearings.get(row)[col].charAt(0) + ",");
            if(ITEM_LIST.get(row)[col] == null || ITEM_LIST.get(row)[col].equals("") || ITEM_LIST.get(row)[col].equals("Nil")) {
                msg += obstId++; 
            } else {
                msg += (-1); 
            }
            if (i < obstacleCoord.size() - 1) msg += "|";
        }
        return msg;
    }

    public void toggleCheckedBtn(String buttonName) {
        ToggleButton startBtn = ((Activity) getContext()).findViewById(R.id.startpointToggleBtn);
        ImageButton obsBtn = ((Activity) getContext()).findViewById(R.id.addObstacleBtn);
        if (!buttonName.equals("setStartPointToggleBtn") && startBtn != null && startBtn.isChecked()) {
            setStartCoordStatus(false);
            startBtn.toggle();
            startBtn.setBackgroundResource(R.drawable.border_black);
        }
        if (!buttonName.equals("obstacleImageBtn") && obsBtn != null && getSetObstacleStatus()) {
            setSetObstacleStatus(false);
            obsBtn.setBackgroundResource(R.drawable.border_black);
        }
    }

    private static void showLog(String m) { Log.d(TAG, m); }
}
