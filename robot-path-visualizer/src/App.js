import React, { useState, useEffect, useRef } from 'react';

const GRID_SIZE = 20;
const CELL_SIZE = 40; // pixels

const GridVisualization = () => {
  // State
  const [obstacles, setObstacles] = useState([]);
  const [allSegments, setAllSegments] = useState([]);
  const [robot, setRobot] = useState({
    direction: 'NORTH',
    south_west: { x: 0, y: 0 },
    north_east: { x: 2, y: 2 }
  });
  const [pathData, setPathData] = useState(null);
  const [animation, setAnimation] = useState({
    isPlaying: false,
    currentStep: 0,
    speed: 100, // ms per step
    showCameraFOV: true,
    showPathTrail: true
  });
  const [selectedCell, setSelectedCell] = useState(null);
  const [direction, setDirection] = useState('NORTH');
  const [loading, setLoading] = useState(false);
  
  const animationRef = useRef(null);
  const pathPoints = pathData?.path || [];

  // Start zone (40x40cm = 4x4 cells)
  const startZone = { x1: 0, y1: 0, x2: 3, y2: 3 };

  // Calculate robot center position from path point
  const getRobotPosition = (step) => {
    if (!pathPoints[step]) return robot;
    
    const point = pathPoints[step];
    // Calculate robot bounds from center point (assuming 2x2 robot)
    return {
      direction: point.direction,
      center: { x: point.x, y: point.y },
      south_west: { x: point.x - 1, y: point.y - 1 },
      north_east: { x: point.x + 1, y: point.y + 1 }
    };
  };

  // Get camera position and field of view
  const getCameraInfo = (robotPos) => {
    const center = robotPos.center;
    const dir = robotPos.direction;
    
    // Camera is at front center of robot (1 cell in front)
    let cameraPos = { ...center };
    let fovAngle = 45; // degrees
    let fovLength = 2; // cells
    
    switch(dir) {
      case 'NORTH':
        cameraPos.y += 2; // Camera extends in front
        break;
      case 'SOUTH':
        cameraPos.y -= 2;
        break;
      case 'EAST':
        cameraPos.x += 2;
        break;
      case 'WEST':
        cameraPos.x -= 2;
        break;
    }
    
    return { cameraPos, fovAngle, fovLength, direction: dir };
  };

  // Get photo positions (where images would be taken)
  const getPhotoPositions = () => {
    if (!allSegments.length) return [];
    
    return allSegments.map(segment => {
      if (segment.path && segment.path.length > 0) {
        const lastPoint = segment.path[segment.path.length - 1];
        return {
          position: { x: lastPoint.x, y: lastPoint.y },
          imageId: segment.image_id,
          step: 0 // Could be enhanced to find actual step in path
        };
      }
      return null;
    }).filter(pos => pos !== null);
  };

  // Animation control
  useEffect(() => {
    if (animation.isPlaying && pathPoints.length > 0) {
      animationRef.current = setInterval(() => {
        setAnimation(prev => {
          if (prev.currentStep >= pathPoints.length - 1) {
            clearInterval(animationRef.current);
            return { ...prev, isPlaying: false };
          }
          return { ...prev, currentStep: prev.currentStep + 1 };
        });
      }, animation.speed);
    } else {
      clearInterval(animationRef.current);
    }
    
    return () => clearInterval(animationRef.current);
  }, [animation.isPlaying, pathPoints.length]);

  // Animation controls
  const playAnimation = () => setAnimation(prev => ({ ...prev, isPlaying: true }));
  const pauseAnimation = () => setAnimation(prev => ({ ...prev, isPlaying: false }));
  const resetAnimation = () => setAnimation(prev => ({ ...prev, currentStep: 0, isPlaying: false }));
  const nextStep = () => setAnimation(prev => ({ 
    ...prev, 
    currentStep: Math.min(prev.currentStep + 1, pathPoints.length - 1) 
  }));
  const prevStep = () => setAnimation(prev => ({ 
    ...prev, 
    currentStep: Math.max(prev.currentStep - 1, 0) 
  }));

  // Generate grid
  const grid = [];
  for (let y = GRID_SIZE - 1; y >= 0; y--) {
    for (let x = 0; x < GRID_SIZE; x++) {
      grid.push({ x, y });
    }
  }

  // Cell status checks
  const isStartZone = (x, y) => x >= startZone.x1 && x <= startZone.x2 && y >= startZone.y1 && y <= startZone.y2;
  
  const getObstacleAt = (x, y) => obstacles.find(obs => 
    x >= obs.south_west.x && x <= obs.north_east.x &&
    y >= obs.south_west.y && y <= obs.north_east.y
  );

  // Get current robot position for animation
  const currentRobot = animation.currentStep > 0 ? 
    getRobotPosition(animation.currentStep) : robot;
  
  const cameraInfo = getCameraInfo(currentRobot);
  const photoPositions = getPhotoPositions();

  // Check if cell is in camera FOV
  const isInCameraFOV = (x, y) => {
    if (!animation.showCameraFOV) return false;
    
    const cam = cameraInfo.cameraPos;
    const dx = x - cam.x;
    const dy = y - cam.y;
    const distance = Math.sqrt(dx*dx + dy*dy);
    
    if (distance > cameraInfo.fovLength) return false;
    
    // Check angle relative to camera direction
    let angle = Math.atan2(dy, dx) * (180 / Math.PI);
    
    // Adjust based on camera direction
    const dirAngles = {
      'NORTH': 90,
      'SOUTH': -90,
      'EAST': 0,
      'WEST': 180
    };
    
    const targetAngle = dirAngles[cameraInfo.direction] || 0;
    const angleDiff = Math.abs(angle - targetAngle);
    
    return angleDiff <= cameraInfo.fovAngle / 2;
  };

  // Handle cell click for obstacle placement
  const handleCellClick = (x, y) => {
    if (isStartZone(x, y)) {
      alert('Cannot place obstacles in start zone!');
      return;
    }

    const existingObstacle = getObstacleAt(x, y);
    if (existingObstacle) {
      setObstacles(obs => obs.filter(o => o.id !== existingObstacle.id));
      return;
    }

    const newId = obstacles.length > 0 ? Math.max(...obstacles.map(o => o.id)) + 1 : 1;
    const newObstacle = {
      id: newId,
      direction: direction,
      south_west: { x, y },
      north_east: { x, y }
    };
    
    setObstacles([...obstacles, newObstacle]);
  };

  const generatePath = async () => {
    setLoading(true);
    
    try {
      const requestBody = {
        robot: {
          direction: "NORTH",
          south_west: { x: 0, y: 0 },
          north_east: { x: 30, y: 30 }
        },
        obstacles: obstacles.map(obs => ({
          image_id: obs.id,
          direction: obs.direction,
          south_west: { x: obs.south_west.x * 10, y: obs.south_west.y * 10 },
          north_east: { x: (obs.north_east.x + 1) * 10, y: (obs.north_east.y + 1) * 10 }
        })),
        verbose: true
      };

      const response = await fetch('http://localhost:5001/pathfinding/', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(requestBody)
      });

      const data = await response.json();
      if (data.segments && data.segments.length > 0) {
        // Flatten all instructions
        const combinedInstructions = data.segments.flatMap(seg => seg.instructions);

        // Generate path points from instructions
        const generatePathPoints = (robotStart, segmentInstructions) => {
          const points = [];
          let pos = { ...robotStart.south_west }; // bottom-left corner
          let dir = robotStart.direction;

          const moveMap = {
            NORTH: { x: 0, y: 1 },
            SOUTH: { x: 0, y: -1 },
            EAST: { x: 1, y: 0 },
            WEST: { x: -1, y: 0 }
          };

          const roundToGrid = val => Math.round(val);

          segmentInstructions.forEach(inst => {
            if (typeof inst === 'string') {
              // Turn instructions
              let dx = 0, dy = 0;
              if (inst.includes('FORWARD_LEFT')) {
                switch(dir) {
                  case 'NORTH': dir = 'WEST'; dx = -4; dy = 1; break;
                  case 'SOUTH': dir = 'EAST'; dx = 4; dy = -1; break;
                  case 'EAST': dir = 'NORTH'; dy = 4; dx = 1; break;
                  case 'WEST': dir = 'SOUTH'; dy = -4; dx = -1; break;
                }
              } else if (inst.includes('FORWARD_RIGHT')) {
                switch(dir) {
                  case 'NORTH': dir = 'EAST'; dx = 5; dy = 1; break;
                  case 'SOUTH': dir = 'WEST'; dx = -5; dy = -1; break;
                  case 'EAST': dir = 'SOUTH'; dy = -5; dx = 1; break;
                  case 'WEST': dir = 'NORTH'; dy = 5; dx = -1; break;
                }
              } else if (inst.includes('BACKWARD_LEFT')) {
                switch(dir) {
                  case 'NORTH': dir = 'EAST'; dx = -3; dy = -2; break;
                  case 'SOUTH': dir = 'WEST'; dx = 3; dy = 2; break;
                  case 'EAST': dir = 'SOUTH'; dy = 3; dx = -2; break;
                  case 'WEST': dir = 'NORTH'; dy = -3; dx = 2; break;
                }
              } else if (inst.includes('BACKWARD_RIGHT')) {
                switch(dir) {
                  case 'NORTH': dir = 'WEST'; dx = 3; dy = -2; break;
                  case 'SOUTH': dir = 'EAST'; dx = -3; dy = 2; break;
                  case 'EAST': dir = 'NORTH'; dy = -3; dx = -2; break;
                  case 'WEST': dir = 'SOUTH'; dy = 3; dx = 2; break;
                }
              }

              pos = { x: roundToGrid(pos.x + dx), y: roundToGrid(pos.y + dy) };
              points.push({ ...pos, direction: dir });
            } else if (inst.move) {
              // Forward/Backward moves
              const vec = moveMap[dir];
              const steps = Math.round(inst.amount / 10); // 10cm per cell
              for (let s = 0; s < steps; s++) {
                pos = {
                  x: roundToGrid(pos.x + (inst.move === 'FORWARD' ? vec.x : -vec.x)),
                  y: roundToGrid(pos.y + (inst.move === 'FORWARD' ? vec.y : -vec.y))
                };
                points.push({ ...pos, direction: dir });
              }
            }
          });

          return points;
        };

        const path = generatePathPoints(robot, combinedInstructions);

        // Store path inside pathData to fit existing animation code
        setPathData({
          ...data.segments[0], // keep first segment metadata
          path: path,
          instructions: combinedInstructions
        });

        setAllSegments(data.segments);
      }
      resetAnimation();
      
    } catch (error) {
      console.error('Error:', error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div style={{ padding: '20px', fontFamily: 'Arial, sans-serif' }}>
      <h1>Animated Robot Path Planning</h1>
      
      <div style={{ display: 'flex', gap: '30px' }}>
        {/* Controls Panel */}
        <div style={{ width: '300px' }}>
          <h3>Controls</h3>
          
          <div style={{ marginBottom: '15px' }}>
            <label>Obstacle Direction:</label>
            <select 
              value={direction} 
              onChange={(e) => setDirection(e.target.value)}
              style={{ padding: '5px', width: '100%', marginTop: '5px' }}
            >
              <option value="NORTH">North</option>
              <option value="SOUTH">South</option>
              <option value="EAST">East</option>
              <option value="WEST">West</option>
            </select>
          </div>

          <div style={{ marginBottom: '15px' }}>
            <button 
              onClick={generatePath}
              disabled={loading || obstacles.length === 0}
              style={{
                padding: '10px 20px',
                backgroundColor: obstacles.length === 0 ? '#ccc' : '#4CAF50',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: obstacles.length === 0 ? 'not-allowed' : 'pointer',
                width: '100%'
              }}
            >
              {loading ? 'Generating...' : 'Generate Path'}
            </button>
          </div>

          {/* Animation Controls */}
          {pathData && (
            <div style={{ marginBottom: '20px', padding: '15px', backgroundColor: '#f5f5f5', borderRadius: '4px' }}>
              <h4>Animation Controls</h4>
              <div style={{ display: 'flex', gap: '5px', marginBottom: '10px' }}>
                <button onClick={playAnimation} disabled={animation.isPlaying}>▶ Play</button>
                <button onClick={pauseAnimation} disabled={!animation.isPlaying}>⏸ Pause</button>
                <button onClick={resetAnimation}>↺ Reset</button>
              </div>
              <div style={{ display: 'flex', gap: '5px', marginBottom: '10px' }}>
                <button onClick={prevStep}>◀ Prev</button>
                <button onClick={nextStep}>Next ▶</button>
              </div>
              
              <div style={{ marginBottom: '10px' }}>
                <label>Speed:</label>
                <input 
                  type="range" 
                  min="50" 
                  max="500" 
                  step="50"
                  value={animation.speed}
                  onChange={(e) => setAnimation(prev => ({ ...prev, speed: parseInt(e.target.value) }))}
                  style={{ width: '100%' }}
                />
                <span>{animation.speed}ms/step</span>
              </div>
              
              <div style={{ marginBottom: '10px' }}>
                Step: {animation.currentStep} / {pathPoints.length - 1}
              </div>
              
              <div style={{ display: 'flex', gap: '10px' }}>
                <label>
                  <input 
                    type="checkbox"
                    checked={animation.showCameraFOV}
                    onChange={(e) => setAnimation(prev => ({ ...prev, showCameraFOV: e.target.checked }))}
                  />
                  Show Camera FOV
                </label>
                <label>
                  <input 
                    type="checkbox"
                    checked={animation.showPathTrail}
                    onChange={(e) => setAnimation(prev => ({ ...prev, showPathTrail: e.target.checked }))}
                  />
                  Show Path Trail
                </label>
              </div>
            </div>
          )}

          {/* Legend */}
          <div style={{ marginTop: '20px' }}>
            <h4>Legend:</h4>
            <div style={{ display: 'flex', alignItems: 'center', marginBottom: '5px' }}>
              <div style={{ width: '20px', height: '20px', backgroundColor: '#90EE90', marginRight: '10px' }}></div>
              <span>Start Zone</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', marginBottom: '5px' }}>
              <div style={{ width: '20px', height: '20px', backgroundColor: '#2196F3', marginRight: '10px' }}></div>
              <span>Robot</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', marginBottom: '5px' }}>
              <div style={{ width: '20px', height: '20px', backgroundColor: '#FF9800', marginRight: '10px' }}></div>
              <span>Obstacle</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', marginBottom: '5px' }}>
              <div style={{ width: '20px', height: '20px', backgroundColor: '#FF5252', marginRight: '10px' }}></div>
              <span>Path</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', marginBottom: '5px' }}>
              <div style={{ width: '20px', height: '20px', backgroundColor: '#9C27B0', marginRight: '10px' }}></div>
              <span>Photo Position</span>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', marginBottom: '5px' }}>
              <div style={{ width: '20px', height: '20px', backgroundColor: 'rgba(255, 235, 59, 0.3)', marginRight: '10px' }}></div>
              <span>Camera FOV</span>
            </div>
          </div>
        </div>

        {/* Grid */}
        <div>
          <h3>Robot Path Animation</h3>
          <div style={{ 
            display: 'grid', 
            gridTemplateColumns: `repeat(${GRID_SIZE}, ${CELL_SIZE}px)`,
            gridTemplateRows: `repeat(${GRID_SIZE}, ${CELL_SIZE}px)`,
            border: '2px solid #333',
            width: 'fit-content',
            position: 'relative'
          }}>
            {grid.map((cell) => {
              const obstacle = getObstacleAt(cell.x, cell.y);
              const inStartZone = isStartZone(cell.x, cell.y);
              const inCameraFOV = isInCameraFOV(cell.x, cell.y);
              const isRobotCell = cell.x >= currentRobot.south_west.x && 
                                 cell.x <= currentRobot.north_east.x &&
                                 cell.y >= currentRobot.south_west.y && 
                                 cell.y <= currentRobot.north_east.y;
              
              // Check if this is a photo position
              const isPhotoPosition = photoPositions.some(photo => 
                photo.position.x === cell.x && photo.position.y === cell.y
              );
              
              // Check if this cell is in the path
              const pathIndex = pathPoints.findIndex(p => p.x === cell.x && p.y === cell.y);
              const isInPath = pathIndex !== -1 && 
                               animation.showPathTrail && 
                               pathIndex <= animation.currentStep;

              let bgColor = '#fff';
              if (isRobotCell) bgColor = '#2196F3';
              else if (inCameraFOV) bgColor = 'rgba(255, 235, 59, 0.3)';
              else if (isPhotoPosition) bgColor = '#9C27B0';
              else if (isInPath) bgColor = '#FF5252';
              else if (obstacle) bgColor = '#FF9800';
              else if (inStartZone) bgColor = '#90EE90';

              return (
                <div
                  key={`${cell.x}-${cell.y}`}
                  onClick={() => handleCellClick(cell.x, cell.y)}
                  style={{
                    width: CELL_SIZE,
                    height: CELL_SIZE,
                    backgroundColor: bgColor,
                    border: '1px solid #ddd',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    cursor: 'pointer',
                    position: 'relative',
                    fontSize: '12px'
                  }}
                  title={`(${cell.x}, ${cell.y})`}
                >
                  {/* Show robot direction */}
                  {isRobotCell && (
                    <div style={{
                      fontSize: '20px',
                      fontWeight: 'bold',
                      color: '#fff'
                    }}>
                      {currentRobot.direction === 'NORTH' && '↑'}
                      {currentRobot.direction === 'SOUTH' && '↓'}
                      {currentRobot.direction === 'EAST' && '→'}
                      {currentRobot.direction === 'WEST' && '←'}
                    </div>
                  )}
                  
                  {/* Show camera icon at camera position */}
                  {cell.x === cameraInfo.cameraPos.x && cell.y === cameraInfo.cameraPos.y && (
                    <div style={{
                      position: 'absolute',
                      top: '2px',
                      left: '2px',
                      fontSize: '14px',
                      color: '#000'
                    }}>
                      📷
                    </div>
                  )}
                  
                  {/* Show photo icon */}
                  {isPhotoPosition && !isRobotCell && (
                    <div style={{
                      fontSize: '16px'
                    }}>
                      📸
                    </div>
                  )}
                  
                  {/* Show obstacle direction */}
                  {obstacle && !isRobotCell && (
                    <div style={{
                      position: 'absolute',
                      top: '2px',
                      right: '2px',
                      fontSize: '10px',
                      fontWeight: 'bold',
                      color: '#333'
                    }}>
                      {obstacle.direction.charAt(0)}
                    </div>
                  )}
                </div>
              );
            })}
          </div>
          
          <div style={{ marginTop: '10px', fontSize: '14px', color: '#666' }}>
            <p>Step: {animation.currentStep}/{pathPoints.length - 1} | Robot: ({currentRobot.center?.x}, {currentRobot.center?.y}) {currentRobot.direction}</p>
            <p>Camera: ({cameraInfo.cameraPos.x}, {cameraInfo.cameraPos.y}) | Photos: {photoPositions.length}</p>
          </div>
        </div>
      </div>

      {/* Instructions Display */}
      {pathData && (
        <div style={{ marginTop: '30px', padding: '15px', backgroundColor: '#f0f0f0', borderRadius: '4px' }}>
          <h3>Instructions</h3>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: '10px' }}>
            {pathData.instructions.map((inst, idx) => (
              <div 
                key={idx}
                style={{
                  padding: '8px 12px',
                  backgroundColor: idx === animation.currentStep ? '#4CAF50' : '#fff',
                  color: idx === animation.currentStep ? '#fff' : '#333',
                  borderRadius: '4px',
                  border: '1px solid #ddd',
                  minWidth: '100px',
                  textAlign: 'center'
                }}
              >
                {typeof inst === 'string' ? inst : `${inst.move} ${inst.amount}cm`}
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};

export default GridVisualization;