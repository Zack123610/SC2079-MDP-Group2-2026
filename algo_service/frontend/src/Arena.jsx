import './Arena.css'

const ARENA_SIZE = 20

// Grid: x 0..19 left→right, y 0..19 bottom→top (NORTH = +y = up on screen)
// Row index in DOM: row = 19 - y (so y=0 is bottom row, y=19 is top row)

function cellKey(x, y) {
  return `${x},${y}`
}

function Arena({ robot, obstacles, overrideRobot, pathTrail = [] }) {
  const displayRobot = overrideRobot ?? robot
  const sw = displayRobot?.south_west ?? { x: 0, y: 0 }
  const direction = displayRobot?.direction ?? 'NORTH'
  const robotCells = new Set()
  for (let dx = 0; dx < 3; dx++) {
    for (let dy = 0; dy < 3; dy++) {
      robotCells.add(cellKey(sw.x + dx, sw.y + dy))
    }
  }

  const pathTrailSet = new Set(pathTrail.map((p) => cellKey(p.sx ?? p.x, p.sy ?? p.y)))

  const obstacleMap = new Map()
  obstacles?.forEach((obs) => {
    const ox = obs.south_west?.x ?? 0
    const oy = obs.south_west?.y ?? 0
    obstacleMap.set(cellKey(ox, oy), { id: obs.image_id, face: obs.direction })
  })

  const rows = []
  for (let r = ARENA_SIZE - 1; r >= 0; r--) {
    const y = r
    const cells = []
    for (let x = 0; x < ARENA_SIZE; x++) {
      const key = cellKey(x, y)
      const isRobot = robotCells.has(key)
      const isTrail = pathTrailSet.has(key) && !isRobot
      const obs = obstacleMap.get(key)
      const isCenterRobot = isRobot && x === sw.x + 1 && y === sw.y + 1
      cells.push(
        <div
          key={key}
          className={`arena-cell ${isRobot ? 'robot' : ''} ${obs ? 'obstacle' : ''} ${isTrail ? 'trail' : ''}`}
          data-x={x}
          data-y={y}
        >
          {obs && <span className={`obstacle-face obstacle-face-${obs.face}`} title={`Obstacle ${obs.id} (image on ${obs.face})`} />}
          {isCenterRobot && <span className={`robot-direction robot-direction-${direction}`} title={direction} />}
        </div>
      )
    }
    rows.push(
      <div key={r} className="arena-row">
        {cells}
      </div>
    )
  }

  return (
    <div className="arena-wrap">
      <div className="arena-grid">
        {rows}
      </div>
      <div className="arena-legend">
        <span className="legend-item"><span className="legend-robot" /> Robot</span>
        <span className="legend-item"><span className="legend-obstacle" /> Obstacle</span>
        <span className="legend-item"><span className="legend-trail" /> Path</span>
      </div>
    </div>
  )
}

export default Arena
