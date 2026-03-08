import { useState, useMemo, useEffect, useCallback } from 'react'
import Arena from './Arena'
import { buildSteps, formatInstruction } from './simulation'
import './App.css'

const DIRECTIONS = ['NORTH', 'SOUTH', 'EAST', 'WEST']
const ARENA_SIZE = 20

const defaultRobot = () => ({
  direction: 'NORTH',
  south_west: { x: 0, y: 0 },
  north_east: { x: 3, y: 3 },
})

const defaultObstacle = (id) => ({
  image_id: id,
  direction: 'NORTH',
  south_west: { x: 10, y: 10 },
  north_east: { x: 11, y: 11 },
})

function App() {
  const [robot, setRobot] = useState(defaultRobot())
  const [obstacles, setObstacles] = useState([defaultObstacle(1)])
  const [apiBase, setApiBase] = useState('/api')
  const [loading, setLoading] = useState(false)
  const [result, setResult] = useState(null)
  const [error, setError] = useState(null)
  const [stepIndex, setStepIndex] = useState(0)
  const [playing, setPlaying] = useState(false)
  const [speed, setSpeed] = useState(800)

  const steps = useMemo(() => (result?.segments ? buildSteps(robot, result.segments) : []), [result, robot])
  const maxStepIndex = Math.max(0, steps.length - 1)

  const currentPosition = steps[stepIndex]?.position
  const overrideRobot = currentPosition
    ? {
        south_west: { x: currentPosition.sx, y: currentPosition.sy },
        direction: currentPosition.heading,
      }
    : undefined

  const pathTrail = useMemo(() => {
    const out = []
    for (let i = 0; i < stepIndex; i++) {
      const p = steps[i]?.position
      if (!p) continue
      for (let dx = 0; dx < 3; dx++) for (let dy = 0; dy < 3; dy++) out.push({ sx: p.sx + dx, sy: p.sy + dy })
    }
    return out
  }, [steps, stepIndex])

  useEffect(() => {
    if (!playing || stepIndex >= maxStepIndex) {
      if (playing && stepIndex >= maxStepIndex) setPlaying(false)
      return
    }
    const t = setTimeout(() => setStepIndex((i) => Math.min(i + 1, maxStepIndex)), speed)
    return () => clearTimeout(t)
  }, [playing, stepIndex, maxStepIndex, speed])

  const updateRobot = useCallback((field, value) => {
    setRobot((prev) => {
      const next = { ...prev }
      if (field === 'direction') next.direction = value
      else if (field.startsWith('sw_')) next.south_west = { ...prev.south_west, [field.slice(3)]: Number(value) }
      return next
    })
  }, [])

  const addObstacle = useCallback(() => {
    const nextId = obstacles.length ? Math.max(...obstacles.map((o) => o.image_id)) + 1 : 1
    setObstacles((prev) => [...prev, defaultObstacle(nextId)])
  }, [obstacles.length])

  const removeObstacle = useCallback((index) => {
    setObstacles((prev) => prev.filter((_, i) => i !== index))
  }, [])

  const updateObstacle = useCallback((index, field, value) => {
    setObstacles((prev) =>
      prev.map((o, i) => {
        if (i !== index) return o
        const next = { ...o }
        if (field === 'image_id') next.image_id = Number(value)
        else if (field === 'direction') next.direction = value
        else if (field.startsWith('sw_')) next.south_west = { ...o.south_west, [field.slice(3)]: Number(value) }
        return next
      })
    )
  }, [])

  const requestPath = async () => {
    setError(null)
    setResult(null)
    setStepIndex(0)
    setPlaying(false)
    setLoading(true)
    try {
      const res = await fetch(`${apiBase}/pathfinding`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ robot, obstacles, verbose: true }),
      })
      const data = await res.json()
      if (!res.ok) {
        setError(data.error || `HTTP ${res.status}`)
        return
      }
      setResult(data)
    } catch (e) {
      setError(e.message || 'Request failed')
    } finally {
      setLoading(false)
    }
  }

  const goToStep = (i) => setStepIndex(Math.max(0, Math.min(i, maxStepIndex)))

  return (
    <div className="app">
      <header className="header">
        <div className="header-inner">
          <h1>Task 1 Pathfinding</h1>
          <p className="tagline">20×20 arena · 3×3 robot · step-by-step execution</p>
        </div>
      </header>

      <main className="main">
        <section className="arena-section">
          <div className="arena-card">
            <Arena
              robot={robot}
              obstacles={obstacles}
              overrideRobot={overrideRobot}
              pathTrail={pathTrail}
            />
          </div>

          {steps.length > 0 && (
            <div className="execution-card">
              <h3>Execution</h3>
              <div className="execution-controls">
                <button type="button" className="ctrl-btn" onClick={() => goToStep(0)} disabled={stepIndex === 0} title="First">⏮</button>
                <button type="button" className="ctrl-btn" onClick={() => goToStep(stepIndex - 1)} disabled={stepIndex === 0} title="Previous">◀</button>
                <button
                  type="button"
                  className="ctrl-btn ctrl-btn-play"
                  onClick={() => setPlaying((p) => !p)}
                  disabled={stepIndex >= maxStepIndex}
                  title={playing ? 'Pause' : 'Play'}
                >
                  {playing ? '⏸' : '▶'}
                </button>
                <button type="button" className="ctrl-btn" onClick={() => goToStep(stepIndex + 1)} disabled={stepIndex >= maxStepIndex} title="Next">▶</button>
                <button type="button" className="ctrl-btn" onClick={() => goToStep(maxStepIndex)} disabled={stepIndex >= maxStepIndex} title="Last">⏭</button>
                <label className="speed-label">
                  Speed
                  <select value={speed} onChange={(e) => setSpeed(Number(e.target.value))}>
                    <option value={200}>Fast</option>
                    <option value={500}>Medium</option>
                    <option value={800}>Normal</option>
                    <option value={1500}>Slow</option>
                  </select>
                </label>
              </div>
              <div className="step-slider-wrap">
                <input
                  type="range"
                  min={0}
                  max={maxStepIndex}
                  value={stepIndex}
                  onChange={(e) => goToStep(Number(e.target.value))}
                  className="step-slider"
                />
                <span className="step-counter">Step {stepIndex + 1} / {steps.length}</span>
              </div>
              <div className="instruction-list">
                {steps.map((s, i) => (
                  <div
                    key={s.stepIndex}
                    className={`instruction-item ${i === stepIndex ? 'current' : ''} ${i < stepIndex ? 'done' : ''}`}
                    onClick={() => goToStep(i)}
                  >
                    <span className="instruction-num">{i + 1}</span>
                    <span className="instruction-text">{formatInstruction(s.instruction)}</span>
                    {s.obstacleId != null && <span className="instruction-obstacle">Obstacle {s.obstacleId}</span>}
                  </div>
                ))}
              </div>
            </div>
          )}
        </section>

        <aside className="sidebar">
          <div className="card config-card">
            <h2>Setup</h2>
            <div className="field">
              <label>Robot direction</label>
              <select value={robot.direction} onChange={(e) => updateRobot('direction', e.target.value)}>
                {DIRECTIONS.map((d) => <option key={d} value={d}>{d}</option>)}
              </select>
            </div>
            <div className="field-row">
              <div className="field">
                <label>Start X</label>
                <input type="number" min={0} max={ARENA_SIZE - 3} value={robot.south_west.x} onChange={(e) => updateRobot('sw_x', e.target.value)} />
              </div>
              <div className="field">
                <label>Start Y</label>
                <input type="number" min={0} max={ARENA_SIZE - 3} value={robot.south_west.y} onChange={(e) => updateRobot('sw_y', e.target.value)} />
              </div>
            </div>

            <h2>Obstacles</h2>
            {obstacles.map((obs, i) => (
              <div key={i} className="obstacle-card">
                <div className="obstacle-card-head">
                  <span>Obstacle {obs.image_id}</span>
                  <button type="button" className="btn-icon" onClick={() => removeObstacle(i)} title="Remove">×</button>
                </div>
                <div className="field">
                  <label>Image face</label>
                  <select value={obs.direction} onChange={(e) => updateObstacle(i, 'direction', e.target.value)}>
                    {DIRECTIONS.map((d) => <option key={d} value={d}>{d}</option>)}
                  </select>
                </div>
                <div className="field-row">
                  <div className="field">
                    <label>X</label>
                    <input type="number" min={0} max={ARENA_SIZE - 1} value={obs.south_west.x} onChange={(e) => updateObstacle(i, 'sw_x', e.target.value)} />
                  </div>
                  <div className="field">
                    <label>Y</label>
                    <input type="number" min={0} max={ARENA_SIZE - 1} value={obs.south_west.y} onChange={(e) => updateObstacle(i, 'sw_y', e.target.value)} />
                  </div>
                </div>
              </div>
            ))}
            <button type="button" className="btn btn-add" onClick={addObstacle}>+ Add obstacle</button>

            <div className="field api-field">
              <label>API base</label>
              <input type="text" value={apiBase} onChange={(e) => setApiBase(e.target.value)} placeholder="/api" />
            </div>
            <button type="button" className="btn btn-primary" onClick={requestPath} disabled={loading || !obstacles.length}>
              {loading ? 'Computing…' : 'Get path'}
            </button>
          </div>

          {error && <div className="card error-card">{error}</div>}
        </aside>
      </main>
    </div>
  )
}

export default App
