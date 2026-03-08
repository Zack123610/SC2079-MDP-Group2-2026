/**
 * Step-by-step simulation: compute robot position and heading after each instruction.
 * Matches backend: FORWARD/BACKWARD in cm; arc moves (2 fwd, 3 side) and update heading 90°.
 * At CAPTURE_IMAGE the robot is facing the obstacle (heading = obstacle face).
 */

const UNIT_CM = 10

const FORWARD_DELTA = {
  NORTH: [0, 1],
  SOUTH: [0, -1],
  EAST: [1, 0],
  WEST: [-1, 0],
}

const LEFT_DELTA = {
  NORTH: [-1, 0],
  SOUTH: [1, 0],
  EAST: [0, 1],
  WEST: [0, -1],
}

const RIGHT_DELTA = {
  NORTH: [1, 0],
  SOUTH: [-1, 0],
  EAST: [0, -1],
  WEST: [0, 1],
}

// After arc move: 90° turn. FORWARD_LEFT/BACKWARD_LEFT → turn left; FORWARD_RIGHT/BACKWARD_RIGHT → turn right.
const HEADING_AFTER_LEFT = { NORTH: 'WEST', WEST: 'SOUTH', SOUTH: 'EAST', EAST: 'NORTH' }
const HEADING_AFTER_RIGHT = { NORTH: 'EAST', EAST: 'SOUTH', SOUTH: 'WEST', WEST: 'NORTH' }

function arcDelta(heading, forward, left) {
  const fd = FORWARD_DELTA[heading]
  const ld = left ? LEFT_DELTA[heading] : RIGHT_DELTA[heading]
  const sign = forward ? 1 : -1
  return [
    sign * 2 * fd[0] + 3 * ld[0],
    sign * 2 * fd[1] + 3 * ld[1],
  ]
}

/**
 * Apply a single instruction; returns new { sx, sy, heading }.
 * Arc moves update heading (90° left or right).
 */
export function applyInstruction(sx, sy, heading, instruction) {
  if (typeof instruction === 'string') {
    if (instruction === 'CAPTURE_IMAGE') return { sx, sy, heading }
    const forward = instruction.startsWith('FORWARD')
    const left = instruction.includes('LEFT')
    const [dx, dy] = arcDelta(heading, forward, left)
    const newHeading = left ? HEADING_AFTER_LEFT[heading] : HEADING_AFTER_RIGHT[heading]
    return { sx: sx + dx, sy: sy + dy, heading: newHeading }
  }
  if (instruction?.move === 'FORWARD' && typeof instruction.amount === 'number') {
    const units = instruction.amount / UNIT_CM
    const [dx, dy] = FORWARD_DELTA[heading]
    return { sx: sx + dx * units, sy: sy + dy * units, heading }
  }
  if (instruction?.move === 'BACKWARD' && typeof instruction.amount === 'number') {
    const units = instruction.amount / UNIT_CM
    const [dx, dy] = FORWARD_DELTA[heading]
    return { sx: sx - dx * units, sy: sy - dy * units, heading }
  }
  return { sx, sy, heading }
}

/**
 * Flatten segments into a list of steps with positions.
 * Each step: { stepIndex, obstacleId, instruction, position: { sx, sy, heading } }.
 * First step is start position (instruction null).
 */
export function buildSteps(robot, segments) {
  if (!segments?.length) return []
  const steps = []
  let sx = robot.south_west?.x ?? 0
  let sy = robot.south_west?.y ?? 0
  let heading = robot.direction ?? 'NORTH'

  steps.push({ stepIndex: 0, obstacleId: null, instruction: null, position: { sx, sy, heading } })

  let stepIndex = 1
  for (const seg of segments) {
    const obstacleId = seg.image_id
    for (const instr of seg.instructions ?? []) {
      const next = applyInstruction(sx, sy, heading, instr)
      sx = Math.round(next.sx)
      sy = Math.round(next.sy)
      heading = next.heading
      steps.push({ stepIndex: stepIndex++, obstacleId, instruction: instr, position: { sx, sy, heading } })
    }
  }
  return steps
}

export function formatInstruction(instr) {
  if (instr == null) return 'Start'
  if (typeof instr === 'string') return instr
  if (instr?.move && typeof instr.amount === 'number') return `${instr.move} ${instr.amount} cm`
  return JSON.stringify(instr)
}
