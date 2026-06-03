import { GRID_SIZE, OBSTACLE_WAIT_MS, SEEDS_TO_PLANT } from "./constants";
import { labelOf } from "./grid";
import type { Simulation } from "./simulation";

export function renderStatusHtml(sim: Simulation): string {
  const knownF = sim.knownFertileCells().length;
  const knownNF = sim.knownNonFertileCount();
  const unknown = sim.unknownCells().length;
  const obstacles = sim.knownObstacleCount();
  const rescues = sim.knownRescueCount();
  const revealed = sim.serverRevealedCount();
  const robotExplored = sim.exploredByThisRobotCount();

  const pathSteps = Math.max(0, sim.plan.path.length - 1);
  const totalCells = GRID_SIZE * GRID_SIZE;

  return `
    <div class="status-card">
      <span class="status-label">Mission</span>
      <div class="status-value">
        ${modeText(sim)}<br>
        Seeds: ${sim.plantedByRobot}/${SEEDS_TO_PLANT}<br>
        Robot: ${labelOf(sim.robotPos)}<br>
        Heading: ${headingText(sim.heading)}<br>
        Moves: ${sim.movesCompleted}, turns: ${sim.turnsCompleted}
      </div>
    </div>

    <div class="status-card">
      <span class="status-label">Server map</span>
      <div class="status-value">
        Revealed: ${revealed}/${totalCells}<br>
        Fertile: ${knownF}, non-fertile: ${knownNF}<br>
        Unknown: ${unknown}, blocked: ${obstacles}<br>
        Robots needing rescue: ${rescues}
      </div>
    </div>

    <div class="status-card">
      <span class="status-label">Navigation</span>
      <div class="status-value">
        State: ${navigationStateText(sim)}<br>
        Next: ${objectiveText(sim)}<br>
        Preview steps: ${pathSteps}<br>
        Wait: ${waitText(sim)}
      </div>
    </div>

    <div class="status-card">
      <span class="status-label">Run state</span>
      <div class="status-value">
        Seed: ${sim.currentSeed}<br>
        This robot revealed: ${robotExplored}<br>
        Plan reason: ${sim.plan.reason}
      </div>
    </div>

    <div class="status-card status-wide">
      <span class="status-label">Latest event</span>
      <div class="status-value">${sim.statusText}</div>
    </div>
  `;
}

function objectiveText(sim: Simulation): string {
  if (!sim.plan.objective) return "none";

  const label = labelOf(sim.plan.objective);

  if (sim.plan.objectiveKind === "follow-right-wall") {
    return `${label} by right-hand rule`;
  }

  if (sim.plan.objectiveKind === "wait-blocked") {
    return `${label} blocked ahead`;
  }

  if (sim.plan.objectiveKind === "complete") {
    return `${label} complete`;
  }

  return label;
}

function navigationStateText(sim: Simulation): string {
  switch (sim.plan.objectiveKind) {
    case "follow-right-wall":
      return "following right wall";
    case "wait-blocked":
      return "waiting / blocked ahead";
    case "complete":
      return "complete";
    case "none":
      return "no move";
  }
}

function modeText(sim: Simulation): string {
  if (sim.forceReturn) return "Emergency wall-follow";
  if (sim.plantedByRobot >= SEEDS_TO_PLANT) return "Finish traversal";
  return "Explore / plant by wall";
}

function waitText(sim: Simulation): string {
  if (!sim.waitingForCell) return "none";

  const remaining = sim.waitRemainingMs();
  const seconds = Math.ceil(remaining / 1000);

  return `${labelOf(sim.waitingForCell)} (${seconds}s of ${(OBSTACLE_WAIT_MS / 1000).toFixed(1)}s left)`;
}

function headingText(heading: string): string {
  return heading[0].toUpperCase() + heading.slice(1);
}
