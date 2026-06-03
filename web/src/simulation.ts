import {
  FINISH_LABEL,
  GRID_SIZE,
  INITIAL_SERVER_REVEAL_COUNT,
  OBSTACLE_WAIT_MS,
  SEEDS_TO_PLANT,
} from "./constants";
import {
  allCells,
  FINISH,
  isHighValueCell,
  keyOf,
  labelOf,
  sameCell,
  START,
  wallDistance,
} from "./grid";
import { makeRng } from "./random";
import type {
  Cell,
  CellKey,
  DirectionName,
  KnownCell,
  Plan,
  StepOutcome,
  TruthCell,
  TruthEditMode,
} from "./types";

type RelativeTurn = "right" | "ahead" | "left" | "back";

interface MoveDecision {
  cell: Cell;
  heading: DirectionName;
  turn: RelativeTurn;
}

interface BlockedAhead {
  cell: Cell;
  truth: TruthCell;
}

const DIRECTION_ORDER: DirectionName[] = ["north", "east", "south", "west"];

const DIRECTION_DELTAS: Record<DirectionName, Cell> = {
  north: { r: -1, c: 0 },
  east: { r: 0, c: 1 },
  south: { r: 1, c: 0 },
  west: { r: 0, c: -1 },
};

const WALL_FOLLOW_TURNS: RelativeTurn[] = ["right", "ahead", "left", "back"];
const AHEAD_BLOCKED_TURNS: RelativeTurn[] = ["left", "back", "right"];

export class Simulation {
  currentSeed = 2026;
  truth = new Map<CellKey, TruthCell>();
  knowledge = new Map<CellKey, KnownCell>();
  robotPos: Cell = { ...START };
  heading: DirectionName = "north";
  plan: Plan = {
    objective: null,
    objectiveKind: "none",
    path: [],
    reason: "not planned yet",
    blockedCell: null,
  };
  plantedByRobot = 0;
  movesCompleted = 0;
  turnsCompleted = 0;
  forceReturn = false;
  waitingForCell: Cell | null = null;
  waitStartedAtMs: number | null = null;
  statusText = "Simulator not started.";

  private rand = makeRng(this.currentSeed);
  private navigationVersion = 0;
  private visitedNavigationStates = new Set<string>();

  resetSimulation(seed?: number): void {
    this.currentSeed = seed ?? Math.floor(Math.random() * 1_000_000_000);
    this.rand = makeRng(this.currentSeed);

    this.robotPos = { ...START };
    this.heading = "north";
    this.plantedByRobot = 0;
    this.movesCompleted = 0;
    this.turnsCompleted = 0;
    this.forceReturn = false;
    this.waitingForCell = null;
    this.waitStartedAtMs = null;
    this.navigationVersion = 0;
    this.visitedNavigationStates.clear();

    this.generateSecretTruthMap();
    this.initialiseBlankServerKnowledge();
    this.revealInitialServerCells();

    this.replan("simulation reset");
    this.rememberCurrentNavigationState();

    this.statusText =
      "New secret map generated. Robot starts at C9 facing north after the entry turn.";
  }

  triggerEmergencyReturn(): void {
    this.forceReturn = true;
    this.replan("emergency return");
    this.statusText = `Emergency return active. Robot will skip planting and keep following the right wall to ${FINISH_LABEL}.`;
  }

  stepOnce(cohortUpdatesEnabled: boolean, nowMs = Date.now()): StepOutcome {
    if (this.traversalComplete()) {
      this.markComplete();
      return { shouldStopAuto: true };
    }

    if (cohortUpdatesEnabled) {
      this.simulateOtherTeamUpdate();
    }

    if (this.waitingForCell) {
      return this.continueBlockedWait(nowMs);
    }

    const blockedAhead = this.blockedCellAhead();

    if (blockedAhead) {
      this.startBlockedWait(blockedAhead, nowMs);
      return { shouldStopAuto: false };
    }

    const decision = this.chooseMove(WALL_FOLLOW_TURNS);

    if (!decision) {
      this.plan = {
        objective: null,
        objectiveKind: "none",
        path: [this.robotPos],
        reason: "right-hand rule has no available move",
        blockedCell: null,
      };
      this.statusText = "No available move from current cell.";

      return { shouldStopAuto: true };
    }

    return this.moveByDecision(decision, "Right-hand rule step.");
  }

  replan(reason: string): void {
    this.refreshNavigationPlan(reason);
  }

  knownFertileCells(): Cell[] {
    return allCells().filter((cell) => {
      const known = this.knowledge.get(keyOf(cell));

      return (
        known !== undefined &&
        known.status === "fertile" &&
        !known.knownObstacle &&
        !known.needsRescue &&
        !known.planted
      );
    });
  }

  knownRescueCells(): Cell[] {
    return allCells().filter((cell) => {
      const known = this.knowledge.get(keyOf(cell));

      return (
        known !== undefined &&
        known.needsRescue &&
        !known.knownObstacle &&
        !sameCell(cell, START) &&
        !sameCell(cell, FINISH)
      );
    });
  }

  unknownCells(): Cell[] {
    return allCells().filter((cell) => {
      const known = this.knowledge.get(keyOf(cell));

      return (
        known !== undefined &&
        known.status === "unknown" &&
        !known.knownObstacle &&
        !known.needsRescue &&
        !sameCell(cell, START) &&
        !sameCell(cell, FINISH)
      );
    });
  }

  knownNonFertileCount(): number {
    let count = 0;

    for (const cell of allCells()) {
      const known = this.knowledge.get(keyOf(cell));
      if (known?.status === "nonfertile") count++;
    }

    return count;
  }

  knownObstacleCount(): number {
    let count = 0;

    for (const cell of allCells()) {
      const known = this.knowledge.get(keyOf(cell));
      if (known?.knownObstacle || known?.needsRescue) count++;
    }

    return count;
  }

  knownRescueCount(): number {
    return this.knownRescueCells().length;
  }

  serverRevealedCount(): number {
    let count = 0;

    for (const cell of allCells()) {
      const known = this.knowledge.get(keyOf(cell));
      if (known?.exploredByServer || known?.knownObstacle) count++;
    }

    return count;
  }

  exploredByThisRobotCount(): number {
    let count = 0;

    for (const cell of allCells()) {
      const known = this.knowledge.get(keyOf(cell));
      if (known?.exploredByThisRobot) count++;
    }

    return count;
  }

  observedFertilityProbability(): number {
    let fertile = 0;
    let nonFertile = 0;

    for (const cell of allCells()) {
      const known = this.knowledge.get(keyOf(cell));
      if (!known) continue;

      if (known.status === "fertile") fertile++;
      if (known.status === "nonfertile") nonFertile++;
    }

    const totalKnown = fertile + nonFertile;

    if (totalKnown === 0) {
      return 0.4;
    }

    return Math.max(0.15, Math.min(0.75, fertile / totalKnown));
  }

  waitRemainingMs(nowMs = Date.now()): number {
    if (!this.waitingForCell || this.waitStartedAtMs === null) return 0;

    return Math.max(0, OBSTACLE_WAIT_MS - (nowMs - this.waitStartedAtMs));
  }

  applyTruthEdit(cell: Cell, mode: TruthEditMode): void {
    const k = keyOf(cell);
    const t = this.truth.get(k);

    if (!t) return;

    const wasAlreadyTarget = truthCellMatchesEditMode(t, mode);

    if (mode === "planted" && !wasAlreadyTarget && !isPlainFertileTruthCell(t)) {
      this.statusText = `Cannot mark ${labelOf(cell)} planted because only fertile cells can be planted.`;
      return;
    }

    switch (mode) {
      case "fertile":
        setTruthCell(t, {
          fertile: !wasAlreadyTarget,
          obstacle: false,
          planted: false,
          needsRescue: false,
        });
        break;
      case "nonfertile":
        setTruthCell(t, {
          fertile: wasAlreadyTarget,
          obstacle: false,
          planted: false,
          needsRescue: false,
        });
        break;
      case "planted":
        setTruthCell(t, {
          fertile: true,
          obstacle: false,
          planted: !wasAlreadyTarget,
          needsRescue: false,
        });
        break;
      case "obstacle":
        setTruthCell(t, {
          fertile: false,
          obstacle: !wasAlreadyTarget,
          planted: false,
          needsRescue: false,
        });
        break;
      case "needs-rescue":
        setTruthCell(t, {
          fertile: false,
          obstacle: false,
          planted: false,
          needsRescue: !wasAlreadyTarget,
        });
        break;
    }

    this.waitingForCell = null;
    this.waitStartedAtMs = null;
    this.navigationVersion++;
    this.resetVisitedNavigationStates();

    if (t.needsRescue) {
      this.revealCellToServer(cell, this.knowledge.get(k)?.exploredByThisRobot ?? false);
    } else {
      this.refreshKnownCellAfterTruthEdit(cell);
    }

    this.replan(`secret map edited at ${labelOf(cell)}`);
    this.statusText = `Secret map edited: ${labelOf(cell)} is now ${truthCellLabel(t)}.`;
  }

  private startBlockedWait(blockedAhead: BlockedAhead, nowMs: number): void {
    this.revealCellToServer(blockedAhead.cell, false);
    this.waitingForCell = blockedAhead.cell;
    this.waitStartedAtMs = nowMs;
    this.refreshNavigationPlan("blocked cell ahead");

    const label = labelOf(blockedAhead.cell);
    const blocker = blockedAhead.truth.needsRescue
      ? "robot needing rescue"
      : "obstacle";

    this.statusText = `Detected ${blocker} ahead at ${label}. Waiting ${(OBSTACLE_WAIT_MS / 1000).toFixed(1)} seconds before reassessing.`;
  }

  private continueBlockedWait(nowMs: number): StepOutcome {
    const target = this.waitingForCell;

    if (!target || this.waitStartedAtMs === null) {
      this.waitingForCell = null;
      this.waitStartedAtMs = null;
      this.replan("wait state cleared");

      return { shouldStopAuto: false };
    }

    const remaining = this.waitRemainingMs(nowMs);

    if (remaining > 0) {
      this.refreshNavigationPlan("waiting for blocked cell");
      this.statusText = `Waiting for ${labelOf(target)} to clear. ${Math.ceil(remaining / 1000)}s remaining.`;

      return { shouldStopAuto: false };
    }

    this.waitingForCell = null;
    this.waitStartedAtMs = null;

    const blockedAhead = this.blockedCellAhead();

    if (!blockedAhead || !sameCell(blockedAhead.cell, target)) {
      this.replan(`${labelOf(target)} cleared after wait`);
      this.statusText = `${labelOf(target)} is clear after waiting. Continuing right-hand traversal.`;

      return { shouldStopAuto: false };
    }

    this.revealCellToServer(blockedAhead.cell, false);

    if (blockedAhead.truth.needsRescue && isEdgeCell(blockedAhead.cell)) {
      setTruthCell(blockedAhead.truth, {
        fertile: false,
        obstacle: false,
        planted: false,
        needsRescue: false,
      });
      this.navigationVersion++;
      this.resetVisitedNavigationStates();
      this.revealCellToServer(blockedAhead.cell, true);
      this.replan("edge rescue revived");
      this.statusText = `Revived edge robot at ${labelOf(blockedAhead.cell)} from ${labelOf(this.robotPos)}. Continuing without turning around.`;

      return { shouldStopAuto: false };
    }

    const decision = this.chooseMove(AHEAD_BLOCKED_TURNS);

    if (!decision) {
      this.plan = {
        objective: blockedAhead.cell,
        objectiveKind: "wait-blocked",
        path: [this.robotPos],
        reason: "blocked ahead after wait with no route around",
        blockedCell: blockedAhead.cell,
      };
      this.statusText = `${labelOf(blockedAhead.cell)} is still blocked and no adjacent move is available.`;

      return { shouldStopAuto: true };
    }

    const blocker = blockedAhead.truth.needsRescue
      ? "Robot needing rescue"
      : "Obstacle";

    return this.moveByDecision(
      decision,
      `${blocker} remains at ${labelOf(blockedAhead.cell)} after waiting; going around it.`,
    );
  }

  private moveByDecision(decision: MoveDecision, prefix: string): StepOutcome {
    const previousHeading = this.heading;

    if (decision.heading !== previousHeading) {
      this.turnsCompleted++;
    }

    this.heading = decision.heading;
    this.movesCompleted++;
    this.robotPos = decision.cell;
    this.revealCellToServer(this.robotPos, true);

    const movementText = movementDescription(decision.turn, this.robotPos);
    const arrivalText = this.handleArrivalAtCurrentCell();
    this.statusText = `${prefix} ${movementText} ${arrivalText}`;

    if (this.traversalComplete()) {
      this.markComplete();
      return { shouldStopAuto: true };
    }

    if (sameCell(this.robotPos, FINISH)) {
      this.statusText += ` Reached ${FINISH_LABEL} before the stop condition; continuing traversal.`;
    }

    this.replan("right-hand rule step");

    if (this.hasRepeatedNavigationState()) {
      this.plan = {
        objective: null,
        objectiveKind: "none",
        path: [this.robotPos],
        reason: "right-hand traversal repeated before completion",
        blockedCell: null,
      };
      this.statusText = `Traversal repeated at ${labelOf(this.robotPos)} facing ${this.heading} before completion. Stopping to avoid an infinite loop.`;

      return { shouldStopAuto: true };
    }

    return { shouldStopAuto: false };
  }

  private handleArrivalAtCurrentCell(): string {
    const truth = this.truth.get(keyOf(this.robotPos));
    const known = this.knowledge.get(keyOf(this.robotPos));

    if (!truth || !known) {
      return `Reached ${labelOf(this.robotPos)}.`;
    }

    if (
      !this.forceReturn &&
      known.status === "fertile" &&
      !known.planted &&
      this.plantedByRobot < SEEDS_TO_PLANT
    ) {
      truth.planted = true;
      known.planted = true;
      this.plantedByRobot++;
      this.navigationVersion++;

      return `Planted seed ${this.plantedByRobot}/${SEEDS_TO_PLANT} at ${labelOf(this.robotPos)}.`;
    }

    if (known.status === "fertile" && known.planted) {
      return `Reached ${labelOf(this.robotPos)}; it is fertile but already planted.`;
    }

    if (known.status === "fertile" && this.forceReturn) {
      return `Reached ${labelOf(this.robotPos)}; emergency return is active, so planting is skipped.`;
    }

    if (known.status === "fertile" && this.plantedByRobot >= SEEDS_TO_PLANT) {
      return `Reached ${labelOf(this.robotPos)}; seed target is complete, so planting is skipped.`;
    }

    if (known.status === "nonfertile") {
      return `Explored ${labelOf(this.robotPos)}; cell is non-fertile.`;
    }

    return `Moved to ${labelOf(this.robotPos)}.`;
  }

  private refreshNavigationPlan(reason: string): void {
    if (this.traversalComplete()) {
      this.plan = {
        objective: this.robotPos,
        objectiveKind: "complete",
        path: [this.robotPos],
        reason,
        blockedCell: null,
      };

      return;
    }

    if (this.waitingForCell) {
      this.plan = {
        objective: this.waitingForCell,
        objectiveKind: "wait-blocked",
        path: [this.robotPos],
        reason,
        blockedCell: this.waitingForCell,
      };

      return;
    }

    const blockedAhead = this.blockedCellAhead();

    if (blockedAhead) {
      this.plan = {
        objective: blockedAhead.cell,
        objectiveKind: "wait-blocked",
        path: [this.robotPos],
        reason,
        blockedCell: blockedAhead.cell,
      };

      return;
    }

    const decision = this.chooseMove(WALL_FOLLOW_TURNS);

    if (!decision) {
      this.plan = {
        objective: null,
        objectiveKind: "none",
        path: [this.robotPos],
        reason,
        blockedCell: null,
      };

      return;
    }

    this.plan = {
      objective: decision.cell,
      objectiveKind: "follow-right-wall",
      path: [this.robotPos, decision.cell],
      reason,
      blockedCell: null,
    };
  }

  private chooseMove(turns: RelativeTurn[]): MoveDecision | null {
    for (const turn of turns) {
      const heading = headingForTurn(this.heading, turn);
      const cell = cellInDirection(this.robotPos, heading);

      if (this.cellIsOpen(cell)) {
        return { cell, heading, turn };
      }
    }

    return null;
  }

  private blockedCellAhead(): BlockedAhead | null {
    const cell = cellInDirection(this.robotPos, this.heading);

    if (!inBounds(cell)) return null;

    const truth = this.truth.get(keyOf(cell));

    if (!truth?.obstacle && !truth?.needsRescue) return null;

    return { cell, truth };
  }

  private cellIsOpen(cell: Cell): boolean {
    if (!inBounds(cell)) return false;

    const truth = this.truth.get(keyOf(cell));

    return truth !== undefined && !truth.obstacle && !truth.needsRescue;
  }

  private traversalComplete(): boolean {
    return (
      sameCell(this.robotPos, FINISH) &&
      (this.forceReturn || this.plantedByRobot >= SEEDS_TO_PLANT)
    );
  }

  private markComplete(): void {
    this.plan = {
      objective: this.robotPos,
      objectiveKind: "complete",
      path: [this.robotPos],
      reason: "traversal complete",
      blockedCell: null,
    };

    this.statusText = this.forceReturn
      ? `Emergency traversal complete: robot reached ${FINISH_LABEL}.`
      : `Mission complete: planted ${this.plantedByRobot}/${SEEDS_TO_PLANT} and reached ${FINISH_LABEL} by right-hand traversal.`;
  }

  private hasRepeatedNavigationState(): boolean {
    const state = this.navigationStateKey();

    if (this.visitedNavigationStates.has(state)) {
      return true;
    }

    this.visitedNavigationStates.add(state);
    return false;
  }

  private rememberCurrentNavigationState(): void {
    this.visitedNavigationStates.add(this.navigationStateKey());
  }

  private resetVisitedNavigationStates(): void {
    this.visitedNavigationStates.clear();
    this.rememberCurrentNavigationState();
  }

  private navigationStateKey(): string {
    return `${keyOf(this.robotPos)}|${this.heading}|${this.navigationVersion}`;
  }

  private generateSecretTruthMap(): void {
    this.truth.clear();

    const firstMoveCell = cellInDirection(START, "north");

    for (const cell of allCells()) {
      const protectedCell =
        sameCell(cell, START) ||
        sameCell(cell, FINISH) ||
        sameCell(cell, firstMoveCell);

      const obstacleChance = protectedCell ? 0 : 0.07;
      const obstacle = this.rand() < obstacleChance;

      const fertilityBase = isHighValueCell(cell) ? 0.48 : 0.34;
      const wallBonus = wallDistance(cell) <= 1 ? 0.08 : 0;
      const fertile = !obstacle && this.rand() < fertilityBase + wallBonus;

      this.truth.set(keyOf(cell), {
        fertile,
        obstacle,
        planted: false,
        needsRescue: false,
      });
    }
  }

  private initialiseBlankServerKnowledge(): void {
    this.knowledge.clear();

    for (const cell of allCells()) {
      this.knowledge.set(keyOf(cell), defaultKnownCell());
    }
  }

  private revealCellToServer(cell: Cell, byThisRobot: boolean): void {
    const k = keyOf(cell);
    const t = this.truth.get(k);
    const known = this.knowledge.get(k) ?? defaultKnownCell();

    if (!t) return;

    if (t.obstacle) {
      known.knownObstacle = true;
      known.status = "unknown";
      known.planted = false;
      known.needsRescue = false;
    } else {
      known.knownObstacle = false;
      known.status = t.fertile ? "fertile" : "nonfertile";
      known.planted = t.planted;
      known.needsRescue = t.needsRescue;
    }

    known.exploredByServer = true;

    if (byThisRobot) {
      known.exploredByThisRobot = true;
    }

    this.knowledge.set(k, known);
  }

  private refreshKnownCellAfterTruthEdit(cell: Cell): void {
    const known = this.knowledge.get(keyOf(cell));

    if (!known) return;
    if (!known.exploredByServer && !known.knownObstacle) return;

    this.revealCellToServer(cell, known.exploredByThisRobot);
  }

  private revealInitialServerCells(): void {
    const candidates = allCells().filter((cell) => {
      const t = this.truth.get(keyOf(cell));
      return t !== undefined && !t.obstacle;
    });

    for (let i = 0; i < INITIAL_SERVER_REVEAL_COUNT && candidates.length > 0; i++) {
      const index = Math.floor(this.rand() * candidates.length);
      const [cell] = candidates.splice(index, 1);
      this.revealCellToServer(cell, false);
    }

    this.revealCellToServer(START, true);
    this.revealCellToServer(FINISH, false);
  }

  private simulateOtherTeamUpdate(): void {
    const candidates = this.unknownCells().filter((cell) => {
      const t = this.truth.get(keyOf(cell));
      return t !== undefined && !t.obstacle;
    });

    if (candidates.length === 0) return;

    const cell = candidates[Math.floor(this.rand() * candidates.length)];
    const t = this.truth.get(keyOf(cell));

    if (!t) return;

    if (t.fertile && !t.planted && this.rand() < 0.2) {
      t.planted = true;
    }

    this.revealCellToServer(cell, false);
  }
}

function defaultKnownCell(): KnownCell {
  return {
    status: "unknown",
    knownObstacle: false,
    planted: false,
    needsRescue: false,
    exploredByServer: false,
    exploredByThisRobot: false,
  };
}

function cellInDirection(cell: Cell, heading: DirectionName): Cell {
  const delta = DIRECTION_DELTAS[heading];

  return {
    r: cell.r + delta.r,
    c: cell.c + delta.c,
  };
}

function headingForTurn(
  heading: DirectionName,
  turn: RelativeTurn,
): DirectionName {
  const index = DIRECTION_ORDER.indexOf(heading);

  if (turn === "right") return DIRECTION_ORDER[(index + 1) % DIRECTION_ORDER.length];
  if (turn === "left") return DIRECTION_ORDER[(index + 3) % DIRECTION_ORDER.length];
  if (turn === "back") return DIRECTION_ORDER[(index + 2) % DIRECTION_ORDER.length];

  return heading;
}

function inBounds(cell: Cell): boolean {
  return (
    cell.r >= 1 &&
    cell.r <= GRID_SIZE &&
    cell.c >= 1 &&
    cell.c <= GRID_SIZE
  );
}

function isEdgeCell(cell: Cell): boolean {
  return (
    cell.r === 1 ||
    cell.r === GRID_SIZE ||
    cell.c === 1 ||
    cell.c === GRID_SIZE
  );
}

function movementDescription(turn: RelativeTurn, cell: Cell): string {
  switch (turn) {
    case "right":
      return `Turned right and moved to ${labelOf(cell)}.`;
    case "left":
      return `Turned left and moved to ${labelOf(cell)}.`;
    case "back":
      return `Turned around and moved to ${labelOf(cell)}.`;
    case "ahead":
      return `Moved ahead to ${labelOf(cell)}.`;
  }
}

function isPlainFertileTruthCell(cell: TruthCell): boolean {
  return cell.fertile && !cell.obstacle && !cell.planted && !cell.needsRescue;
}

function truthCellMatchesEditMode(cell: TruthCell, mode: TruthEditMode): boolean {
  switch (mode) {
    case "fertile":
      return isPlainFertileTruthCell(cell);
    case "nonfertile":
      return !cell.fertile && !cell.obstacle && !cell.planted && !cell.needsRescue;
    case "planted":
      return cell.planted;
    case "obstacle":
      return cell.obstacle;
    case "needs-rescue":
      return cell.needsRescue;
  }
}

function setTruthCell(cell: TruthCell, next: TruthCell): void {
  cell.fertile = next.fertile;
  cell.obstacle = next.obstacle;
  cell.planted = next.planted;
  cell.needsRescue = next.needsRescue;
}

function truthCellLabel(cell: TruthCell): string {
  if (cell.obstacle) return "an obstacle";
  if (cell.needsRescue) return "needs rescue";
  if (cell.planted) return "planted";
  if (cell.fertile) return "fertile";
  return "unfertile";
}
