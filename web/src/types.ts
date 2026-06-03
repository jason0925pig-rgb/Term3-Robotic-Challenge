export type Cell = {
  r: number;
  c: number;
};

export type CellKey = string;

export type KnowledgeStatus = "unknown" | "fertile" | "nonfertile";

export type ObjectiveKind =
  | "follow-right-wall"
  | "wait-blocked"
  | "complete"
  | "none";

export type MapMode = "truth" | "server" | "robot";

export type DirectionName = "north" | "east" | "south" | "west";

export type TruthEditMode =
  | "fertile"
  | "nonfertile"
  | "planted"
  | "obstacle"
  | "needs-rescue";

export interface TruthCell {
  fertile: boolean;
  obstacle: boolean;
  planted: boolean;
  needsRescue: boolean;
}

export interface KnownCell {
  status: KnowledgeStatus;
  knownObstacle: boolean;
  planted: boolean;
  needsRescue: boolean;
  exploredByServer: boolean;
  exploredByThisRobot: boolean;
}

export interface Plan {
  objective: Cell | null;
  objectiveKind: ObjectiveKind;
  path: Cell[];
  reason: string;
  blockedCell?: Cell | null;
}

export interface StepOutcome {
  shouldStopAuto: boolean;
}
