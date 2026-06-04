import {
  GRID_SIZE,
  HIGH_VALUE_MAX_COL,
  FINISH_LABEL,
  ROW_LABELS,
  START_LABEL,
} from "./constants";
import type { Cell, CellKey } from "./types";

export function keyOf(cell: Cell): CellKey {
  return `${cell.r},${cell.c}`;
}

export function sameCell(a: Cell, b: Cell): boolean {
  return a.r === b.r && a.c === b.c;
}

export function cellFromLabel(label: string): Cell {
  const clean = label.trim().toUpperCase();
  const rowLetter = clean[0];
  const r = ROW_LABELS.indexOf(rowLetter) + 1;
  const c = Number(clean.slice(1));

  if (r < 1 || r > GRID_SIZE || c < 1 || c > GRID_SIZE) {
    throw new Error(`Bad cell label: ${label}`);
  }

  return { r, c };
}

export function labelOf(cell: Cell): string {
  return `${ROW_LABELS[cell.r - 1]}${cell.c}`;
}

export function isHighValueCell(cell: Cell): boolean {
  return cell.c <= HIGH_VALUE_MAX_COL;
}

export function seedScore(cell: Cell): number {
  return isHighValueCell(cell) ? 2 : 1;
}

export function wallDistance(cell: Cell): number {
  return Math.min(
    cell.r - 1,
    GRID_SIZE - cell.r,
    cell.c - 1,
    GRID_SIZE - cell.c,
  );
}

export function wallClosenessScore(cell: Cell): number {
  return GRID_SIZE - wallDistance(cell);
}

export function neighbours(cell: Cell): Cell[] {
  const out: Cell[] = [];

  const directions = [
    { dr: -1, dc: 0 },
    { dr: 0, dc: 1 },
    { dr: 1, dc: 0 },
    { dr: 0, dc: -1 },
  ];

  for (const d of directions) {
    const r = cell.r + d.dr;
    const c = cell.c + d.dc;

    if (r >= 1 && r <= GRID_SIZE && c >= 1 && c <= GRID_SIZE) {
      out.push({ r, c });
    }
  }

  return out;
}

export function allCells(): Cell[] {
  const cells: Cell[] = [];

  for (let r = 1; r <= GRID_SIZE; r++) {
    for (let c = 1; c <= GRID_SIZE; c++) {
      cells.push({ r, c });
    }
  }

  return cells;
}

export function countTurns(path: Cell[]): number {
  if (path.length < 3) return 0;

  let turns = 0;

  let lastDr = path[1].r - path[0].r;
  let lastDc = path[1].c - path[0].c;

  for (let i = 1; i < path.length - 1; i++) {
    const dr = path[i + 1].r - path[i].r;
    const dc = path[i + 1].c - path[i].c;

    if (dr !== lastDr || dc !== lastDc) {
      turns++;
    }

    lastDr = dr;
    lastDc = dc;
  }

  return turns;
}

export const START = cellFromLabel(START_LABEL);
export const FINISH = cellFromLabel(FINISH_LABEL);
