import type p5 from "p5";
import {
  CELL_PX,
  GRID_SIZE,
  HEADER_PX,
  MAP_GAP_PX,
  PAGE_PADDING_PX,
  FINISH_LABEL,
  ROW_LABELS,
  START_LABEL,
} from "./constants";
import {
  allCells,
  FINISH,
  keyOf,
  sameCell,
  START,
} from "./grid";
import type { Simulation } from "./simulation";
import type { Cell, MapMode } from "./types";

export function singleMapWidth(): number {
  return HEADER_PX + GRID_SIZE * CELL_PX;
}

export function canvasWidth(): number {
  return PAGE_PADDING_PX * 2 + 3 * singleMapWidth() + 2 * MAP_GAP_PX;
}

export function canvasHeight(): number {
  return PAGE_PADDING_PX * 2 + singleMapWidth() + 96;
}

export function truthCellAtCanvasPoint(x: number, y: number): Cell | null {
  return cellAtMapPoint(x, y, 0);
}

export function drawSimulator(p: p5, sim: Simulation): void {
  p.background("#ffffff");

  const truthOrigin = mapOrigin(0);
  const serverOrigin = mapOrigin(1);
  const robotOrigin = mapOrigin(2);

  drawMapPanel(
    p,
    sim,
    truthOrigin.x,
    truthOrigin.y,
    "1. Real secret map",
    "Editable ground truth",
    "truth",
  );

  drawMapPanel(
    p,
    sim,
    serverOrigin.x,
    serverOrigin.y,
    "2. Shared server map",
    "Partial cohort knowledge from RFID queries",
    "server",
  );

  drawMapPanel(
    p,
    sim,
    robotOrigin.x,
    robotOrigin.y,
    "3. Robot navigation map",
    "Right-hand rule preview",
    "robot",
  );

  drawLegend(p);
}

function mapOrigin(index: number): { x: number; y: number } {
  return {
    x: PAGE_PADDING_PX + index * (singleMapWidth() + MAP_GAP_PX),
    y: PAGE_PADDING_PX + 38,
  };
}

function cellAtMapPoint(x: number, y: number, index: number): Cell | null {
  const origin = mapOrigin(index);
  const gridX = x - origin.x - HEADER_PX;
  const gridY = y - origin.y - HEADER_PX;
  const gridSizePx = GRID_SIZE * CELL_PX;

  if (gridX < 0 || gridY < 0 || gridX >= gridSizePx || gridY >= gridSizePx) {
    return null;
  }

  return {
    r: Math.floor(gridY / CELL_PX) + 1,
    c: Math.floor(gridX / CELL_PX) + 1,
  };
}

function cellScreenRect(originX: number, originY: number, cell: Cell) {
  return {
    x: originX + HEADER_PX + (cell.c - 1) * CELL_PX,
    y: originY + HEADER_PX + (cell.r - 1) * CELL_PX,
    w: CELL_PX,
    h: CELL_PX,
  };
}

function cellScreenCentre(originX: number, originY: number, cell: Cell) {
  const rect = cellScreenRect(originX, originY, cell);

  return {
    x: rect.x + rect.w / 2,
    y: rect.y + rect.h / 2,
  };
}

function drawMapPanel(
  p: p5,
  sim: Simulation,
  originX: number,
  originY: number,
  title: string,
  subtitle: string,
  mode: MapMode,
): void {
  const panelX = originX - 10;
  const panelY = originY - 34;
  const panelW = singleMapWidth() + 20;
  const panelH = singleMapWidth() + 54;

  p.push();

  p.noStroke();
  p.fill("#f8fafc");
  p.rect(panelX, panelY, panelW, panelH, 16);

  p.fill("#111827");
  p.textAlign(p.LEFT, p.TOP);
  p.textSize(15);
  p.textStyle(p.BOLD);
  p.text(title, originX, originY - 26);

  p.fill("#64748b");
  p.textSize(11);
  p.textStyle(p.NORMAL);
  p.text(subtitle, originX, originY - 9);

  drawGrid(p, originX, originY);
  drawCells(p, sim, originX, originY, mode);
  drawSpecialCell(p, originX, originY, START, START_LABEL, "#2563eb");
  drawSpecialCell(p, originX, originY, FINISH, FINISH_LABEL, "#7c3aed");

  if (mode === "robot") {
    drawPlannedPath(p, sim, originX, originY);
    drawObjective(p, sim, originX, originY);
  }

  p.pop();
}

function drawGrid(p: p5, originX: number, originY: number): void {
  for (let r = 0; r <= GRID_SIZE; r++) {
    for (let c = 0; c <= GRID_SIZE; c++) {
      const w = c === 0 ? HEADER_PX : CELL_PX;
      const h = r === 0 ? HEADER_PX : CELL_PX;

      const actualX = c === 0 ? originX : originX + HEADER_PX + (c - 1) * CELL_PX;
      const actualY = r === 0 ? originY : originY + HEADER_PX + (r - 1) * CELL_PX;

      p.stroke("#cbd5e1");
      p.strokeWeight(1);
      p.fill(r === 0 || c === 0 ? "#e2e8f0" : "#ffffff");
      p.rect(actualX, actualY, w, h);

      p.noStroke();
      p.fill("#334155");
      p.textAlign(p.CENTER, p.CENTER);
      p.textSize(12);
      p.textStyle(p.BOLD);

      if (r === 0 && c > 0) {
        p.text(String(c), actualX + w / 2, actualY + h / 2);
      }

      if (c === 0 && r > 0) {
        p.text(ROW_LABELS[r - 1], actualX + w / 2, actualY + h / 2);
      }
    }
  }
}

function drawCells(
  p: p5,
  sim: Simulation,
  originX: number,
  originY: number,
  mode: MapMode,
): void {
  for (const cell of allCells()) {
    const rect = cellScreenRect(originX, originY, cell);
    const t = sim.truth.get(keyOf(cell));
    const known = sim.knowledge.get(keyOf(cell));

    let fill = "#ffffff";
    let text = ".";
    let textFill = "#64748b";
    let stroke = "#cbd5e1";

    if (mode === "truth") {
      if (t?.obstacle) {
        fill = "#1f2937";
        text = "X";
        textFill = "#ffffff";
      } else if (t?.needsRescue) {
        fill = "#fecdd3";
        text = "R";
        textFill = "#881337";
      } else if (t?.planted) {
        fill = "#f6c177";
        text = "P";
        textFill = "#422006";
      } else if (t?.fertile) {
        fill = "#bbf7d0";
        text = "F";
        textFill = "#14532d";
      } else {
        fill = "#f1f5f9";
        text = "NF";
        textFill = "#475569";
      }
    }

    if (mode === "server" || mode === "robot") {
      if (known?.knownObstacle) {
        fill = "#1f2937";
        text = "X";
        textFill = "#ffffff";
      } else if (mode === "robot" && sameCell(cell, sim.robotPos)) {
        fill = "#fdba74";
        text = "BOT";
        textFill = "#431407";
      } else if (known?.needsRescue) {
        fill = "#fecdd3";
        text = "R";
        textFill = "#881337";
      } else if (known?.planted) {
        fill = "#f6c177";
        text = "P";
        textFill = "#422006";
      } else if (known?.status === "fertile") {
        fill = "#bbf7d0";
        text = "F";
        textFill = "#14532d";
      } else if (known?.status === "nonfertile") {
        fill = "#f1f5f9";
        text = "NF";
        textFill = "#475569";
      } else {
        fill = "#dbe2ea";
        text = "?";
        textFill = "#475569";
      }

      if (mode === "server" && known?.exploredByThisRobot) {
        stroke = "#2563eb";
      }
    }

    p.stroke(stroke);
    p.strokeWeight(mode === "server" && known?.exploredByThisRobot ? 2 : 1);
    p.fill(fill);
    p.rect(rect.x, rect.y, rect.w, rect.h, 4);

    p.noStroke();
    p.fill(textFill);
    p.textAlign(p.CENTER, p.CENTER);
    p.textStyle(text === "." ? p.NORMAL : p.BOLD);
    p.textSize(text.length >= 3 ? 8 : 11);
    p.text(text, rect.x + rect.w / 2, rect.y + rect.h / 2);

    if (
      mode === "robot" &&
      sim.plan.path.some((pathCell) => sameCell(pathCell, cell))
    ) {
      p.noFill();
      p.stroke("#38bdf8");
      p.strokeWeight(2);
      p.rect(rect.x + 3, rect.y + 3, rect.w - 6, rect.h - 6, 4);
    }
  }
}

function drawSpecialCell(
  p: p5,
  originX: number,
  originY: number,
  cell: Cell,
  label: string,
  colour: string,
): void {
  const rect = cellScreenRect(originX, originY, cell);

  p.noFill();
  p.stroke(colour);
  p.strokeWeight(3);
  p.rect(rect.x + 4, rect.y + 4, rect.w - 8, rect.h - 8, 5);

  p.noStroke();
  p.fill(colour);
  p.textAlign(p.LEFT, p.TOP);
  p.textStyle(p.BOLD);
  p.textSize(8);
  p.text(label, rect.x + 5, rect.y + 5);
}

function drawPlannedPath(
  p: p5,
  sim: Simulation,
  originX: number,
  originY: number,
): void {
  if (sim.plan.path.length < 2) return;

  p.noFill();
  p.stroke("#0284c7");
  p.strokeWeight(4);

  p.beginShape();

  for (const cell of sim.plan.path) {
    const centre = cellScreenCentre(originX, originY, cell);
    p.vertex(centre.x, centre.y);
  }

  p.endShape();
}

function drawObjective(
  p: p5,
  sim: Simulation,
  originX: number,
  originY: number,
): void {
  if (!sim.plan.objective) return;

  const rect = cellScreenRect(originX, originY, sim.plan.objective);

  let colour = "#0284c7";

  if (sim.plan.objectiveKind === "follow-right-wall") colour = "#2563eb";
  if (sim.plan.objectiveKind === "wait-blocked") colour = "#e11d48";
  if (sim.plan.objectiveKind === "complete") colour = "#7c3aed";

  p.noFill();
  p.stroke(colour);
  p.strokeWeight(4);
  p.rect(rect.x + 8, rect.y + 8, rect.w - 16, rect.h - 16, 6);

  if (sim.plan.objectiveKind === "wait-blocked" && sim.plan.blockedCell) {
    const blockedRect = cellScreenRect(originX, originY, sim.plan.blockedCell);

    p.stroke("#be123c");
    p.strokeWeight(4);
    p.rect(
      blockedRect.x + 5,
      blockedRect.y + 5,
      blockedRect.w - 10,
      blockedRect.h - 10,
      6,
    );
  }
}

function drawLegend(p: p5): void {
  const y = PAGE_PADDING_PX + 38 + singleMapWidth() + 24;
  const x = PAGE_PADDING_PX;

  p.push();

  p.noStroke();
  p.fill("#334155");
  p.textAlign(p.LEFT, p.TOP);
  p.textSize(12);
  p.textStyle(p.NORMAL);

  p.text(
    "Legend: F = fertile, NF = non-fertile, P = planted, X = obstacle, R = needs rescue, ? = unknown, BOT = robot. Blue border in server map = cell revealed by this robot.",
    x,
    y,
  );

  p.fill("#64748b");
  p.text(
    "Navigation rule: follow the right wall from C9. Wait 1.5s for an ahead blocker, revive only edge rescues, and keep traversing after 5 seeds until G9.",
    x,
    y + 22,
  );

  p.pop();
}
