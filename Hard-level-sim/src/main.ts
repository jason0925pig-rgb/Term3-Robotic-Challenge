import p5 from "p5";
import { AUTO_STEP_MS, FINISH_LABEL } from "./constants";
import {
  canvasHeight,
  canvasWidth,
  drawSimulator,
  truthCellAtCanvasPoint,
} from "./drawing";
import { Simulation } from "./simulation";
import { renderStatusHtml } from "./status";
import { injectStyles } from "./styles";
import type { TruthEditMode } from "./types";

const sim = new Simulation();

let autoCheckbox: any;
let cohortCheckbox: any;
let editModeSelect: any;
let statusDiv: any;
let autoRunning = false;
let lastStepAt = 0;

function updateStatusDiv(): void {
  statusDiv.html(renderStatusHtml(sim));
}

function setAutoRun(enabled: boolean): void {
  autoRunning = enabled;

  if (autoCheckbox?.checked) {
    autoCheckbox.checked(enabled);
  }
}

function cohortUpdatesEnabled(): boolean {
  return Boolean(cohortCheckbox?.checked?.());
}

function selectedTruthEditMode(): TruthEditMode {
  const value = editModeSelect?.value?.();

  switch (value) {
    case "fertile":
    case "nonfertile":
    case "planted":
    case "obstacle":
    case "needs-rescue":
      return value;
    default:
      return "fertile";
  }
}

function stepAndRefresh(): void {
  const outcome = sim.stepOnce(cohortUpdatesEnabled());
  updateStatusDiv();

  if (outcome.shouldStopAuto) {
    setAutoRun(false);
  }
}

const sketch = (p: p5) => {
  p.setup = () => {
    injectStyles();

    const root = p.createDiv();
    root.id("sim-root");

    const header = p.createDiv(`
      <h1 class="sim-title">Right-Hand Rule Navigation Simulator</h1>
      <p class="sim-subtitle">
        Partial-map simulator for the seed-planting robot. The robot enters at C9, follows the right wall, waits for ahead blockers, and keeps traversing after planting 5 seeds.
      </p>
    `);
    header.parent(root);
    header.class("sim-header");

    const controls = p.createDiv();
    controls.parent(root);
    controls.class("control-panel");

    const buttonRow = p.createDiv();
    buttonRow.parent(controls);
    buttonRow.class("button-row");

    const stepButton = p.createButton("Step");
    stepButton.parent(buttonRow);
    stepButton.class("sim-btn primary");
    stepButton.mousePressed(stepAndRefresh);

    const replanButton = p.createButton("Refresh navigation");
    replanButton.parent(buttonRow);
    replanButton.class("sim-btn");
    replanButton.mousePressed(() => {
      sim.replan("manual navigation refresh");
      sim.statusText = "Navigation preview refreshed.";
      updateStatusDiv();
    });

    const emergencyButton = p.createButton(`Emergency wall-follow to ${FINISH_LABEL}`);
    emergencyButton.parent(buttonRow);
    emergencyButton.class("sim-btn danger");
    emergencyButton.mousePressed(() => {
      sim.triggerEmergencyReturn();
      updateStatusDiv();
    });

    const resetButton = p.createButton("Generate new map");
    resetButton.parent(buttonRow);
    resetButton.class("sim-btn");
    resetButton.mousePressed(() => {
      sim.resetSimulation();
      setAutoRun(false);
      updateStatusDiv();
    });

    const repeatButton = p.createButton("Reset same map");
    repeatButton.parent(buttonRow);
    repeatButton.class("sim-btn");
    repeatButton.mousePressed(() => {
      sim.resetSimulation(sim.currentSeed);
      setAutoRun(false);
      updateStatusDiv();
    });

    const toggleRow = p.createDiv();
    toggleRow.parent(controls);
    toggleRow.class("toggle-row");

    autoCheckbox = p.createCheckbox("Auto run", false);
    autoCheckbox.parent(toggleRow);
    autoCheckbox.class("sim-checkbox");

    cohortCheckbox = p.createCheckbox("Add one server update each step", true);
    cohortCheckbox.parent(toggleRow);
    cohortCheckbox.class("sim-checkbox");

    const editRow = p.createDiv();
    editRow.parent(controls);
    editRow.class("truth-edit-row");

    const editLabel = p.createSpan("Truth map edit");
    editLabel.parent(editRow);
    editLabel.class("edit-label");

    editModeSelect = p.createSelect();
    editModeSelect.parent(editRow);
    editModeSelect.class("sim-select");
    editModeSelect.option("Fertile", "fertile");
    editModeSelect.option("Unfertile", "nonfertile");
    editModeSelect.option("Planted", "planted");
    editModeSelect.option("Obstacle", "obstacle");
    editModeSelect.option("Needs rescue", "needs-rescue");
    editModeSelect.selected("fertile");

    statusDiv = p.createDiv("");
    statusDiv.parent(root);
    statusDiv.class("status-grid");

    const canvasWrap = p.createDiv();
    canvasWrap.parent(root);
    canvasWrap.class("canvas-wrap");

    const canvas = p.createCanvas(canvasWidth(), canvasHeight());
    canvas.parent(canvasWrap);
    canvas.mousePressed(() => {
      const cell = truthCellAtCanvasPoint(p.mouseX, p.mouseY);

      if (!cell) return;

      sim.applyTruthEdit(cell, selectedTruthEditMode());
      updateStatusDiv();
    });

    sim.resetSimulation(2026);
    updateStatusDiv();
  };

  p.draw = () => {
    if (autoCheckbox?.checked?.()) {
      const now = p.millis();

      if (!autoRunning) {
        autoRunning = true;
        lastStepAt = now;
      }

      if (now - lastStepAt >= AUTO_STEP_MS) {
        stepAndRefresh();
        lastStepAt = now;
      }
    } else {
      autoRunning = false;
    }

    drawSimulator(p, sim);
  };
};

new p5(sketch);
