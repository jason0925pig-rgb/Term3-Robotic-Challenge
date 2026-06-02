export function injectStyles(): void {
  if (document.getElementById("exploration-sim-styles")) return;

  const style = document.createElement("style");
  style.id = "exploration-sim-styles";

  style.textContent = `
    :root {
      color-scheme: light;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f5f7fb;
      color: #111827;
    }

    body {
      margin: 0;
      background: #f5f7fb;
    }

    #sim-root {
      max-width: 1480px;
      margin: 0 auto;
      padding: 24px;
      box-sizing: border-box;
    }

    .sim-header {
      margin-bottom: 16px;
    }

    .sim-title {
      margin: 0 0 6px 0;
      font-size: 26px;
      line-height: 1.15;
      font-weight: 750;
      letter-spacing: 0;
    }

    .sim-subtitle {
      margin: 0;
      color: #4b5563;
      font-size: 15px;
      max-width: 980px;
      line-height: 1.45;
    }

    .control-panel {
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      gap: 12px;
      padding: 14px;
      margin-bottom: 14px;
      background: #ffffff;
      border: 1px solid #d9e0ea;
      border-radius: 18px;
      box-shadow: 0 8px 24px rgba(15, 23, 42, 0.06);
    }

    .button-row {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }

    .sim-btn {
      appearance: none;
      border: 1px solid #cbd5e1;
      border-radius: 12px;
      padding: 10px 16px;
      min-height: 42px;
      font-size: 15px;
      font-weight: 650;
      line-height: 1;
      background: #ffffff;
      color: #111827;
      cursor: pointer;
      box-shadow: 0 1px 2px rgba(15, 23, 42, 0.05);
    }

    .sim-btn:hover {
      background: #f8fafc;
      border-color: #94a3b8;
    }

    .sim-btn.primary {
      background: #2563eb;
      border-color: #2563eb;
      color: white;
    }

    .sim-btn.primary:hover {
      background: #1d4ed8;
    }

    .sim-btn.danger {
      background: #dc2626;
      border-color: #dc2626;
      color: white;
    }

    .sim-btn.danger:hover {
      background: #b91c1c;
    }

    .toggle-row {
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      align-items: center;
    }

    .sim-checkbox {
      display: inline-flex !important;
      align-items: center;
      gap: 8px;
      padding: 9px 12px;
      min-height: 42px;
      border: 1px solid #d1d5db;
      border-radius: 12px;
      background: #f9fafb;
      font-size: 14px;
      font-weight: 600;
      color: #1f2937;
      cursor: pointer;
      box-sizing: border-box;
    }

    .sim-checkbox input {
      width: 18px;
      height: 18px;
      accent-color: #2563eb;
    }

    .truth-edit-row {
      display: inline-flex;
      align-items: center;
      gap: 10px;
      padding: 8px 10px 8px 12px;
      min-height: 42px;
      border: 1px solid #d1d5db;
      border-radius: 12px;
      background: #f9fafb;
      box-sizing: border-box;
    }

    .edit-label {
      color: #1f2937;
      font-size: 14px;
      font-weight: 700;
      white-space: nowrap;
    }

    .sim-select {
      min-height: 32px;
      border: 1px solid #cbd5e1;
      border-radius: 8px;
      padding: 4px 28px 4px 10px;
      background: #ffffff;
      color: #111827;
      font: inherit;
      font-size: 14px;
      font-weight: 650;
      cursor: pointer;
    }

    .status-grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(180px, 1fr));
      gap: 12px;
      margin-bottom: 14px;
    }

    .status-card {
      background: #ffffff;
      border: 1px solid #d9e0ea;
      border-radius: 16px;
      padding: 12px 14px;
      box-shadow: 0 6px 18px rgba(15, 23, 42, 0.05);
      min-height: 76px;
    }

    .status-label {
      display: block;
      color: #64748b;
      font-size: 12px;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.04em;
      margin-bottom: 4px;
    }

    .status-value {
      color: #111827;
      font-size: 15px;
      line-height: 1.35;
      font-weight: 650;
    }

    .status-wide {
      grid-column: span 2;
    }

    .canvas-wrap {
      overflow-x: auto;
      padding: 14px;
      background: #ffffff;
      border: 1px solid #d9e0ea;
      border-radius: 20px;
      box-shadow: 0 10px 30px rgba(15, 23, 42, 0.07);
    }

    canvas {
      display: block;
      border-radius: 14px;
    }

    @media (max-width: 950px) {
      #sim-root {
        padding: 14px;
      }

      .status-grid {
        grid-template-columns: 1fr;
      }

      .status-wide {
        grid-column: span 1;
      }

      .sim-title {
        font-size: 22px;
      }
    }
  `;

  document.head.appendChild(style);
}
