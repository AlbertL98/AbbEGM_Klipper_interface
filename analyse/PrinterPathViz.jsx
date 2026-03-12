import { useState, useRef, useEffect, useCallback } from "react";
import * as THREE from "three";

/* ───────────────── helpers ───────────────── */
function parseCSV(text) {
  const lines = text.replace(/\r/g, "").trim().split("\n");
  const rows = [];
  for (let i = 1; i < lines.length; i++) {
    const vals = lines[i].split(",");
    if (vals.length < 14) continue;
    rows.push({
      t: +vals[0], sx: +vals[1], sy: +vals[2], sz: +vals[3],
      ix: +vals[4], iy: +vals[5], iz: +vals[6], extPos: +vals[7],
      sollVel: +vals[8], istVelTCP: +vals[9], istVelExt: +vals[10],
      errTCP: +vals[11], errSpeed: +vals[12], ratio: +vals[13],
    });
  }
  return rows;
}

function downsample(rows, maxPts = 4000) {
  if (rows.length <= maxPts) return rows;
  const step = rows.length / maxPts;
  const out = [];
  for (let i = 0; i < maxPts; i++) out.push(rows[Math.floor(i * step)]);
  return out;
}

function valueToColor(val, min, max) {
  const t = max === min ? 0.5 : Math.max(0, Math.min(1, (val - min) / (max - min)));
  const hue = (1 - t) * 0.66;
  const c = new THREE.Color();
  c.setHSL(hue, 0.95, 0.5);
  return c;
}

const COLOR_MODES = [
  { key: "istVelTCP", label: "Speed TCP", unit: "mm/s" },
  { key: "istVelExt", label: "Extruder Speed", unit: "mm/s" },
  { key: "errTCP", label: "Error Position", unit: "mm" },
  { key: "errSpeed", label: "Error Speed", unit: "mm/s" },
  { key: "ratio", label: "Ratio Factor", unit: "" },
];

function buildColoredLine(points, colors) {
  const geo = new THREE.BufferGeometry();
  const positions = new Float32Array(points.length * 3);
  const cols = new Float32Array(points.length * 3);
  for (let i = 0; i < points.length; i++) {
    positions[i * 3] = points[i].x;
    positions[i * 3 + 1] = points[i].y;
    positions[i * 3 + 2] = points[i].z;
    cols[i * 3] = colors[i].r;
    cols[i * 3 + 1] = colors[i].g;
    cols[i * 3 + 2] = colors[i].b;
  }
  geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geo.setAttribute("color", new THREE.BufferAttribute(cols, 3));
  return new THREE.Line(geo, new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 2 }));
}

/* ────────── Three.js scene manager ────────── */
class SceneManager {
  constructor(canvas) {
    this.canvas = canvas;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(w, h);
    this.renderer.setClearColor(0x0a0e17, 1);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(50, w / h, 0.1, 5000);
    this.camera.position.set(300, 250, 300);
    this.camera.lookAt(150, 30, 150);
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const dl = new THREE.DirectionalLight(0xffffff, 0.8);
    dl.position.set(200, 300, 200);
    this.scene.add(dl);
    this.addGrid();
    this.isDragging = false;
    this.lastMouse = { x: 0, y: 0 };
    this.spherical = new THREE.Spherical().setFromVector3(
      this.camera.position.clone().sub(new THREE.Vector3(125, 25, 125))
    );
    this.target = new THREE.Vector3(125, 25, 125);
    this.setupControls();
    this.datasets = {};
    this.animate();
  }
  addGrid() {
    const g = new THREE.GridHelper(300, 30, 0x1a2744, 0x111b2e);
    g.position.set(125, 0, 125);
    this.scene.add(g);
    const axMat = (c) => new THREE.LineBasicMaterial({ color: c, linewidth: 2 });
    const ml = (a, b, c) => { const g2 = new THREE.BufferGeometry().setFromPoints([a, b]); return new THREE.Line(g2, axMat(c)); };
    this.scene.add(ml(new THREE.Vector3(0,0,0), new THREE.Vector3(270,0,0), 0xff4444));
    this.scene.add(ml(new THREE.Vector3(0,0,0), new THREE.Vector3(0,60,0), 0x44ff44));
    this.scene.add(ml(new THREE.Vector3(0,0,0), new THREE.Vector3(0,0,270), 0x4488ff));
  }
  setupControls() {
    const c = this.canvas;
    c.addEventListener("pointerdown", (e) => { this.isDragging = true; this.lastMouse = { x: e.clientX, y: e.clientY }; });
    window.addEventListener("pointerup", () => { this.isDragging = false; });
    window.addEventListener("pointermove", (e) => {
      if (!this.isDragging) return;
      const dx = e.clientX - this.lastMouse.x, dy = e.clientY - this.lastMouse.y;
      this.lastMouse = { x: e.clientX, y: e.clientY };
      if (e.buttons === 1 && !e.shiftKey) {
        this.spherical.theta -= dx * 0.005;
        this.spherical.phi -= dy * 0.005;
        this.spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, this.spherical.phi));
      } else if (e.buttons === 1 && e.shiftKey) {
        const right = new THREE.Vector3(), up = new THREE.Vector3();
        right.setFromMatrixColumn(this.camera.matrix, 0);
        up.setFromMatrixColumn(this.camera.matrix, 1);
        this.target.addScaledVector(right, -dx * 0.3);
        this.target.addScaledVector(up, dy * 0.3);
      }
      this.updateCamera();
    });
    c.addEventListener("wheel", (e) => {
      e.preventDefault();
      this.spherical.radius *= 1 + e.deltaY * 0.001;
      this.spherical.radius = Math.max(50, Math.min(1500, this.spherical.radius));
      this.updateCamera();
    }, { passive: false });
    this.updateCamera();
  }
  updateCamera() {
    const pos = new THREE.Vector3().setFromSpherical(this.spherical).add(this.target);
    this.camera.position.copy(pos);
    this.camera.lookAt(this.target);
  }
  resize() {
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }
  animate = () => { requestAnimationFrame(this.animate); this.renderer.render(this.scene, this.camera); };
  addDataset(id, data, colorMode, showSoll, showIst) {
    this.removeDataset(id);
    const ds = downsample(data);
    const entry = { sollLine: null, istLine: null };
    if (showSoll && ds.length > 1) {
      const pts = ds.map(r => new THREE.Vector3(r.sx, r.sz, r.sy));
      const minV = Math.min(...ds.map(r => r.sollVel));
      const maxV = Math.max(...ds.map(r => r.sollVel));
      const cols = ds.map(r => valueToColor(r.sollVel, minV, maxV));
      entry.sollLine = buildColoredLine(pts, cols);
      entry.sollLine.renderOrder = 1;
      this.scene.add(entry.sollLine);
    }
    if (showIst && ds.length > 1) {
      const pts = ds.map(r => new THREE.Vector3(r.ix, r.iz, r.iy));
      const vals = ds.map(r => r[colorMode]);
      const minV = Math.min(...vals), maxV = Math.max(...vals);
      const cols = ds.map(r => valueToColor(r[colorMode], minV, maxV));
      entry.istLine = buildColoredLine(pts, cols);
      this.scene.add(entry.istLine);
    }
    this.datasets[id] = entry;
  }
  removeDataset(id) {
    const entry = this.datasets[id];
    if (!entry) return;
    if (entry.sollLine) { this.scene.remove(entry.sollLine); entry.sollLine.geometry.dispose(); }
    if (entry.istLine) { this.scene.remove(entry.istLine); entry.istLine.geometry.dispose(); }
    delete this.datasets[id];
  }
  dispose() { this.renderer.dispose(); }
}

/* ────────── Dual-handle range slider ────────── */
function RangeSlider({ min, max, valueStart, valueEnd, onChange }) {
  const trackRef = useRef(null);
  const dragging = useRef(null);
  const pctStart = max === min ? 0 : ((valueStart - min) / (max - min)) * 100;
  const pctEnd = max === min ? 100 : ((valueEnd - min) / (max - min)) * 100;

  const valFromX = useCallback((clientX) => {
    const rect = trackRef.current.getBoundingClientRect();
    const pct = Math.max(0, Math.min(1, (clientX - rect.left) / rect.width));
    return Math.round(min + pct * (max - min));
  }, [min, max]);

  const onPointerDown = useCallback((handle) => (e) => {
    e.preventDefault();
    e.stopPropagation();
    dragging.current = handle;
    const onMove = (ev) => {
      if (!dragging.current) return;
      const v = valFromX(ev.clientX);
      if (dragging.current === "start") onChange(Math.min(v, valueEnd - 1), valueEnd);
      else onChange(valueStart, Math.max(v, valueStart + 1));
    };
    const onUp = () => { dragging.current = null; window.removeEventListener("pointermove", onMove); window.removeEventListener("pointerup", onUp); };
    window.addEventListener("pointermove", onMove);
    window.addEventListener("pointerup", onUp);
  }, [valFromX, valueStart, valueEnd, onChange]);

  return (
    <div className="relative w-full" style={{ height: 28, touchAction: "none" }}>
      <div ref={trackRef} className="absolute rounded-full" style={{ top: 11, left: 0, right: 0, height: 6, background: "rgba(255,255,255,0.06)" }} />
      <div className="absolute rounded-full" style={{ top: 11, height: 6, left: `${pctStart}%`, width: `${pctEnd - pctStart}%`, background: "linear-gradient(90deg, #38bdf8, #818cf8)", opacity: 0.6 }} />
      <div onPointerDown={onPointerDown("start")} className="absolute" style={{ left: `${pctStart}%`, top: 5, width: 18, height: 18, marginLeft: -9, borderRadius: "50%", background: "#38bdf8", border: "2px solid #0a0e17", cursor: "pointer", zIndex: 2, boxShadow: "0 0 6px rgba(56,189,248,0.5)" }} />
      <div onPointerDown={onPointerDown("end")} className="absolute" style={{ left: `${pctEnd}%`, top: 5, width: 18, height: 18, marginLeft: -9, borderRadius: "50%", background: "#818cf8", border: "2px solid #0a0e17", cursor: "pointer", zIndex: 2, boxShadow: "0 0 6px rgba(129,140,248,0.5)" }} />
    </div>
  );
}

/* ────────── Color legend ────────── */
function ColorLegend({ min, max, label }) {
  return (
    <div className="flex items-center gap-2 text-xs" style={{ fontFamily: "'JetBrains Mono', monospace" }}>
      <span className="opacity-60">{min.toFixed(1)}</span>
      <div className="h-3 rounded-sm flex-1" style={{ background: "linear-gradient(to right, #0055ff, #00cccc, #00dd44, #dddd00, #ff2200)", minWidth: 100 }} />
      <span className="opacity-60">{max.toFixed(1)}</span>
      <span className="opacity-40 ml-1">{label}</span>
    </div>
  );
}

const FILE_COLORS = ["#38bdf8", "#f472b6", "#a78bfa", "#34d399", "#fb923c", "#facc15"];

/* ────────── Main App ────────── */
export default function PrinterPathViz() {
  const canvasRef = useRef(null);
  const sceneRef = useRef(null);
  const [files, setFiles] = useState([]);
  const [colorMode, setColorMode] = useState("istVelTCP");
  const [showSoll, setShowSoll] = useState(true);
  const [showIst, setShowIst] = useState(true);
  const [expandedFile, setExpandedFile] = useState(null);
  const fileInputRef = useRef(null);
  const idCounter = useRef(0);

  useEffect(() => {
    if (!canvasRef.current) return;
    sceneRef.current = new SceneManager(canvasRef.current);
    const onResize = () => sceneRef.current?.resize();
    window.addEventListener("resize", onResize);
    return () => { window.removeEventListener("resize", onResize); sceneRef.current?.dispose(); };
  }, []);

  useEffect(() => {
    if (!sceneRef.current) return;
    files.forEach((f) => {
      if (f.visible) {
        const trimmed = f.data.slice(f.rangeStart, f.rangeEnd + 1);
        sceneRef.current.addDataset(f.id, trimmed, colorMode, showSoll, showIst);
      } else {
        sceneRef.current.removeDataset(f.id);
      }
    });
  }, [files, colorMode, showSoll, showIst]);

  const handleUpload = useCallback((e) => {
    Array.from(e.target.files).forEach((file) => {
      const reader = new FileReader();
      reader.onload = (ev) => {
        const data = parseCSV(ev.target.result);
        if (data.length === 0) return;
        const id = `ds_${idCounter.current++}`;
        setFiles((prev) => [...prev, { id, name: file.name, data, visible: true, rangeStart: 0, rangeEnd: data.length - 1 }]);
        setExpandedFile(id);
      };
      reader.readAsText(file);
    });
    e.target.value = "";
  }, []);

  const toggleFile = useCallback((id) => {
    setFiles((prev) => prev.map((f) => (f.id === id ? { ...f, visible: !f.visible } : f)));
  }, []);

  const removeFile = useCallback((id) => {
    sceneRef.current?.removeDataset(id);
    setFiles((prev) => prev.filter((f) => f.id !== id));
    setExpandedFile((prev) => (prev === id ? null : prev));
  }, []);

  const setRange = useCallback((id, start, end) => {
    setFiles((prev) => prev.map((f) => (f.id === id ? { ...f, rangeStart: start, rangeEnd: end } : f)));
  }, []);

  const allData = files.filter(f => f.visible).flatMap(f => downsample(f.data.slice(f.rangeStart, f.rangeEnd + 1)));
  const modeInfo = COLOR_MODES.find(m => m.key === colorMode);
  let istMin = 0, istMax = 1, sollMin = 0, sollMax = 1;
  if (allData.length > 0) {
    const iv = allData.map(r => r[colorMode]);
    istMin = Math.min(...iv); istMax = Math.max(...iv);
    const sv = allData.map(r => r.sollVel);
    sollMin = Math.min(...sv); sollMax = Math.max(...sv);
  }

  return (
    <div className="flex flex-col w-full h-screen overflow-hidden" style={{ background: "linear-gradient(135deg, #060a12 0%, #0c1220 50%, #0a0e17 100%)", color: "#c8d6e5", fontFamily: "'Segoe UI', system-ui, sans-serif" }}>
      {/* TOP BAR */}
      <div className="flex items-center justify-between px-5 py-3 shrink-0" style={{ background: "rgba(12,18,32,0.85)", borderBottom: "1px solid rgba(56,189,248,0.12)", backdropFilter: "blur(12px)" }}>
        <div className="flex items-center gap-3">
          <div className="w-8 h-8 rounded flex items-center justify-center text-sm font-bold" style={{ background: "linear-gradient(135deg, #38bdf8, #818cf8)", color: "#060a12" }}>3D</div>
          <div>
            <div className="text-sm font-semibold tracking-wide" style={{ color: "#e2e8f0" }}>Druckpfad Visualisierung</div>
            <div className="text-xs opacity-40">Soll / Ist Vergleich</div>
          </div>
        </div>
        <button onClick={() => fileInputRef.current?.click()} className="px-4 py-2 rounded text-xs font-semibold tracking-wide" style={{ background: "linear-gradient(135deg, #38bdf8, #818cf8)", color: "#060a12", border: "none", cursor: "pointer" }}>+ CSV Hochladen</button>
        <input ref={fileInputRef} type="file" accept=".csv" multiple onChange={handleUpload} className="hidden" />
      </div>

      {/* MAIN */}
      <div className="flex flex-1 overflow-hidden">
        {/* SIDEBAR */}
        <div className="shrink-0 flex flex-col gap-4 p-4 overflow-y-auto" style={{ width: 280, background: "rgba(8,12,22,0.65)", borderRight: "1px solid rgba(56,189,248,0.08)" }}>
          {/* Path toggles */}
          <div>
            <div className="text-xs font-semibold uppercase tracking-widest opacity-40 mb-2">Pfade</div>
            <label className="flex items-center gap-2 cursor-pointer mb-1">
              <input type="checkbox" checked={showSoll} onChange={() => setShowSoll(!showSoll)} className="accent-sky-400" />
              <span className="text-xs">Soll-Pfad anzeigen</span>
            </label>
            <label className="flex items-center gap-2 cursor-pointer">
              <input type="checkbox" checked={showIst} onChange={() => setShowIst(!showIst)} className="accent-sky-400" />
              <span className="text-xs">Ist-Pfad anzeigen</span>
            </label>
          </div>

          {/* Color mode */}
          <div>
            <div className="text-xs font-semibold uppercase tracking-widest opacity-40 mb-2">Ist-Pfad Farbe</div>
            <div className="flex flex-col gap-1">
              {COLOR_MODES.map(m => (
                <button key={m.key} onClick={() => setColorMode(m.key)} className="text-left px-3 py-2 rounded text-xs" style={{ background: colorMode === m.key ? "linear-gradient(90deg, rgba(56,189,248,0.2), rgba(129,140,248,0.1))" : "transparent", border: colorMode === m.key ? "1px solid rgba(56,189,248,0.35)" : "1px solid transparent", color: colorMode === m.key ? "#38bdf8" : "#8899aa", cursor: "pointer" }}>
                  {m.label}
                </button>
              ))}
            </div>
          </div>

          {/* Legend */}
          <div>
            <div className="text-xs font-semibold uppercase tracking-widest opacity-40 mb-2">Legende</div>
            {showIst && allData.length > 0 && (
              <div className="mb-2">
                <div className="text-xs opacity-50 mb-1">Ist — {modeInfo?.label}</div>
                <ColorLegend min={istMin} max={istMax} label={modeInfo?.unit || ""} />
              </div>
            )}
            {showSoll && allData.length > 0 && (
              <div>
                <div className="text-xs opacity-50 mb-1">Soll — Geschwindigkeit</div>
                <ColorLegend min={sollMin} max={sollMax} label="mm/s" />
              </div>
            )}
          </div>

          {/* Files + Range Trimmer */}
          <div>
            <div className="text-xs font-semibold uppercase tracking-widest opacity-40 mb-2">Dateien ({files.length})</div>
            {files.length === 0 && <div className="text-xs opacity-30 italic">Keine CSV geladen</div>}
            {files.map((f, idx) => {
              const isExpanded = expandedFile === f.id;
              const totalRows = f.data.length;
              const shownRows = f.rangeEnd - f.rangeStart + 1;
              const startTime = f.data[f.rangeStart]?.t?.toFixed(2) ?? "–";
              const endTime = f.data[f.rangeEnd]?.t?.toFixed(2) ?? "–";
              return (
                <div key={f.id} className="mb-2 rounded" style={{ background: "rgba(255,255,255,0.03)", border: isExpanded ? "1px solid rgba(56,189,248,0.15)" : "1px solid transparent", transition: "border 0.2s" }}>
                  {/* file header */}
                  <div className="flex items-center gap-2 px-2 py-2">
                    <div className="w-2 h-2 rounded-full shrink-0" style={{ background: FILE_COLORS[idx % FILE_COLORS.length] }} />
                    <span className="text-xs flex-1 truncate cursor-pointer" style={{ opacity: f.visible ? 1 : 0.35 }} onClick={() => setExpandedFile(isExpanded ? null : f.id)} title="Bereich einstellen">{f.name}</span>
                    <button onClick={() => setExpandedFile(isExpanded ? null : f.id)} className="text-xs px-1 opacity-40 hover:opacity-100" style={{ background: "none", border: "none", color: "#38bdf8", cursor: "pointer", transform: isExpanded ? "rotate(180deg)" : "rotate(0deg)", transition: "transform 0.2s" }} title="Bereich">▾</button>
                    <button onClick={() => toggleFile(f.id)} className="text-xs px-1 opacity-50 hover:opacity-100" style={{ background: "none", border: "none", color: "#c8d6e5", cursor: "pointer" }} title={f.visible ? "Ausblenden" : "Einblenden"}>{f.visible ? "👁" : "👁‍🗨"}</button>
                    <button onClick={() => removeFile(f.id)} className="text-xs px-1 opacity-40 hover:opacity-100" style={{ background: "none", border: "none", color: "#f87171", cursor: "pointer" }} title="Entfernen">✕</button>
                  </div>

                  {/* Expandable range trimmer */}
                  {isExpanded && (
                    <div className="px-3 pb-3" style={{ borderTop: "1px solid rgba(255,255,255,0.04)" }}>
                      <div className="text-xs opacity-40 mt-2 mb-1">Datenbereich einschränken</div>

                      <RangeSlider min={0} max={totalRows - 1} valueStart={f.rangeStart} valueEnd={f.rangeEnd} onChange={(s, e) => setRange(f.id, s, e)} />

                      {/* numeric inputs */}
                      <div className="flex gap-2 mt-2">
                        <div className="flex-1">
                          <div className="text-xs opacity-30 mb-1">Start-Zeile</div>
                          <input type="number" min={0} max={f.rangeEnd - 1} value={f.rangeStart}
                            onChange={(e) => { const v = Math.max(0, Math.min(f.rangeEnd - 1, +e.target.value || 0)); setRange(f.id, v, f.rangeEnd); }}
                            className="w-full text-xs px-2 py-1 rounded" style={{ background: "rgba(255,255,255,0.06)", border: "1px solid rgba(255,255,255,0.1)", color: "#38bdf8", outline: "none" }} />
                        </div>
                        <div className="flex-1">
                          <div className="text-xs opacity-30 mb-1">End-Zeile</div>
                          <input type="number" min={f.rangeStart + 1} max={totalRows - 1} value={f.rangeEnd}
                            onChange={(e) => { const v = Math.max(f.rangeStart + 1, Math.min(totalRows - 1, +e.target.value || 0)); setRange(f.id, f.rangeStart, v); }}
                            className="w-full text-xs px-2 py-1 rounded" style={{ background: "rgba(255,255,255,0.06)", border: "1px solid rgba(255,255,255,0.1)", color: "#818cf8", outline: "none" }} />
                        </div>
                      </div>

                      {/* info badges */}
                      <div className="flex flex-wrap gap-2 mt-2">
                        <span className="text-xs px-2 py-0.5 rounded" style={{ background: "rgba(56,189,248,0.1)", color: "#38bdf8" }}>{shownRows.toLocaleString()} / {totalRows.toLocaleString()} Zeilen</span>
                        <span className="text-xs px-2 py-0.5 rounded" style={{ background: "rgba(255,255,255,0.05)", color: "#8899aa" }}>t: {startTime}s – {endTime}s</span>
                      </div>

                      {/* quick preset buttons */}
                      <div className="flex flex-wrap gap-1 mt-2">
                        <button onClick={() => setRange(f.id, 0, totalRows - 1)} className="text-xs px-2 py-1 rounded" style={{ background: "rgba(255,255,255,0.05)", border: "none", color: "#8899aa", cursor: "pointer" }}>Alles</button>
                        <button onClick={() => setRange(f.id, Math.floor(totalRows * 0.1), f.rangeEnd)} className="text-xs px-2 py-1 rounded" style={{ background: "rgba(255,255,255,0.05)", border: "none", color: "#8899aa", cursor: "pointer" }}>Ab 10%</button>
                        <button onClick={() => setRange(f.id, Math.floor(totalRows * 0.25), f.rangeEnd)} className="text-xs px-2 py-1 rounded" style={{ background: "rgba(255,255,255,0.05)", border: "none", color: "#8899aa", cursor: "pointer" }}>Ab 25%</button>
                        <button onClick={() => setRange(f.id, f.rangeStart, Math.floor(totalRows * 0.75))} className="text-xs px-2 py-1 rounded" style={{ background: "rgba(255,255,255,0.05)", border: "none", color: "#8899aa", cursor: "pointer" }}>Bis 75%</button>
                        <button onClick={() => setRange(f.id, f.rangeStart, Math.floor(totalRows * 0.5))} className="text-xs px-2 py-1 rounded" style={{ background: "rgba(255,255,255,0.05)", border: "none", color: "#8899aa", cursor: "pointer" }}>Bis 50%</button>
                      </div>
                    </div>
                  )}
                </div>
              );
            })}
          </div>

          {/* Controls help */}
          <div className="mt-auto pt-4" style={{ borderTop: "1px solid rgba(255,255,255,0.05)" }}>
            <div className="text-xs opacity-30 leading-relaxed">
              <strong className="opacity-60">Steuerung:</strong><br />
              Maus ziehen → Rotieren<br />
              Shift + Maus → Verschieben<br />
              Scrollrad → Zoom
            </div>
          </div>
        </div>

        {/* 3D CANVAS */}
        <div className="flex-1 relative">
          <canvas ref={canvasRef} className="w-full h-full block" style={{ touchAction: "none" }} />
          {files.length === 0 && (
            <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
              <div className="text-center px-8 py-6 rounded-lg" style={{ background: "rgba(12,18,32,0.7)", border: "1px dashed rgba(56,189,248,0.2)" }}>
                <div className="text-4xl mb-3 opacity-20">⬡</div>
                <div className="text-sm opacity-40">CSV-Datei hochladen um die<br />3D-Druckpfade zu visualisieren</div>
              </div>
            </div>
          )}
          <div className="absolute bottom-3 right-3 text-xs opacity-30 text-right" style={{ fontFamily: "'JetBrains Mono', monospace" }}>
            <span style={{ color: "#ff4444" }}>X</span>{" "}
            <span style={{ color: "#44ff44" }}>Z</span>{" "}
            <span style={{ color: "#4488ff" }}>Y</span>
          </div>
        </div>
      </div>
    </div>
  );
}
