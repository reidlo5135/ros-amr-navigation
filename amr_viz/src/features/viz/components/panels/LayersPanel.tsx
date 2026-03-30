import type { LayerVisibility } from "../../types";

type LayersPanelProps = {
  viewMode: "nav" | "mapping";
  layerVisibility: LayerVisibility;
  onToggleLayer: (layer: keyof LayerVisibility) => void;
};

type LayerEntry = {
  key: keyof LayerVisibility;
  label: string;
  icon: string;
};

const layerEntries: LayerEntry[] = [
  { key: "grid", label: "Grid", icon: "grid" },
  { key: "map", label: "Map", icon: "map" },
  { key: "tempMap", label: "Temp SLAM Map", icon: "temp-map" },
  { key: "globalCostmap", label: "Global Costmap", icon: "costmap-global" },
  { key: "localCostmap", label: "Local Costmap", icon: "costmap-local" },
  { key: "footprint", label: "Exact Footprint", icon: "footprint" },
  { key: "robot", label: "Robot", icon: "robot" },
  { key: "globalPlan", label: "Global Plan", icon: "path-global" },
  { key: "localPlan", label: "Local Plan", icon: "path-local" },
  { key: "scan", label: "LaserScan", icon: "scan" },
  { key: "tf", label: "TF", icon: "tf" },
  { key: "keyframes", label: "Keyframes", icon: "keyframes" },
  { key: "graphEdges", label: "Graph Edges", icon: "graph-edges" },
  { key: "loopMarkers", label: "Loop Markers", icon: "loop-markers" },
];

function LayerIcon({ kind }: { kind: string }) {
  return (
    <span className={`layer-icon layer-icon-${kind}`} aria-hidden="true">
      {kind === "grid" && (
        <svg viewBox="0 0 16 16">
          <path d="M1 5.5h14M1 10.5h14M5.5 1v14M10.5 1v14" />
        </svg>
      )}
      {kind === "map" && (
        <svg viewBox="0 0 16 16">
          <path d="M2 3.5 5.5 2l5 1.5L14 2.5v10L10.5 14l-5-1.5L2 13.5z" />
        </svg>
      )}
      {kind === "temp-map" && (
        <svg viewBox="0 0 16 16">
          <rect x="2" y="2" width="12" height="12" rx="1.2" />
          <path d="M2 8h12M8 2v12" />
          <circle cx="11.5" cy="4.5" r="1.1" />
        </svg>
      )}
      {kind === "costmap-global" && (
        <svg viewBox="0 0 16 16">
          <rect x="2" y="2" width="12" height="12" rx="1.2" />
          <path d="M2 8h12M8 2v12" />
        </svg>
      )}
      {kind === "costmap-local" && (
        <svg viewBox="0 0 16 16">
          <rect x="3" y="3" width="10" height="10" rx="1.2" />
          <circle cx="8" cy="8" r="2.4" />
        </svg>
      )}
      {kind === "footprint" && (
        <svg viewBox="0 0 16 16">
          <path d="M3 5.5 7.2 3.5 13 5.3 11.2 12.5 4.2 11.2z" />
        </svg>
      )}
      {kind === "robot" && (
        <svg viewBox="0 0 16 16">
          <rect x="4" y="4.5" width="8" height="7" rx="1" />
          <path d="M6 13v2M10 13v2M3 7H1M15 7h-2M6 2h4" />
        </svg>
      )}
      {(kind === "path-global" || kind === "path-local") && (
        <svg viewBox="0 0 16 16">
          <circle cx="3" cy="12" r="1.2" />
          <circle cx="13" cy="4" r="1.2" />
          <path d="M4.5 11 7.2 8.2 9 9.1 11.5 6.4" />
        </svg>
      )}
      {kind === "scan" && (
        <svg viewBox="0 0 16 16">
          <path d="M3 12a6 6 0 0 1 10-4.2" />
          <path d="M3 12a6 6 0 0 0 4.6 1.8" />
          <circle cx="8" cy="8" r="1.2" />
        </svg>
      )}
      {kind === "tf" && (
        <svg viewBox="0 0 16 16">
          <path d="M8 8V2M8 8H14M8 8 3 13" />
        </svg>
      )}
      {kind === "keyframes" && (
        <svg viewBox="0 0 16 16">
          <circle cx="3.5" cy="12.5" r="1.2" />
          <circle cx="8" cy="8" r="1.2" />
          <circle cx="12.5" cy="3.5" r="1.2" />
          <path d="M4.5 11.5 7 9 11.2 4.8" />
        </svg>
      )}
      {kind === "graph-edges" && (
        <svg viewBox="0 0 16 16">
          <path d="M2.5 12.5 6.2 8.2 9.1 9.3 13.5 3.5" />
        </svg>
      )}
      {kind === "loop-markers" && (
        <svg viewBox="0 0 16 16">
          <path d="M4 9a4 4 0 1 0 2-3.46" />
          <path d="M6 3.5H3.2V6.3" />
        </svg>
      )}
    </span>
  );
}

export function LayersPanel({ viewMode, layerVisibility, onToggleLayer }: LayersPanelProps) {
  const visibleEntries = layerEntries.filter(({ key }) => {
    if (viewMode === "nav") {
      return key !== "tempMap" && key !== "keyframes" && key !== "graphEdges" && key !== "loopMarkers";
    }
    return key !== "globalCostmap" && key !== "localCostmap" && key !== "globalPlan" && key !== "localPlan";
  });

  return (
    <section className="panel-card panel-card-fill">
      <div className="panel-section-title">Displays</div>
      <div className="layer-list">
        {visibleEntries.map(({ key, label, icon }) => (
          <label className="layer-item" key={key}>
            <input
              type="checkbox"
              checked={layerVisibility[key]}
              onChange={() => onToggleLayer(key)}
            />
            <LayerIcon kind={icon} />
            <span>{label}</span>
          </label>
        ))}
      </div>
    </section>
  );
}
