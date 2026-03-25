import type { LayerVisibility } from "../../types";

type LayersPanelProps = {
  layerVisibility: LayerVisibility;
  onToggleLayer: (layer: keyof LayerVisibility) => void;
};

const layerEntries: Array<[keyof LayerVisibility, string]> = [
  ["grid", "Grid"],
  ["map", "Map"],
  ["globalCostmap", "Global Costmap"],
  ["localCostmap", "Local Costmap"],
  ["footprint", "Exact Footprint"],
  ["blockedDebug", "Blocked Footprint"],
  ["collisionDebug", "Collision Debug"],
  ["robot", "Robot"],
  ["paths", "Plans"],
  ["scan", "LaserScan"],
  ["tf", "TF"],
];

export function LayersPanel({ layerVisibility, onToggleLayer }: LayersPanelProps) {
  return (
    <section className="panel-card">
      <div className="panel-section-title">Displays</div>
      <div className="layer-list">
        {layerEntries.map(([key, label]) => (
          <label className="layer-item" key={key}>
            <input
              type="checkbox"
              checked={layerVisibility[key]}
              onChange={() => onToggleLayer(key)}
            />
            <span>{label}</span>
          </label>
        ))}
      </div>
    </section>
  );
}
