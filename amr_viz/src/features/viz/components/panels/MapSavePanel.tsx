type MapSavePanelProps = {
  basename: string;
  onBasenameChange: (value: string) => void;
  onSave: () => void;
};

export function MapSavePanel({
  basename,
  onBasenameChange,
  onSave,
}: MapSavePanelProps) {
  return (
    <section className="panel-card">
      <div className="panel-section-title">Map Saving</div>
      <div className="field-grid">
        <label className="field-label">
          <span>Base Filename</span>
          <input
            type="text"
            value={basename}
            onChange={(event) => onBasenameChange(event.target.value)}
            placeholder="amr_mapping_map"
          />
        </label>
        <button type="button" onClick={onSave}>
          Save `.pgm` + `.yaml`
        </button>
      </div>
    </section>
  );
}
