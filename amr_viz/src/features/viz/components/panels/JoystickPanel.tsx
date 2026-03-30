import { useMemo, useRef, useState } from "react";
import type { PointerEvent as ReactPointerEvent } from "react";

type JoystickPanelProps = {
  linearX: number;
  angularZ: number;
  onCommandChange: (linearX: number, angularZ: number) => void;
  onCommandStop: () => void;
};

const PAD_SIZE = 160;
const PAD_RADIUS = PAD_SIZE / 2;
const KNOB_RADIUS = 20;
const MAX_LINEAR_X = 0.18;
const MAX_ANGULAR_Z = 1.4;

function clamp(value: number, min: number, max: number) {
  return Math.min(max, Math.max(min, value));
}

export function JoystickPanel({
  linearX,
  angularZ,
  onCommandChange,
  onCommandStop,
}: JoystickPanelProps) {
  const padRef = useRef<HTMLDivElement | null>(null);
  const [dragVector, setDragVector] = useState({ x: 0, y: 0 });

  const knobStyle = useMemo(() => ({
    transform: `translate(${dragVector.x}px, ${dragVector.y}px)`,
  }), [dragVector.x, dragVector.y]);

  const updateFromPointer = (clientX: number, clientY: number) => {
    const pad = padRef.current;
    if (!pad) {
      return;
    }

    const rect = pad.getBoundingClientRect();
    const offsetX = clientX - (rect.left + (rect.width / 2));
    const offsetY = clientY - (rect.top + (rect.height / 2));
    const magnitude = Math.hypot(offsetX, offsetY);
    const limitedScale = magnitude > (PAD_RADIUS - KNOB_RADIUS)
      ? (PAD_RADIUS - KNOB_RADIUS) / magnitude
      : 1;
    const clampedX = offsetX * limitedScale;
    const clampedY = offsetY * limitedScale;
    const normalizedX = clamp(clampedX / (PAD_RADIUS - KNOB_RADIUS), -1, 1);
    const normalizedY = clamp(clampedY / (PAD_RADIUS - KNOB_RADIUS), -1, 1);

    setDragVector({ x: clampedX, y: clampedY });
    onCommandChange(-normalizedY * MAX_LINEAR_X, -normalizedX * MAX_ANGULAR_Z);
  };

  const handlePointerDown = (event: ReactPointerEvent<HTMLDivElement>) => {
    event.preventDefault();
    event.currentTarget.setPointerCapture(event.pointerId);
    updateFromPointer(event.clientX, event.clientY);
  };

  const handlePointerMove = (event: ReactPointerEvent<HTMLDivElement>) => {
    if (!event.currentTarget.hasPointerCapture(event.pointerId)) {
      return;
    }
    updateFromPointer(event.clientX, event.clientY);
  };

  const stop = () => {
    setDragVector({ x: 0, y: 0 });
    onCommandStop();
  };

  const handlePointerUp = (event: ReactPointerEvent<HTMLDivElement>) => {
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
    stop();
  };

  return (
    <section className="panel-card">
      <div className="panel-section-title">Joystick</div>
      <div className="joystick-panel">
        <div
          ref={padRef}
          className="joystick-pad"
          onPointerDown={handlePointerDown}
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerUp}
          onPointerCancel={handlePointerUp}
        >
          <div className="joystick-cross joystick-cross-horizontal" />
          <div className="joystick-cross joystick-cross-vertical" />
          <div className="joystick-knob" style={knobStyle} />
        </div>
        <div className="metric-list">
          <div className="metric-row">
            <span>Linear X</span>
            <strong>{linearX.toFixed(3)} m/s</strong>
          </div>
          <div className="metric-row">
            <span>Angular Z</span>
            <strong>{angularZ.toFixed(3)} rad/s</strong>
          </div>
        </div>
      </div>
    </section>
  );
}
