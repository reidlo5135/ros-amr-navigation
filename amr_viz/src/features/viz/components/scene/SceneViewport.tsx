import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

import type {
  BridgeState,
  LaserScanMessage,
  OccupancyGridMessage,
  Pose,
  TfMessage,
} from "../../../../lib/protocol";
import type { GoalLifecycleState } from "../../types";

type SceneViewportProps = {
  state: BridgeState;
  layerVisibility: {
    grid: boolean;
    map: boolean;
    globalCostmap: boolean;
    localCostmap: boolean;
    footprint: boolean;
    robot: boolean;
    globalPlan: boolean;
    localPlan: boolean;
    scan: boolean;
    tf: boolean;
  };
  goalMarker?: {
    x: number;
    y: number;
    yaw: number;
    kind: "goal" | "initial_pose";
  } | null;
  goalLifecycle?: GoalLifecycleState;
  interactionMode?: "idle" | "goal" | "initial_pose";
  onPoseSelection?: (x: number, y: number, yaw: number) => void;
  onPosePlacement?: (mode: "goal" | "initial_pose", x: number, y: number, yaw: number) => void;
};

type ResolvedFrame = {
  x: number;
  y: number;
  z: number;
  yaw: number;
};

type FrameEdge = {
  parent: string;
  translation: { x: number; y: number; z: number };
  yaw: number;
};

type UrdfVisual = {
  linkName: string;
  xyz: THREE.Vector3;
  rpy: THREE.Vector3;
  geometry:
    | { type: "box"; size: THREE.Vector3 }
    | { type: "cylinder"; radius: number; length: number }
    | { type: "sphere"; radius: number }
    | { type: "mesh"; scale: THREE.Vector3; filename: string };
  color: THREE.Color;
  opacity: number;
};

type UrdfProxyGeometry =
  | { type: "box"; size: THREE.Vector3 }
  | { type: "cylinder"; radius: number; length: number }
  | { type: "sphere"; radius: number };

function rotate2d(x: number, y: number, yaw: number) {
  const cosYaw = Math.cos(yaw);
  const sinYaw = Math.sin(yaw);
  return {
    x: (x * cosYaw) - (y * sinYaw),
    y: (x * sinYaw) + (y * cosYaw),
  };
}

function parseTriplet(value: string | null | undefined, fallback = 0): THREE.Vector3 {
  const tokens = (value ?? "")
    .trim()
    .split(/\s+/)
    .map((token) => Number(token));
  return new THREE.Vector3(
    Number.isFinite(tokens[0]) ? tokens[0] : fallback,
    Number.isFinite(tokens[1]) ? tokens[1] : fallback,
    Number.isFinite(tokens[2]) ? tokens[2] : fallback,
  );
}

function parseUrdfVisuals(robotDescription?: string): UrdfVisual[] {
  if (!robotDescription) {
    return [];
  }

  const parser = new DOMParser();
  const documentNode = parser.parseFromString(robotDescription, "application/xml");
  const robot = documentNode.querySelector("robot");
  if (!robot) {
    return [];
  }

  const materialTable = new Map<string, { color: THREE.Color; opacity: number }>();
  for (const materialNode of Array.from(robot.querySelectorAll(":scope > material"))) {
    const name = materialNode.getAttribute("name");
    const colorNode = materialNode.querySelector("color");
    if (!name || !colorNode) {
      continue;
    }
    const rgba = (colorNode.getAttribute("rgba") ?? "").trim().split(/\s+/).map(Number);
    if (rgba.length < 3) {
      continue;
    }
    materialTable.set(name, {
      color: new THREE.Color(rgba[0] ?? 0.15, rgba[1] ?? 0.15, rgba[2] ?? 0.15),
      opacity: Number.isFinite(rgba[3]) ? rgba[3] : 1,
    });
  }

  const visuals: UrdfVisual[] = [];
  for (const linkNode of Array.from(robot.querySelectorAll("link"))) {
    const linkName = linkNode.getAttribute("name");
    if (!linkName) {
      continue;
    }

    const collisionNode = linkNode.querySelector(":scope > collision");
    let collisionProxy:
      | {
          xyz: THREE.Vector3;
          rpy: THREE.Vector3;
          geometry: UrdfProxyGeometry;
        }
      | null = null;

    if (collisionNode) {
      const collisionOriginNode = collisionNode.querySelector(":scope > origin");
      const collisionGeometryNode = collisionNode.querySelector(":scope > geometry");
      if (collisionGeometryNode) {
        const collisionBoxNode = collisionGeometryNode.querySelector("box");
        const collisionCylinderNode = collisionGeometryNode.querySelector("cylinder");
        const collisionSphereNode = collisionGeometryNode.querySelector("sphere");
        let geometry: UrdfProxyGeometry | null = null;

        if (collisionBoxNode) {
          geometry = {
            type: "box",
            size: parseTriplet(collisionBoxNode.getAttribute("size"), 0.06),
          };
        } else if (collisionCylinderNode) {
          geometry = {
            type: "cylinder",
            radius: Number(collisionCylinderNode.getAttribute("radius") ?? 0.03),
            length: Number(collisionCylinderNode.getAttribute("length") ?? 0.06),
          };
        } else if (collisionSphereNode) {
          geometry = {
            type: "sphere",
            radius: Number(collisionSphereNode.getAttribute("radius") ?? 0.04),
          };
        }

        if (geometry) {
          collisionProxy = {
            xyz: parseTriplet(collisionOriginNode?.getAttribute("xyz"), 0),
            rpy: parseTriplet(collisionOriginNode?.getAttribute("rpy"), 0),
            geometry,
          };
        }
      }
    }

    for (const visualNode of Array.from(linkNode.querySelectorAll(":scope > visual"))) {
      const originNode = visualNode.querySelector(":scope > origin");
      const geometryNode = visualNode.querySelector(":scope > geometry");
      if (!geometryNode) {
        continue;
      }

      const xyz = parseTriplet(originNode?.getAttribute("xyz"), 0);
      const rpy = parseTriplet(originNode?.getAttribute("rpy"), 0);
      let color = new THREE.Color("#202020");
      let opacity = 1;

      const materialNode = visualNode.querySelector(":scope > material");
      const materialColorNode = materialNode?.querySelector("color");
      const materialName = materialNode?.getAttribute("name") ?? "";
      if (materialColorNode) {
        const rgba = (materialColorNode.getAttribute("rgba") ?? "").trim().split(/\s+/).map(Number);
        color = new THREE.Color(rgba[0] ?? 0.15, rgba[1] ?? 0.15, rgba[2] ?? 0.15);
        opacity = Number.isFinite(rgba[3]) ? rgba[3] : 1;
      } else if (materialName && materialTable.has(materialName)) {
        const resolved = materialTable.get(materialName)!;
        color = resolved.color.clone();
        opacity = resolved.opacity;
      }

      const boxNode = geometryNode.querySelector("box");
      const cylinderNode = geometryNode.querySelector("cylinder");
      const sphereNode = geometryNode.querySelector("sphere");
      const meshNode = geometryNode.querySelector("mesh");
      let geometry: UrdfVisual["geometry"] | null = null;

      if (boxNode) {
        geometry = { type: "box", size: parseTriplet(boxNode.getAttribute("size"), 0.06) };
      } else if (cylinderNode) {
        geometry = {
          type: "cylinder",
          radius: Number(cylinderNode.getAttribute("radius") ?? 0.03),
          length: Number(cylinderNode.getAttribute("length") ?? 0.06),
        };
      } else if (sphereNode) {
        geometry = {
          type: "sphere",
          radius: Number(sphereNode.getAttribute("radius") ?? 0.04),
        };
      } else if (meshNode) {
        if (collisionProxy) {
          xyz.copy(collisionProxy.xyz);
          geometry = collisionProxy.geometry;
        } else {
          const scale = parseTriplet(meshNode.getAttribute("scale"), 1);
          geometry = {
            type: "mesh",
            scale,
            filename: meshNode.getAttribute("filename") ?? "",
          };
        }
      }

      if (geometry) {
        visuals.push({ linkName, xyz, rpy, geometry, color, opacity });
      }
    }
  }

  return visuals;
}

function addRobotOutline(object: THREE.Object3D) {
  const mesh = object as THREE.Mesh;
  if (!("geometry" in mesh) || !mesh.geometry) {
    return;
  }

  const outline = new THREE.LineSegments(
    new THREE.EdgesGeometry(mesh.geometry),
    new THREE.LineBasicMaterial({
      color: "#111111",
      transparent: true,
      opacity: 0.35,
      depthTest: false,
      depthWrite: false,
    }),
  );
  outline.renderOrder = 41;
  object.add(outline);
}

function disposeObject(object: THREE.Object3D | null) {
  if (!object) {
    return;
  }

  object.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if ("geometry" in mesh && mesh.geometry) {
      mesh.geometry.dispose();
    }
    if ("material" in mesh && mesh.material) {
      const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
      for (const material of materials) {
        const textureMaterial = material as THREE.MeshBasicMaterial & { map?: THREE.Texture | null };
        if (textureMaterial.map) {
          textureMaterial.map.dispose();
        }
        material.dispose();
      }
    }
  });
}

function buildPathLine(
  points: Array<{ x: number; y: number }>,
  color: string,
  yOffset: number,
): THREE.Group | null {
  if (points.length < 2) {
    return null;
  }

  const curvePoints = points.map((point) => new THREE.Vector3(point.x, yOffset, -point.y));
  const curve = new THREE.CatmullRomCurve3(curvePoints, false, "catmullrom", 0.05);
  const tubeGeometry = new THREE.TubeGeometry(curve, Math.max(points.length * 4, 32), 0.018, 8, false);
  const tubeMaterial = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity: 0.96,
    depthTest: false,
    depthWrite: false,
  });
  const tube = new THREE.Mesh(tubeGeometry, tubeMaterial);
  tube.renderOrder = 20;

  const pointGeometry = new THREE.BufferGeometry().setFromPoints(
    points.map((point) => new THREE.Vector3(point.x, yOffset + 0.01, -point.y)),
  );
  const pointMaterial = new THREE.PointsMaterial({
    color,
    size: 0.12,
    transparent: true,
    opacity: 1,
    sizeAttenuation: true,
    depthTest: false,
    depthWrite: false,
  });
  const pointCloud = new THREE.Points(pointGeometry, pointMaterial);
  pointCloud.renderOrder = 21;

  const group = new THREE.Group();
  group.add(tube, pointCloud);
  return group;
}

function buildArrowPoseMarker(
  goalMarker: { x: number; y: number; yaw: number },
  colors: { primary: string; accent: string },
): THREE.Group {
  const group = new THREE.Group();

  const tail = new THREE.Mesh(
    new THREE.PlaneGeometry(0.28, 0.036),
    new THREE.MeshBasicMaterial({
      color: colors.primary,
      side: THREE.DoubleSide,
      depthTest: false,
      depthWrite: false,
    }),
  );
  tail.rotation.x = -Math.PI / 2;
  tail.position.set(0.16, 0.06, 0);
  tail.renderOrder = 24;

  const head = new THREE.Mesh(
    new THREE.ConeGeometry(0.07, 0.16, 3),
    new THREE.MeshBasicMaterial({
      color: colors.primary,
      depthTest: false,
      depthWrite: false,
    }),
  );
  head.rotation.x = Math.PI / 2;
  head.rotation.z = -Math.PI / 2;
  head.position.set(0.36, 0.062, 0);
  head.renderOrder = 25;

  const origin = new THREE.Mesh(
    new THREE.CircleGeometry(0.055, 24),
    new THREE.MeshBasicMaterial({
      color: colors.accent,
      depthTest: false,
      depthWrite: false,
    }),
  );
  origin.rotation.x = -Math.PI / 2;
  origin.position.set(0, 0.054, 0);
  origin.renderOrder = 26;

  const originOutline = new THREE.Mesh(
    new THREE.RingGeometry(0.05, 0.065, 28),
    new THREE.MeshBasicMaterial({
      color: colors.primary,
      side: THREE.DoubleSide,
      depthTest: false,
      depthWrite: false,
    }),
  );
  originOutline.rotation.x = -Math.PI / 2;
  originOutline.position.set(0, 0.058, 0);
  originOutline.renderOrder = 27;

  group.add(tail, head, origin, originOutline);
  group.position.set(goalMarker.x, 0, -goalMarker.y);
  group.rotation.y = -goalMarker.yaw;
  return group;
}

function buildInitialPoseMarker(goalMarker: { x: number; y: number; yaw: number }): THREE.Group {
  return buildArrowPoseMarker(goalMarker, {
    primary: "#2aa84a",
    accent: "#d9f7df",
  });
}

function buildGoalMarker(goalMarker: { x: number; y: number; yaw: number; kind: "goal" | "initial_pose" }): THREE.Group {
  if (goalMarker.kind === "initial_pose") {
    return buildInitialPoseMarker(goalMarker);
  }
  return buildArrowPoseMarker(goalMarker, {
    primary: "#d62828",
    accent: "#ffe3e3",
  });
}

function buildFootprintOverlay(
  footprintPolygon: number[] | undefined,
  robotPose: Pose | undefined,
  options?: {
    fillColor?: string;
    outlineColor?: string;
    fillOpacity?: number;
    yOffset?: number;
    renderOrder?: number;
  },
): THREE.Group | null {
  if (!robotPose || !footprintPolygon || footprintPolygon.length < 6 || footprintPolygon.length % 2 !== 0) {
    return null;
  }

  const fillColor = options?.fillColor ?? "#2b7cff";
  const outlineColor = options?.outlineColor ?? "#134ed0";
  const fillOpacity = options?.fillOpacity ?? 0.14;
  const yOffset = options?.yOffset ?? 0.042;
  const renderOrder = options?.renderOrder ?? 18;

  const shape = new THREE.Shape();
  shape.moveTo(footprintPolygon[0], -footprintPolygon[1]);
  for (let index = 2; index < footprintPolygon.length; index += 2) {
    shape.lineTo(footprintPolygon[index], -footprintPolygon[index + 1]);
  }
  shape.closePath();

  const fill = new THREE.Mesh(
    new THREE.ShapeGeometry(shape),
    new THREE.MeshBasicMaterial({
      color: fillColor,
      transparent: true,
      opacity: fillOpacity,
      side: THREE.DoubleSide,
      depthTest: false,
      depthWrite: false,
    }),
  );
  fill.rotation.x = -Math.PI / 2;
  fill.position.y = yOffset;
  fill.renderOrder = renderOrder;

  const outlinePoints: THREE.Vector3[] = [];
  for (let index = 0; index < footprintPolygon.length; index += 2) {
    outlinePoints.push(new THREE.Vector3(footprintPolygon[index], 0.048, -footprintPolygon[index + 1]));
  }
  outlinePoints.push(new THREE.Vector3(footprintPolygon[0], 0.048, -footprintPolygon[1]));
  const outline = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(outlinePoints),
    new THREE.LineBasicMaterial({
      color: outlineColor,
      transparent: true,
      opacity: 0.85,
      depthTest: false,
      depthWrite: false,
    }),
  );
  outline.renderOrder = renderOrder + 1;

  const group = new THREE.Group();
  group.add(fill, outline);
  group.position.set(robotPose.position.x, 0, -robotPose.position.y);
  group.rotation.y = -robotPose.orientation.yaw;
  return group;
}

function buildBlockedLinkOverlay(
  robotPose: Pose | undefined,
  blockedPose: Pose | undefined,
): THREE.Group | null {
  if (!robotPose || !blockedPose) {
    return null;
  }

  const start = new THREE.Vector3(robotPose.position.x, 0.07, -robotPose.position.y);
  const end = new THREE.Vector3(blockedPose.position.x, 0.07, -blockedPose.position.y);
  const line = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([start, end]),
    new THREE.LineDashedMaterial({
      color: "#1e1e1e",
      transparent: true,
      opacity: 0.95,
      dashSize: 0.08,
      gapSize: 0.05,
      depthTest: false,
      depthWrite: false,
    }),
  );
  line.computeLineDistances();
  line.renderOrder = 23;

  const marker = new THREE.Mesh(
    new THREE.CircleGeometry(0.05, 20),
    new THREE.MeshBasicMaterial({
      color: "#2aff53",
      transparent: true,
      opacity: 0.22,
      depthTest: false,
      depthWrite: false,
    }),
  );
  marker.rotation.x = -Math.PI / 2;
  marker.position.copy(end);
  marker.position.y = 0.065;
  marker.renderOrder = 23;

  const group = new THREE.Group();
  group.add(line, marker);
  return group;
}

function createScanSpriteTexture(): THREE.CanvasTexture {
  const canvas = document.createElement("canvas");
  canvas.width = 32;
  canvas.height = 32;
  const context = canvas.getContext("2d");
  if (!context) {
    const fallback = new THREE.CanvasTexture(canvas);
    fallback.needsUpdate = true;
    return fallback;
  }

  context.clearRect(0, 0, canvas.width, canvas.height);
  const gradient = context.createRadialGradient(16, 16, 1, 16, 16, 14);
  gradient.addColorStop(0, "rgba(29,156,44,1)");
  gradient.addColorStop(0.55, "rgba(53,205,74,0.92)");
  gradient.addColorStop(1, "rgba(53,205,74,0)");
  context.fillStyle = gradient;
  context.beginPath();
  context.arc(16, 16, 14, 0, Math.PI * 2);
  context.fill();

  const texture = new THREE.CanvasTexture(canvas);
  texture.needsUpdate = true;
  return texture;
}

function interpolateChannel(start: number, end: number, ratio: number) {
  return Math.round(start + ((end - start) * ratio));
}

function buildOccupancyTexture(
  grid: OccupancyGridMessage,
  palette: "map" | "global_costmap" | "local_costmap",
): THREE.Texture {
  const { width, height } = grid.info;
  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const context = canvas.getContext("2d");
  if (!context) {
    const texture = new THREE.Texture();
    texture.needsUpdate = true;
    return texture;
  }

  const imageData = context.createImageData(width, height);
  const rgba = imageData.data;

  for (let row = 0; row < height; row += 1) {
    for (let col = 0; col < width; col += 1) {
      const sourceIndex = ((height - 1 - row) * width) + col;
      const targetIndex = ((row * width) + col) * 4;
      const value = grid.data[sourceIndex] ?? -1;
      let red = 0;
      let green = 0;
      let blue = 0;
      let alpha = 0;

      if (palette === "map") {
        if (value < 0) {
          red = 201;
          green = 201;
          blue = 201;
          alpha = 255;
        } else if (value >= 50) {
          red = 36;
          green = 22;
          blue = 48;
          alpha = 255;
        } else {
          red = 255;
          green = 255;
          blue = 255;
          alpha = 255;
        }
      } else {
        if (value > 0) {
          const normalized = Math.max(0, Math.min(1, value / 100));
          alpha = Math.max(36, Math.round((palette === "global_costmap" ? 156 : 168) * normalized));
          if (palette === "global_costmap") {
            if (normalized < 0.45) {
              const ratio = normalized / 0.45;
              red = interpolateChannel(214, 118, ratio);
              green = interpolateChannel(250, 235, ratio);
              blue = interpolateChannel(244, 233, ratio);
            } else {
              const ratio = (normalized - 0.45) / 0.55;
              red = interpolateChannel(118, 76, ratio);
              green = interpolateChannel(235, 185, ratio);
              blue = interpolateChannel(233, 206, ratio);
            }
          } else {
            if (normalized < 0.45) {
              const ratio = normalized / 0.45;
              red = interpolateChannel(255, 232, ratio);
              green = interpolateChannel(214, 112, ratio);
              blue = interpolateChannel(226, 182, ratio);
            } else {
              const ratio = (normalized - 0.45) / 0.55;
              red = interpolateChannel(232, 138, ratio);
              green = interpolateChannel(112, 30, ratio);
              blue = interpolateChannel(182, 122, ratio);
            }
          }
        }
      }

      rgba[targetIndex] = red;
      rgba[targetIndex + 1] = green;
      rgba[targetIndex + 2] = blue;
      rgba[targetIndex + 3] = alpha;
    }
  }

  context.putImageData(imageData, 0, 0);

  const texture = new THREE.CanvasTexture(canvas);
  texture.needsUpdate = true;
  texture.magFilter = THREE.NearestFilter;
  texture.minFilter = THREE.NearestFilter;
  texture.generateMipmaps = false;
  texture.flipY = true;
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function buildOccupancyMesh(
  grid: OccupancyGridMessage,
  palette: "map" | "global_costmap" | "local_costmap",
  yOffset: number,
): THREE.Mesh {
  const widthMeters = grid.info.width * grid.info.resolution;
  const heightMeters = grid.info.height * grid.info.resolution;
  const geometry = new THREE.PlaneGeometry(widthMeters, heightMeters);
  const isMap = palette === "map";
  const material = new THREE.MeshBasicMaterial({
    map: buildOccupancyTexture(grid, palette),
    transparent: !isMap,
    depthWrite: isMap,
    side: THREE.DoubleSide,
    toneMapped: false,
  });
  material.depthTest = false;
  const mesh = new THREE.Mesh(geometry, material);
  const originYaw = grid.info.origin.orientation.yaw;
  const rotatedOffset = rotate2d(widthMeters / 2, heightMeters / 2, originYaw);
  const yawQuaternion = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 1, 0),
    -originYaw,
  );
  const planeQuaternion = new THREE.Quaternion().setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0));

  mesh.quaternion.multiplyQuaternions(yawQuaternion, planeQuaternion);
  mesh.position.set(
    grid.info.origin.position.x + rotatedOffset.x,
    yOffset,
    -(grid.info.origin.position.y + rotatedOffset.y),
  );
  mesh.renderOrder = palette === "map" ? 1 : palette === "global_costmap" ? 2 : 3;
  return mesh;
}

function buildFrameLookup(tf?: TfMessage, tfStatic?: TfMessage) {
  const lookup = new Map<string, FrameEdge>();
  for (const transform of tfStatic?.transforms ?? []) {
    lookup.set(transform.child_frame_id, {
      parent: transform.header.frame_id,
      translation: transform.translation,
      yaw: transform.rotation.yaw,
    });
  }
  for (const transform of tf?.transforms ?? []) {
    lookup.set(transform.child_frame_id, {
      parent: transform.header.frame_id,
      translation: transform.translation,
      yaw: transform.rotation.yaw,
    });
  }
  return lookup;
}

function resolveFrame(
  frameId: string,
  lookup: Map<string, FrameEdge>,
  robotPose?: Pose,
  cache = new Map<string, ResolvedFrame | null>(),
  depth = 0,
): ResolvedFrame | null {
  if (cache.has(frameId)) {
    return cache.get(frameId) ?? null;
  }

  if (depth > 24) {
    cache.set(frameId, null);
    return null;
  }

  if (frameId === "map") {
    const identity = { x: 0, y: 0, z: 0, yaw: 0 };
    cache.set(frameId, identity);
    return identity;
  }

  const edge = lookup.get(frameId);
  if (!edge) {
    if (robotPose && (frameId === "base_footprint" || frameId === "base_link")) {
      const resolved = {
        x: robotPose.position.x,
        y: robotPose.position.y,
        z: robotPose.position.z,
        yaw: robotPose.orientation.yaw,
      };
      cache.set(frameId, resolved);
      return resolved;
    }
    cache.set(frameId, null);
    return null;
  }

  const parent = resolveFrame(edge.parent, lookup, robotPose, cache, depth + 1);
  if (!parent) {
    cache.set(frameId, null);
    return null;
  }

  const rotated = rotate2d(edge.translation.x, edge.translation.y, parent.yaw);
  const resolved = {
    x: parent.x + rotated.x,
    y: parent.y + rotated.y,
    z: parent.z + edge.translation.z,
    yaw: parent.yaw + edge.yaw,
  };
  cache.set(frameId, resolved);
  return resolved;
}

function buildScanPoints(
  scan: LaserScanMessage | undefined,
  tf: TfMessage | undefined,
  tfStatic: TfMessage | undefined,
  robotPose: Pose | undefined,
): THREE.Points | null {
  if (!scan || scan.ranges.length === 0) {
    return null;
  }

  const lookup = buildFrameLookup(tf, tfStatic);
  const scanFrame = resolveFrame(scan.header.frame_id, lookup, robotPose);
  const fallbackPose = robotPose
    ? {
        x: robotPose.position.x,
        y: robotPose.position.y,
        z: robotPose.position.z,
        yaw: robotPose.orientation.yaw,
      }
    : { x: 0, y: 0, z: 0, yaw: 0 };
  const pose = scanFrame ?? fallbackPose;
  const points: number[] = [];

  for (let index = 0; index < scan.ranges.length; index += 1) {
    const range = scan.ranges[index];
    if (!Number.isFinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }

    const angle = scan.angle_min + (index * scan.angle_increment);
    const localX = Math.cos(angle) * range;
    const localY = Math.sin(angle) * range;
    const rotated = rotate2d(localX, localY, pose.yaw);
    points.push(pose.x + rotated.x, 0.08, -(pose.y + rotated.y));
  }

  if (points.length === 0) {
    return null;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(points, 3));
  const material = new THREE.PointsMaterial({
    color: "#25d038",
    size: 0.18,
    map: createScanSpriteTexture(),
    transparent: true,
    opacity: 1,
    sizeAttenuation: true,
    depthTest: false,
    depthWrite: false,
    blending: THREE.NormalBlending,
    alphaTest: 0.04,
  });
  const cloud = new THREE.Points(geometry, material);
  cloud.renderOrder = 12;
  return cloud;
}

function buildTfGroup(
  tf: TfMessage | undefined,
  tfStatic: TfMessage | undefined,
  robotPose: Pose | undefined,
  map: OccupancyGridMessage | undefined,
): THREE.Group | null {
  const lookup = buildFrameLookup(tf, tfStatic);
  const transforms = [...(tfStatic?.transforms ?? []), ...(tf?.transforms ?? [])];
  if (transforms.length === 0) {
    return null;
  }

  const group = new THREE.Group();
  const cache = new Map<string, ResolvedFrame | null>();

  const buildAxes = (x: number, y: number, z: number, yaw: number, color?: string) => {
    const frameGroup = new THREE.Group();
    frameGroup.position.set(x, y, -z);
    frameGroup.rotation.y = -yaw;

    const hub = new THREE.Mesh(
      new THREE.SphereGeometry(0.028, 12, 12),
      new THREE.MeshBasicMaterial({
        color: color ?? "#8bff8f",
        depthTest: false,
        depthWrite: false,
      }),
    );
    hub.renderOrder = 16;
    frameGroup.add(hub);

    const axisRadius = 0.008;
    const xAxis = new THREE.Mesh(
      new THREE.CylinderGeometry(axisRadius, axisRadius, 0.24, 10),
      new THREE.MeshBasicMaterial({
        color: "#ff4d4d",
        depthTest: false,
        depthWrite: false,
      }),
    );
    xAxis.rotation.z = -Math.PI / 2;
    xAxis.position.x = 0.12;
    xAxis.renderOrder = 17;

    const yAxis = new THREE.Mesh(
      new THREE.CylinderGeometry(axisRadius, axisRadius, 0.24, 10),
      new THREE.MeshBasicMaterial({
        color: "#2fd06a",
        depthTest: false,
        depthWrite: false,
      }),
    );
    yAxis.rotation.x = Math.PI / 2;
    yAxis.position.z = -0.12;
    yAxis.renderOrder = 17;

    const zAxis = new THREE.Mesh(
      new THREE.CylinderGeometry(axisRadius, axisRadius, 0.18, 10),
      new THREE.MeshBasicMaterial({
        color: "#4f8fff",
        depthTest: false,
        depthWrite: false,
      }),
    );
    zAxis.position.y = 0.09;
    zAxis.renderOrder = 17;

    frameGroup.add(xAxis, yAxis, zAxis);
    return frameGroup;
  };

  group.add(buildAxes(0, 0.1, 0, 0, "#ffd36b"));

  for (const transform of transforms) {
    const child = resolveFrame(transform.child_frame_id, lookup, robotPose, cache);
    const parent = resolveFrame(transform.header.frame_id, lookup, robotPose, cache);
    if (!child) {
      continue;
    }

    const frameGroup = buildAxes(
      child.x,
      0.1,
      child.y,
      child.yaw,
      tfStatic?.transforms.includes(transform) ? "#7aa1ff" : "#8bff8f",
    );
    group.add(frameGroup);

    if (parent) {
      const lineGeometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(parent.x, 0.06, -parent.y),
        new THREE.Vector3(child.x, 0.06, -child.y),
      ]);
      const line = new THREE.Line(
        lineGeometry,
        new THREE.LineBasicMaterial({
          color: tfStatic?.transforms.includes(transform) ? "#5b7dff" : "#56ff86",
          transparent: true,
          opacity: 0.95,
        }),
      );
      line.renderOrder = 15;
      group.add(line);
    }
  }

  return group.children.length > 0 ? group : null;
}

function buildRobotVisualMesh(visual: UrdfVisual): THREE.Object3D {
  let object: THREE.Object3D;
  const descriptor =
    visual.geometry.type === "mesh"
      ? `${visual.linkName} ${visual.geometry.filename}`.toLowerCase()
      : visual.linkName.toLowerCase();
  const material = new THREE.MeshBasicMaterial({
    color: visual.color,
    transparent: visual.opacity < 0.999,
    opacity: visual.opacity,
    depthTest: false,
    depthWrite: false,
  });

  switch (visual.geometry.type) {
    case "box":
      object = new THREE.Mesh(
        new THREE.BoxGeometry(
          Math.max(visual.geometry.size.x, 0.01),
          Math.max(visual.geometry.size.z, 0.01),
          Math.max(visual.geometry.size.y, 0.01),
        ),
        material,
      );
      break;
    case "cylinder":
      object = new THREE.Mesh(
        new THREE.CylinderGeometry(
          Math.max(visual.geometry.radius, 0.005),
          Math.max(visual.geometry.radius, 0.005),
          Math.max(visual.geometry.length, 0.01),
          20,
        ),
        material,
      );
      break;
    case "sphere":
      object = new THREE.Mesh(
        new THREE.SphereGeometry(Math.max(visual.geometry.radius, 0.005), 20, 20),
        material,
      );
      break;
    case "mesh":
      {
        if (descriptor.includes("wheel")) {
          object = new THREE.Mesh(
            new THREE.CylinderGeometry(0.033, 0.033, 0.018, 24),
            material,
          );
        } else if (descriptor.includes("caster") || descriptor.includes("ball")) {
          object = new THREE.Mesh(
            new THREE.SphereGeometry(0.018, 20, 20),
            material,
          );
        } else if (descriptor.includes("laser") || descriptor.includes("lidar") || descriptor.includes("scan")) {
          object = new THREE.Mesh(
            new THREE.CylinderGeometry(0.03, 0.03, 0.012, 28),
            material,
          );
        } else if (descriptor.includes("base_link")) {
          object = new THREE.Mesh(
            new THREE.BoxGeometry(0.165, 0.022, 0.145),
            material,
          );
        } else if (descriptor.includes("plate") || descriptor.includes("burger")) {
          object = new THREE.Mesh(
            new THREE.BoxGeometry(0.14, 0.012, 0.118),
            material,
          );
        } else if (descriptor.includes("base")) {
          object = new THREE.Mesh(
            new THREE.BoxGeometry(0.15, 0.016, 0.13),
            material,
          );
        } else {
          object = new THREE.Mesh(
            new THREE.BoxGeometry(
              Math.max(0.14 * visual.geometry.scale.x, 0.05),
              Math.max(0.05 * visual.geometry.scale.z, 0.03),
              Math.max(0.14 * visual.geometry.scale.y, 0.05),
            ),
            material,
          );
        }
      }
      break;
  }

  const mesh = object as THREE.Mesh;
  mesh.renderOrder = 40;
  addRobotOutline(mesh);
  return object;
}

function buildRobotModelGroup(
  robotDescription: string | undefined,
  tf: TfMessage | undefined,
  tfStatic: TfMessage | undefined,
  robotPose: Pose | undefined,
): THREE.Group {
  const visuals = parseUrdfVisuals(robotDescription);
  const group = new THREE.Group();
  const lookup = buildFrameLookup(tf, tfStatic);
  const cache = new Map<string, ResolvedFrame | null>();

  if (visuals.length === 0) {
    const fallback = new THREE.Group();
    const body = new THREE.Mesh(
      new THREE.CylinderGeometry(0.082, 0.082, 0.028, 36),
      new THREE.MeshBasicMaterial({
        color: "#101010",
        depthTest: false,
        depthWrite: false,
      }),
    );
    body.position.y = 0.055;
    body.renderOrder = 40;

    const bodyOutline = new THREE.Mesh(
      new THREE.RingGeometry(0.078, 0.092, 40),
      new THREE.MeshBasicMaterial({
        color: "#f2f5f8",
        side: THREE.DoubleSide,
        depthTest: false,
        depthWrite: false,
      }),
    );
    bodyOutline.rotation.x = -Math.PI / 2;
    bodyOutline.position.y = 0.07;
    bodyOutline.renderOrder = 41;

    const lidar = new THREE.Mesh(
      new THREE.CylinderGeometry(0.028, 0.028, 0.03, 24),
      new THREE.MeshBasicMaterial({
        color: "#151515",
        depthTest: false,
        depthWrite: false,
      }),
    );
    lidar.position.set(0.0, 0.085, 0.0);
    lidar.renderOrder = 42;

    const heading = new THREE.Mesh(
      new THREE.ConeGeometry(0.045, 0.13, 3),
      new THREE.MeshBasicMaterial({
        color: "#111111",
        depthTest: false,
        depthWrite: false,
      }),
    );
    heading.rotation.z = -Math.PI / 2;
    heading.position.set(0.115, 0.07, 0);
    heading.renderOrder = 42;

    fallback.add(body, bodyOutline, lidar, heading);
    group.add(fallback);
    if (robotPose) {
      group.position.set(robotPose.position.x, 0, -robotPose.position.y);
      group.rotation.y = -robotPose.orientation.yaw;
    }
    return group;
  }

  for (const visual of visuals) {
    const frame = resolveFrame(visual.linkName, lookup, robotPose, cache);
    if (!frame) {
      continue;
    }
    const rotatedOrigin = rotate2d(visual.xyz.x, visual.xyz.y, frame.yaw);
    const mesh = buildRobotVisualMesh(visual);
    mesh.position.set(
      frame.x + rotatedOrigin.x,
      frame.z + visual.xyz.z + 0.01,
      -(frame.y + rotatedOrigin.y),
    );
    mesh.rotation.set(visual.rpy.x, -(frame.yaw + visual.rpy.z), visual.rpy.y, "XYZ");
    group.add(mesh);
  }

  return group;
}

export function SceneViewport({
  state,
  layerVisibility,
  goalMarker,
  goalLifecycle = "Idle",
  interactionMode = "idle",
  onPoseSelection,
  onPosePlacement,
}: SceneViewportProps) {
  const viewportRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const robotRef = useRef<THREE.Group | null>(null);
  const gridRef = useRef<THREE.GridHelper | null>(null);
  const globalPathRef = useRef<THREE.Group | null>(null);
  const localPathRef = useRef<THREE.Group | null>(null);
  const goalMarkerRef = useRef<THREE.Group | null>(null);
  const previewMarkerRef = useRef<THREE.Group | null>(null);
  const mapMeshRef = useRef<THREE.Mesh | null>(null);
  const globalCostmapMeshRef = useRef<THREE.Mesh | null>(null);
  const localCostmapMeshRef = useRef<THREE.Mesh | null>(null);
  const footprintRef = useRef<THREE.Group | null>(null);
  const blockedFootprintRef = useRef<THREE.Group | null>(null);
  const blockedLinkRef = useRef<THREE.Group | null>(null);
  const scanRef = useRef<THREE.Points | null>(null);
  const tfGroupRef = useRef<THREE.Group | null>(null);
  const lastCenteredMapSignatureRef = useRef<string>("");
  const renderRef = useRef<(() => void) | null>(null);
  const interactionModeRef = useRef<SceneViewportProps["interactionMode"]>("idle");
  const onPoseSelectionRef = useRef<SceneViewportProps["onPoseSelection"]>(undefined);
  const onPosePlacementRef = useRef<SceneViewportProps["onPosePlacement"]>(undefined);
  const interactionStateRef = useRef<{
    mode: "goal" | "initial_pose";
    start: THREE.Vector3;
  } | null>(null);

  useEffect(() => {
    interactionModeRef.current = interactionMode;
  }, [interactionMode]);

  useEffect(() => {
    onPoseSelectionRef.current = onPoseSelection;
  }, [onPoseSelection]);

  useEffect(() => {
    onPosePlacementRef.current = onPosePlacement;
  }, [onPosePlacement]);

  useEffect(() => {
    if (!viewportRef.current) {
      return;
    }

    const scene = new THREE.Scene();
    scene.background = new THREE.Color("#dddddd");

    const renderer = new THREE.WebGLRenderer({
      antialias: false,
      alpha: false,
      powerPreference: "low-power",
    });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.25));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.setSize(viewportRef.current.clientWidth, viewportRef.current.clientHeight);
    viewportRef.current.appendChild(renderer.domElement);

    const camera = new THREE.PerspectiveCamera(
      14,
      viewportRef.current.clientWidth / Math.max(viewportRef.current.clientHeight, 1),
      0.1,
      200,
    );
    camera.zoom = 0.4;
    camera.up.set(0, 0, -1);
    camera.position.set(0, 28, 0.001);
    camera.lookAt(0, 0, 0);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.screenSpacePanning = true;
    controls.enablePan = true;
    controls.enableRotate = true;
    controls.enableZoom = true;
    controls.minDistance = 2;
    controls.maxDistance = 80;
    controls.target.set(0, 0, 0);
    controls.addEventListener("change", () => {
      renderer.render(scene, camera);
    });
    controls.update();

    const ambientLight = new THREE.AmbientLight("#ffffff", 1.18);
    const keyLight = new THREE.DirectionalLight("#d8e6ff", 0.84);
    keyLight.position.set(12, 22, 10);
    scene.add(ambientLight, keyLight);

    const grid = new THREE.GridHelper(60, 60, "#9e9e9e", "#c9c9c9");
    grid.position.y = 0.045;
    grid.renderOrder = 10;
    grid.material.depthTest = false;
    grid.material.transparent = true;
    grid.material.opacity = 0.58;
    scene.add(grid);
    gridRef.current = grid;

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(64, 64),
      new THREE.MeshStandardMaterial({
        color: "#d8d8d8",
        metalness: 0.02,
        roughness: 1,
      }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.015;
    floor.renderOrder = -10;
    scene.add(floor);

    const robot = new THREE.Group();
    scene.add(robot);

    const renderScene = () => {
      controls.update();
      renderer.render(scene, camera);
    };
    renderRef.current = renderScene;
    renderScene();

    const handleResize = () => {
      if (!viewportRef.current) {
        return;
      }
      camera.aspect = viewportRef.current.clientWidth / Math.max(viewportRef.current.clientHeight, 1);
      camera.updateProjectionMatrix();
      renderer.setSize(viewportRef.current.clientWidth, viewportRef.current.clientHeight);
      renderScene();
    };

    const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    const raycaster = new THREE.Raycaster();
    const pointer = new THREE.Vector2();
    const intersection = new THREE.Vector3();
    const readGroundPoint = (event: PointerEvent) => {
      const bounds = renderer.domElement.getBoundingClientRect();
      pointer.x = ((event.clientX - bounds.left) / bounds.width) * 2 - 1;
      pointer.y = -(((event.clientY - bounds.top) / bounds.height) * 2 - 1);
      raycaster.setFromCamera(pointer, camera);
      return raycaster.ray.intersectPlane(groundPlane, intersection) ? intersection.clone() : null;
    };

    const clearPreview = () => {
      if (previewMarkerRef.current) {
        scene.remove(previewMarkerRef.current);
        disposeObject(previewMarkerRef.current);
        previewMarkerRef.current = null;
      }
    };

    const updatePreview = (mode: "goal" | "initial_pose", start: THREE.Vector3, current: THREE.Vector3) => {
      clearPreview();
      const dx = current.x - start.x;
      const dy = -(current.z - start.z);
      const yaw = Math.atan2(dy, dx || 0.0001);
      previewMarkerRef.current = buildGoalMarker({
        x: start.x,
        y: -start.z,
        yaw,
        kind: mode,
      });
      previewMarkerRef.current.renderOrder = 29;
      scene.add(previewMarkerRef.current);
      renderScene();
    };

    const handlePointerDown = (event: PointerEvent) => {
      const currentMode = interactionModeRef.current;
      if (currentMode === "idle" || event.button !== 0) {
        return;
      }
      const point = readGroundPoint(event);
      if (!point) {
        return;
      }
      controls.enabled = false;
      interactionStateRef.current = {
        mode: currentMode,
        start: point,
      };
      updatePreview(currentMode, point, point);
      event.preventDefault();
    };

    const handlePointerMove = (event: PointerEvent) => {
      if (!interactionStateRef.current) {
        return;
      }
      const point = readGroundPoint(event);
      if (!point) {
        return;
      }
      updatePreview(interactionStateRef.current.mode, interactionStateRef.current.start, point);
      event.preventDefault();
    };

    const handlePointerUp = (event: PointerEvent) => {
      if (!interactionStateRef.current) {
        return;
      }
      const point = readGroundPoint(event) ?? interactionStateRef.current.start;
      const start = interactionStateRef.current.start;
      const mode = interactionStateRef.current.mode;
      interactionStateRef.current = null;
      controls.enabled = true;
      clearPreview();
      const dx = point.x - start.x;
      const dy = -(point.z - start.z);
      const yaw = Math.atan2(dy, dx || 0.0001);
      onPoseSelectionRef.current?.(start.x, -start.z, yaw);
      onPosePlacementRef.current?.(mode, start.x, -start.z, yaw);
      event.preventDefault();
    };

    renderer.domElement.addEventListener("pointerdown", handlePointerDown);
    window.addEventListener("pointermove", handlePointerMove);
    window.addEventListener("pointerup", handlePointerUp);
    renderer.domElement.addEventListener("contextmenu", (event) => event.preventDefault());
    window.addEventListener("resize", handleResize);

    sceneRef.current = scene;
    rendererRef.current = renderer;
    cameraRef.current = camera;
    controlsRef.current = controls;
    robotRef.current = robot;

    return () => {
      clearPreview();
      controls.enabled = true;
      renderer.domElement.removeEventListener("pointerdown", handlePointerDown);
      window.removeEventListener("pointermove", handlePointerMove);
      window.removeEventListener("pointerup", handlePointerUp);
      window.removeEventListener("resize", handleResize);
      renderRef.current = null;
      controls.dispose();
      controlsRef.current = null;
      disposeObject(globalPathRef.current);
      disposeObject(localPathRef.current);
      disposeObject(mapMeshRef.current);
      disposeObject(globalCostmapMeshRef.current);
      disposeObject(localCostmapMeshRef.current);
      disposeObject(footprintRef.current);
      disposeObject(blockedFootprintRef.current);
      disposeObject(blockedLinkRef.current);
      disposeObject(scanRef.current);
      disposeObject(tfGroupRef.current);
      disposeObject(goalMarkerRef.current);
      disposeObject(robot);
      renderer.dispose();
      scene.clear();
      viewportRef.current?.removeChild(renderer.domElement);
    };
  }, []);

  useEffect(() => {
    const scene = sceneRef.current;
    const robot = robotRef.current;
    if (!scene || !robot) {
      return;
    }

    const robotPose = state.robot_pose;
    if (gridRef.current) {
      gridRef.current.visible = layerVisibility.grid;
    }
    robot.visible = layerVisibility.robot;
    while (robot.children.length > 0) {
      const child = robot.children[0];
      robot.remove(child);
      disposeObject(child);
    }
    const robotModel = buildRobotModelGroup(
      state.robot_description?.data,
      state.tf,
      state.tf_static,
      robotPose,
    );
    robot.add(robotModel);

    disposeObject(globalPathRef.current);
    disposeObject(localPathRef.current);
    if (globalPathRef.current) {
      scene.remove(globalPathRef.current);
      globalPathRef.current = null;
    }
    if (localPathRef.current) {
      scene.remove(localPathRef.current);
      localPathRef.current = null;
    }
    if (goalMarkerRef.current) {
      scene.remove(goalMarkerRef.current);
      disposeObject(goalMarkerRef.current);
      goalMarkerRef.current = null;
    }

    const showLivePlans =
      goalLifecycle === "Running" ||
      goalLifecycle === "Recovering" ||
      goalLifecycle === "Canceling";

    if (showLivePlans) {
      const globalPath = state.global_path?.poses.map((pose) => ({
        x: pose.position.x,
        y: pose.position.y,
      })) ?? [];
      const localPath = state.local_path?.poses.map((pose) => ({
        x: pose.position.x,
        y: pose.position.y,
      })) ?? [];

      globalPathRef.current = buildPathLine(globalPath, "#23d9ff", 0.06);
      localPathRef.current = buildPathLine(localPath, "#95dd00", 0.08);
      if (globalPathRef.current) {
        globalPathRef.current.visible = layerVisibility.globalPlan;
        scene.add(globalPathRef.current);
      }
      if (localPathRef.current) {
        localPathRef.current.visible = layerVisibility.localPlan;
        scene.add(localPathRef.current);
      }
    }
    if (goalMarker) {
      goalMarkerRef.current = buildGoalMarker(goalMarker);
      goalMarkerRef.current.visible = true;
      scene.add(goalMarkerRef.current);
    }

    for (const [ref, grid, palette, yOffset] of [
      [mapMeshRef, state.map, "map", 0.005],
      [globalCostmapMeshRef, state.global_costmap, "global_costmap", 0.02],
      [localCostmapMeshRef, state.local_costmap, "local_costmap", 0.03],
    ] as const) {
      if (ref.current) {
        scene.remove(ref.current);
        disposeObject(ref.current);
        ref.current = null;
      }
      if (grid) {
        ref.current = buildOccupancyMesh(grid, palette, yOffset);
        ref.current.visible =
          palette === "map" ? layerVisibility.map :
          palette === "global_costmap" ? layerVisibility.globalCostmap :
          layerVisibility.localCostmap;
        scene.add(ref.current);
      }
    }

    if (footprintRef.current) {
      scene.remove(footprintRef.current);
      disposeObject(footprintRef.current);
      footprintRef.current = null;
    }
    footprintRef.current = buildFootprintOverlay(
      state.robot_description?.footprint_polygon,
      state.robot_pose,
      state.motion_status?.costmap_blocked || state.motion_status?.safety_gate_blocked
        ? {
            fillColor: state.motion_status?.safety_gate_blocked ? "#ff4b4b" : "#3f7dff",
            outlineColor: state.motion_status?.safety_gate_blocked ? "#9b1818" : "#1a3d96",
            fillOpacity: 0.16,
            yOffset: 0.044,
            renderOrder: 19,
          }
        : undefined,
    );
    if (footprintRef.current) {
      footprintRef.current.visible = layerVisibility.footprint;
      scene.add(footprintRef.current);
    }

    if (blockedFootprintRef.current) {
      scene.remove(blockedFootprintRef.current);
      disposeObject(blockedFootprintRef.current);
      blockedFootprintRef.current = null;
    }
    if (state.motion_status?.has_blocked_pose) {
      blockedFootprintRef.current = buildFootprintOverlay(
        state.robot_description?.footprint_polygon,
        state.motion_status.blocked_pose,
        {
          fillColor: "#4cff63",
          outlineColor: "#0e8d22",
          fillOpacity: 0.1,
          yOffset: 0.052,
          renderOrder: 22,
        },
      );
    }
    if (blockedFootprintRef.current) {
      blockedFootprintRef.current.visible = true;
      scene.add(blockedFootprintRef.current);
    }

    if (blockedLinkRef.current) {
      scene.remove(blockedLinkRef.current);
      disposeObject(blockedLinkRef.current);
      blockedLinkRef.current = null;
    }
    if (state.motion_status?.has_blocked_pose) {
      blockedLinkRef.current = buildBlockedLinkOverlay(
        state.robot_pose,
        state.motion_status.blocked_pose,
      );
    }
    if (blockedLinkRef.current) {
      blockedLinkRef.current.visible = true;
      scene.add(blockedLinkRef.current);
    }

    if (scanRef.current) {
      scene.remove(scanRef.current);
      disposeObject(scanRef.current);
      scanRef.current = null;
    }
    scanRef.current = buildScanPoints(
      state.scan,
      state.tf,
      state.tf_static,
      state.robot_pose,
    );
    if (scanRef.current) {
      scanRef.current.visible = layerVisibility.scan;
      scene.add(scanRef.current);
    }

    if (tfGroupRef.current) {
      scene.remove(tfGroupRef.current);
      disposeObject(tfGroupRef.current);
      tfGroupRef.current = null;
    }
    tfGroupRef.current = buildTfGroup(
      state.tf,
      state.tf_static,
      state.robot_pose,
      state.map,
    );
    if (tfGroupRef.current) {
      tfGroupRef.current.visible = layerVisibility.tf;
      scene.add(tfGroupRef.current);
    }

    if (state.map && cameraRef.current) {
      const controls = controlsRef.current;
      const mapSignature = [
        state.map.info.width,
        state.map.info.height,
        state.map.info.resolution,
        state.map.info.origin.position.x,
        state.map.info.origin.position.y,
        state.map.info.origin.orientation.yaw,
      ].join(":");
      if (lastCenteredMapSignatureRef.current === mapSignature) {
        renderRef.current?.();
        return;
      }
      const widthMeters = state.map.info.width * state.map.info.resolution;
      const heightMeters = state.map.info.height * state.map.info.resolution;
      const center = rotate2d(
        widthMeters / 2,
        heightMeters / 2,
        state.map.info.origin.orientation.yaw,
      );
      const centerX = state.map.info.origin.position.x + center.x;
      const centerY = state.map.info.origin.position.y + center.y;
      cameraRef.current.position.x = centerX;
      cameraRef.current.position.z = -(centerY + 0.001);
      if (controls) {
        controls.target.set(centerX, 0, -centerY);
        controls.update();
      } else {
        cameraRef.current.lookAt(centerX, 0, -centerY);
      }
      lastCenteredMapSignatureRef.current = mapSignature;
    }

    renderRef.current?.();
  }, [state, layerVisibility, goalMarker, goalLifecycle]);

  return <div className="scene-viewport" ref={viewportRef} />;
}
