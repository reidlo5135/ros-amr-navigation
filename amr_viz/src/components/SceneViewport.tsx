import { useDeferredValue, useEffect, useRef } from "react";
import * as THREE from "three";

import type {
  BridgeState,
  LaserScanMessage,
  OccupancyGridMessage,
  Pose,
  TfMessage,
} from "../lib/protocol";

type SceneViewportProps = {
  state: BridgeState;
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

function rotate2d(x: number, y: number, yaw: number) {
  const cosYaw = Math.cos(yaw);
  const sinYaw = Math.sin(yaw);
  return {
    x: (x * cosYaw) - (y * sinYaw),
    y: (x * sinYaw) + (y * cosYaw),
  };
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
): THREE.Line | null {
  if (points.length < 2) {
    return null;
  }

  const geometry = new THREE.BufferGeometry().setFromPoints(
    points.map((point) => new THREE.Vector3(point.x, yOffset, point.y)),
  );
  const material = new THREE.LineBasicMaterial({ color });
  return new THREE.Line(geometry, material);
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
  const gradient = context.createRadialGradient(16, 16, 2, 16, 16, 14);
  gradient.addColorStop(0, "rgba(88,255,120,1)");
  gradient.addColorStop(0.55, "rgba(88,255,120,0.92)");
  gradient.addColorStop(1, "rgba(88,255,120,0)");
  context.fillStyle = gradient;
  context.beginPath();
  context.arc(16, 16, 14, 0, Math.PI * 2);
  context.fill();

  const texture = new THREE.CanvasTexture(canvas);
  texture.needsUpdate = true;
  return texture;
}

function buildOccupancyTexture(
  grid: OccupancyGridMessage,
  palette: "map" | "global_costmap" | "local_costmap",
): THREE.DataTexture {
  const { width, height } = grid.info;
  const rgba = new Uint8Array(width * height * 4);

  for (let row = 0; row < height; row += 1) {
    for (let col = 0; col < width; col += 1) {
      const sourceIndex = row * width + col;
      const targetIndex = ((row * width) + col) * 4;
      const value = grid.data[sourceIndex] ?? -1;
      let red = 0;
      let green = 0;
      let blue = 0;
      let alpha = 0;

      if (palette === "map") {
        if (value < 0) {
          red = 206;
          green = 206;
          blue = 206;
          alpha = 255;
        } else {
          const shade = Math.max(18, Math.min(255, 255 - Math.round((value / 100) * 245)));
          red = shade;
          green = shade;
          blue = shade;
          alpha = 255;
        }
      } else {
        if (value > 0) {
          const normalized = Math.max(0, Math.min(1, value / 100));
          alpha = Math.max(18, Math.round((palette === "global_costmap" ? 88 : 132) * normalized));
          if (palette === "global_costmap") {
            red = Math.round(222 + (24 * normalized));
            green = Math.round(192 + (18 * normalized));
            blue = Math.round(232 + (12 * normalized));
          } else {
            red = Math.round(56 + (36 * normalized));
            green = Math.round(58 + (22 * normalized));
            blue = Math.round(202 + (42 * normalized));
          }
        }
      }

      rgba[targetIndex] = red;
      rgba[targetIndex + 1] = green;
      rgba[targetIndex + 2] = blue;
      rgba[targetIndex + 3] = alpha;
    }
  }

  const texture = new THREE.DataTexture(rgba, width, height, THREE.RGBAFormat);
  texture.needsUpdate = true;
  texture.magFilter = THREE.NearestFilter;
  texture.minFilter = THREE.NearestFilter;
  texture.generateMipmaps = false;
  texture.flipY = false;
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
  });
  const mesh = new THREE.Mesh(geometry, material);
  const originYaw = grid.info.origin.orientation.yaw;
  const rotatedOffset = rotate2d(widthMeters / 2, heightMeters / 2, originYaw);
  const yawQuaternion = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 1, 0),
    -originYaw,
  );
  const planeQuaternion = new THREE.Quaternion().setFromEuler(new THREE.Euler(Math.PI / 2, 0, 0));

  mesh.quaternion.multiplyQuaternions(yawQuaternion, planeQuaternion);
  mesh.position.set(
    grid.info.origin.position.x + rotatedOffset.x,
    yOffset,
    grid.info.origin.position.y + rotatedOffset.y,
  );
  mesh.renderOrder = palette === "map" ? 0 : palette === "global_costmap" ? 1 : 2;
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

  const edge = lookup.get(frameId);
  if (!edge) {
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
    points.push(pose.x + rotated.x, 0.08, pose.y + rotated.y);
  }

  if (points.length === 0) {
    return null;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(points, 3));
  const material = new THREE.PointsMaterial({
    color: "#5cff75",
    size: 0.11,
    map: createScanSpriteTexture(),
    transparent: true,
    opacity: 0.9,
    sizeAttenuation: true,
    depthTest: false,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    alphaTest: 0.08,
  });
  const cloud = new THREE.Points(geometry, material);
  cloud.renderOrder = 12;
  return cloud;
}

function buildTfGroup(
  tf: TfMessage | undefined,
  tfStatic: TfMessage | undefined,
  robotPose: Pose | undefined,
): THREE.Group | null {
  const lookup = buildFrameLookup(tf, tfStatic);
  const transforms = [...(tfStatic?.transforms ?? []), ...(tf?.transforms ?? [])];
  if (transforms.length === 0) {
    return null;
  }

  const group = new THREE.Group();
  const cache = new Map<string, ResolvedFrame | null>();

  for (const transform of transforms) {
    const child = resolveFrame(transform.child_frame_id, lookup, robotPose, cache);
    const parent = resolveFrame(transform.header.frame_id, lookup, robotPose, cache);
    if (!child) {
      continue;
    }

    const marker = new THREE.Mesh(
      new THREE.CylinderGeometry(0.035, 0.035, 0.18, 12),
      new THREE.MeshStandardMaterial({
        color: tfStatic?.transforms.includes(transform) ? "#5a8cff" : "#93ff8b",
        emissive: tfStatic?.transforms.includes(transform) ? "#2b4b99" : "#2f8d26",
        emissiveIntensity: 0.18,
      }),
    );
    marker.position.set(child.x, 0.12, child.y);
    group.add(marker);

    if (parent) {
      const lineGeometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(parent.x, 0.06, parent.y),
        new THREE.Vector3(child.x, 0.06, child.y),
      ]);
      const line = new THREE.Line(
        lineGeometry,
        new THREE.LineBasicMaterial({
          color: tfStatic?.transforms.includes(transform) ? "#4d77d6" : "#79ff9f",
          transparent: true,
          opacity: 0.65,
        }),
      );
      group.add(line);
    }
  }

  return group.children.length > 0 ? group : null;
}

export function SceneViewport({ state }: SceneViewportProps) {
  const viewportRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const robotRef = useRef<THREE.Group | null>(null);
  const globalPathRef = useRef<THREE.Line | null>(null);
  const localPathRef = useRef<THREE.Line | null>(null);
  const obstacleRef = useRef<THREE.Mesh | null>(null);
  const mapMeshRef = useRef<THREE.Mesh | null>(null);
  const globalCostmapMeshRef = useRef<THREE.Mesh | null>(null);
  const localCostmapMeshRef = useRef<THREE.Mesh | null>(null);
  const scanRef = useRef<THREE.Points | null>(null);
  const tfGroupRef = useRef<THREE.Group | null>(null);
  const renderRef = useRef<(() => void) | null>(null);
  const deferredState = useDeferredValue(state);

  useEffect(() => {
    if (!viewportRef.current) {
      return;
    }

    const scene = new THREE.Scene();
    scene.background = new THREE.Color("#303030");

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
    camera.position.set(0, 28, 0.001);
    camera.lookAt(0, 0, 0);

    const ambientLight = new THREE.AmbientLight("#ffffff", 1.18);
    const keyLight = new THREE.DirectionalLight("#d8e6ff", 0.84);
    keyLight.position.set(12, 22, 10);
    scene.add(ambientLight, keyLight);

    const grid = new THREE.GridHelper(60, 60, "#7b7b7b", "#a8a8a8");
    grid.position.y = 0.045;
    grid.renderOrder = 10;
    grid.material.depthTest = false;
    grid.material.transparent = true;
    grid.material.opacity = 0.45;
    scene.add(grid);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(64, 64),
      new THREE.MeshStandardMaterial({
        color: "#343434",
        transparent: true,
        opacity: 1,
        metalness: 0.02,
        roughness: 1,
      }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.015;
    scene.add(floor);

    const robot = new THREE.Group();
    const footprint = new THREE.Mesh(
      new THREE.CircleGeometry(0.082, 32),
      new THREE.MeshBasicMaterial({
        color: "#d4d7da",
        transparent: true,
        opacity: 0.92,
      }),
    );
    footprint.rotation.x = -Math.PI / 2;
    footprint.position.y = 0.032;
    const centerDot = new THREE.Mesh(
      new THREE.CircleGeometry(0.018, 20),
      new THREE.MeshBasicMaterial({ color: "#3b3f45" }),
    );
    centerDot.rotation.x = -Math.PI / 2;
    centerDot.position.y = 0.036;
    const heading = new THREE.Mesh(
      new THREE.ConeGeometry(0.032, 0.12, 3),
      new THREE.MeshBasicMaterial({ color: "#ff8b37" }),
    );
    heading.rotation.x = Math.PI / 2;
    heading.rotation.z = -Math.PI / 2;
    heading.position.set(0.098, 0.04, 0);
    robot.add(footprint, centerDot, heading);
    scene.add(robot);

    const obstacle = new THREE.Group();
    const obstacleRing = new THREE.Mesh(
      new THREE.RingGeometry(0.11, 0.155, 28),
      new THREE.MeshBasicMaterial({
        color: "#ff4a43",
        transparent: true,
        opacity: 0.88,
        side: THREE.DoubleSide,
      }),
    );
    obstacleRing.rotation.x = -Math.PI / 2;
    obstacleRing.position.y = 0.03;
    const obstacleCore = new THREE.Mesh(
      new THREE.CircleGeometry(0.05, 24),
      new THREE.MeshBasicMaterial({
        color: "#ff4a43",
        transparent: true,
        opacity: 0.2,
      }),
    );
    obstacleCore.rotation.x = -Math.PI / 2;
    obstacleCore.position.y = 0.028;
    obstacle.add(obstacleRing, obstacleCore);
    obstacle.visible = false;
    scene.add(obstacle);

    const renderScene = () => {
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

    const handleWheel = (event: WheelEvent) => {
      event.preventDefault();
      const zoomDelta = event.deltaY > 0 ? -0.14 : 0.14;
      camera.zoom = Math.max(0.35, Math.min(3.2, camera.zoom + zoomDelta));
      camera.updateProjectionMatrix();
      renderScene();
    };

    window.addEventListener("resize", handleResize);
    renderer.domElement.addEventListener("wheel", handleWheel, { passive: false });

    sceneRef.current = scene;
    rendererRef.current = renderer;
    cameraRef.current = camera;
    robotRef.current = robot;
    obstacleRef.current = obstacle;

    return () => {
      window.removeEventListener("resize", handleResize);
      renderer.domElement.removeEventListener("wheel", handleWheel);
      renderRef.current = null;
      disposeObject(globalPathRef.current);
      disposeObject(localPathRef.current);
      disposeObject(mapMeshRef.current);
      disposeObject(globalCostmapMeshRef.current);
      disposeObject(localCostmapMeshRef.current);
      disposeObject(scanRef.current);
      disposeObject(tfGroupRef.current);
      disposeObject(obstacle);
      disposeObject(robot);
      renderer.dispose();
      scene.clear();
      viewportRef.current?.removeChild(renderer.domElement);
    };
  }, []);

  useEffect(() => {
    const scene = sceneRef.current;
    const robot = robotRef.current;
    const obstacle = obstacleRef.current;
    if (!scene || !robot || !obstacle) {
      return;
    }

    const robotPose = deferredState.robot_pose;
    if (robotPose) {
      robot.position.set(robotPose.position.x, 0, robotPose.position.y);
      robot.rotation.y = -robotPose.orientation.yaw;
    }

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

    const globalPath = deferredState.global_path?.poses.map((pose) => ({
      x: pose.position.x,
      y: pose.position.y,
    })) ?? [];
    const localPath = deferredState.local_path?.poses.map((pose) => ({
      x: pose.position.x,
      y: pose.position.y,
    })) ?? [];

    globalPathRef.current = buildPathLine(globalPath, "#4ce3d4", 0.06);
    localPathRef.current = buildPathLine(localPath, "#ff9850", 0.08);
    if (globalPathRef.current) {
      scene.add(globalPathRef.current);
    }
    if (localPathRef.current) {
      scene.add(localPathRef.current);
    }

    for (const [ref, grid, palette, yOffset] of [
      [mapMeshRef, deferredState.map, "map", 0.005],
      [globalCostmapMeshRef, deferredState.global_costmap, "global_costmap", 0.02],
      [localCostmapMeshRef, deferredState.local_costmap, "local_costmap", 0.03],
    ] as const) {
      if (ref.current) {
        scene.remove(ref.current);
        disposeObject(ref.current);
        ref.current = null;
      }
      if (grid) {
        ref.current = buildOccupancyMesh(grid, palette, yOffset);
        scene.add(ref.current);
      }
    }

    if (scanRef.current) {
      scene.remove(scanRef.current);
      disposeObject(scanRef.current);
      scanRef.current = null;
    }
    scanRef.current = buildScanPoints(
      deferredState.scan,
      deferredState.tf,
      deferredState.tf_static,
      deferredState.robot_pose,
    );
    if (scanRef.current) {
      scene.add(scanRef.current);
    }

    if (tfGroupRef.current) {
      scene.remove(tfGroupRef.current);
      disposeObject(tfGroupRef.current);
      tfGroupRef.current = null;
    }
    tfGroupRef.current = buildTfGroup(
      deferredState.tf,
      deferredState.tf_static,
      deferredState.robot_pose,
    );
    if (tfGroupRef.current) {
      scene.add(tfGroupRef.current);
    }

    const obstacleReport = deferredState.obstacle_report;
    if (obstacleReport?.active) {
      obstacle.visible = true;
      obstacle.position.set(
        obstacleReport.obstacle_point.x,
        0.12,
        obstacleReport.obstacle_point.y,
      );
      obstacle.scale.setScalar(1 + (obstacleReport.severity * 0.12));
    } else {
      obstacle.visible = false;
    }

    if (deferredState.map && cameraRef.current) {
      const widthMeters = deferredState.map.info.width * deferredState.map.info.resolution;
      const heightMeters = deferredState.map.info.height * deferredState.map.info.resolution;
      const center = rotate2d(
        widthMeters / 2,
        heightMeters / 2,
        deferredState.map.info.origin.orientation.yaw,
      );
      const centerX = deferredState.map.info.origin.position.x + center.x;
      const centerY = deferredState.map.info.origin.position.y + center.y;
      cameraRef.current.position.x = centerX;
      cameraRef.current.position.z = centerY + 0.001;
      cameraRef.current.lookAt(centerX, 0, centerY);
    }

    renderRef.current?.();
  }, [deferredState]);

  return <div className="scene-viewport" ref={viewportRef} />;
}
