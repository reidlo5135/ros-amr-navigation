import { useDeferredValue, useEffect, useRef } from "react";
import * as THREE from "three";

import type { BridgeState } from "../lib/protocol";

type SceneViewportProps = {
  state: BridgeState;
};

function buildPathLine(
  points: Array<{ x: number; y: number }>,
  color: string,
): THREE.Line | null {
  if (points.length < 2) {
    return null;
  }

  const geometry = new THREE.BufferGeometry().setFromPoints(
    points.map((point) => new THREE.Vector3(point.x, 0.05, point.y)),
  );
  const material = new THREE.LineBasicMaterial({ color, linewidth: 2 });
  return new THREE.Line(geometry, material);
}

export function SceneViewport({ state }: SceneViewportProps) {
  const canvasRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const robotRef = useRef<THREE.Group | null>(null);
  const globalPathRef = useRef<THREE.Line | null>(null);
  const localPathRef = useRef<THREE.Line | null>(null);
  const obstacleRef = useRef<THREE.Mesh | null>(null);
  const mapFrameRef = useRef<THREE.Mesh | null>(null);
  const renderRef = useRef<(() => void) | null>(null);
  const deferredState = useDeferredValue(state);

  useEffect(() => {
    if (!canvasRef.current) {
      return;
    }

    const scene = new THREE.Scene();
    scene.background = new THREE.Color("#07111c");
    scene.fog = new THREE.Fog("#07111c", 16, 48);

    const renderer = new THREE.WebGLRenderer({
      antialias: false,
      alpha: false,
      powerPreference: "low-power",
    });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.25));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.setSize(canvasRef.current.clientWidth, canvasRef.current.clientHeight);
    canvasRef.current.appendChild(renderer.domElement);

    const camera = new THREE.PerspectiveCamera(
      42,
      canvasRef.current.clientWidth / Math.max(canvasRef.current.clientHeight, 1),
      0.1,
      200,
    );
    camera.position.set(0, 14, 12);
    camera.lookAt(0, 0, 0);

    const ambientLight = new THREE.AmbientLight("#c4e6ff", 1.3);
    const keyLight = new THREE.DirectionalLight("#9fd3ff", 1.2);
    keyLight.position.set(10, 18, 8);
    scene.add(ambientLight, keyLight);

    const grid = new THREE.GridHelper(40, 40, "#2f7889", "#123646");
    scene.add(grid);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(44, 44),
      new THREE.MeshStandardMaterial({
        color: "#0a2130",
        transparent: true,
        opacity: 0.9,
        metalness: 0.15,
        roughness: 0.92,
      }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.01;
    scene.add(floor);

    const robot = new THREE.Group();
    const body = new THREE.Mesh(
      new THREE.CylinderGeometry(0.22, 0.22, 0.18, 24),
      new THREE.MeshStandardMaterial({
        color: "#f4ede0",
        metalness: 0.25,
        roughness: 0.45,
      }),
    );
    body.position.y = 0.1;
    const heading = new THREE.Mesh(
      new THREE.ConeGeometry(0.12, 0.35, 16),
      new THREE.MeshStandardMaterial({
        color: "#ff8b37",
        emissive: "#ff7a19",
        emissiveIntensity: 0.2,
      }),
    );
    heading.rotation.z = -Math.PI / 2;
    heading.position.set(0.24, 0.14, 0);
    robot.add(body, heading);
    scene.add(robot);

    const obstacle = new THREE.Mesh(
      new THREE.SphereGeometry(0.16, 20, 20),
      new THREE.MeshStandardMaterial({
        color: "#ff4a43",
        emissive: "#ff2b1f",
        emissiveIntensity: 0.35,
      }),
    );
    obstacle.visible = false;
    scene.add(obstacle);

    const mapFrame = new THREE.Mesh(
      new THREE.PlaneGeometry(1, 1),
      new THREE.MeshBasicMaterial({
        color: "#2bbad8",
        transparent: true,
        opacity: 0.08,
        side: THREE.DoubleSide,
      }),
    );
    mapFrame.rotation.x = -Math.PI / 2;
    scene.add(mapFrame);

    const renderScene = () => {
      renderer.render(scene, camera);
    };
    renderRef.current = renderScene;
    renderScene();

    const handleResize = () => {
      if (!canvasRef.current) {
        return;
      }
      camera.aspect = canvasRef.current.clientWidth / Math.max(canvasRef.current.clientHeight, 1);
      camera.updateProjectionMatrix();
      renderer.setSize(canvasRef.current.clientWidth, canvasRef.current.clientHeight);
      renderScene();
    };

    window.addEventListener("resize", handleResize);

    sceneRef.current = scene;
    rendererRef.current = renderer;
    cameraRef.current = camera;
    robotRef.current = robot;
    obstacleRef.current = obstacle;
    mapFrameRef.current = mapFrame;

    return () => {
      window.removeEventListener("resize", handleResize);
      renderRef.current = null;
      renderer.dispose();
      scene.clear();
      canvasRef.current?.removeChild(renderer.domElement);
    };
  }, []);

  useEffect(() => {
    const scene = sceneRef.current;
    const robot = robotRef.current;
    const obstacle = obstacleRef.current;
    const mapFrame = mapFrameRef.current;
    if (!scene || !robot || !obstacle || !mapFrame) {
      return;
    }

    const robotPose = deferredState.robot_pose;
    if (robotPose) {
      robot.position.set(robotPose.position.x, 0, robotPose.position.y);
      robot.rotation.y = -robotPose.orientation.yaw;
    }

    if (globalPathRef.current) {
      scene.remove(globalPathRef.current);
      globalPathRef.current.geometry.dispose();
      (globalPathRef.current.material as THREE.Material).dispose();
      globalPathRef.current = null;
    }
    if (localPathRef.current) {
      scene.remove(localPathRef.current);
      localPathRef.current.geometry.dispose();
      (localPathRef.current.material as THREE.Material).dispose();
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

    globalPathRef.current = buildPathLine(globalPath, "#48e0d2");
    localPathRef.current = buildPathLine(localPath, "#ff8f45");
    if (globalPathRef.current) {
      scene.add(globalPathRef.current);
    }
    if (localPathRef.current) {
      scene.add(localPathRef.current);
    }

    const obstacleReport = deferredState.obstacle_report;
    if (obstacleReport?.active) {
      obstacle.visible = true;
      obstacle.position.set(
        obstacleReport.obstacle_point.x,
        0.12,
        obstacleReport.obstacle_point.y,
      );
    } else {
      obstacle.visible = false;
    }

    const map = deferredState.map;
    if (map) {
      mapFrame.visible = true;
      mapFrame.scale.set(
        map.info.width * map.info.resolution,
        map.info.height * map.info.resolution,
        1,
      );
      mapFrame.position.set(
        map.info.origin.position.x + (map.info.width * map.info.resolution) * 0.5,
        0.001,
        map.info.origin.position.y + (map.info.height * map.info.resolution) * 0.5,
      );
      mapFrame.rotation.z = map.info.origin.orientation.yaw;
    } else {
      mapFrame.visible = false;
    }

    renderRef.current?.();
  }, [deferredState]);

  return <div className="scene-viewport" ref={canvasRef} />;
}
