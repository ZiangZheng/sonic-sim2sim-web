import './style.css';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import loadMujoco from '@mujoco/mujoco';
import wasmUrl from '@mujoco/mujoco/mujoco.wasm?url';

import { loadMuJoCoScene, updateSceneTransforms, getBodyWorldTransform, type MuJoCoScene } from './mujocoScene';
import { loadMotionFromURL, loadMotionFromFile, sampleMotion, type MotionClip } from './motion';
import { applyPDControl, setStateKinematic, DEFAULT_CONTROLLER_OPTIONS, type ControllerOptions } from './controller';
import { CameraWindow } from './cameras';
import { RealTimePlot } from './plots';
import { PolicyController } from './phpPolicyController.js';

const MUJOCO_SCENE_PATH = '/working/scene.xml';
const DEFAULT_MOTION_URL = './motions/stand_sonic.json';

type ModelId = 'sonic' | 'php';
type PlayMode = 'kinematic' | 'sonic';

interface ModelProfile {
  id: ModelId;
  label: string;
  sceneUrl: string;
  includeRobotXml: boolean;
  defaultMotionUrl?: string;
  initialJointPos?: number[];
  policy?: {
    modelPath: string;
    depthModelPath: string | null;
    controlDt: number;
  };
}

const PHP_DEFAULT_JOINT_POS = [
  0.162997201, -0.0361181423, -0.0214254409, 0.267154634, -0.174296871, 0.212671682,
  0.282425106, -0.0584460497, -0.556104779, 0.126711249, -0.123827517, -0.190653816,
  0.000492588617, -0.0195334535, 0.428676069,
  -0.00628881808, 0.161155701, 0.236345276, 0.980316162, 0.15456377, 0.0774896815, 0.0205286704,
  -0.128641531, -0.0847690701, -0.255017966, 1.09530210, -0.134532213, 0.0875737667, 0.0601755157,
];

const MODEL_PROFILES: Record<ModelId, ModelProfile> = {
  sonic: {
    id: 'sonic',
    label: 'Sonic',
    sceneUrl: './assets/g1/scene.xml',
    includeRobotXml: true,
    defaultMotionUrl: DEFAULT_MOTION_URL,
  },
  php: {
    id: 'php',
    label: 'PHP',
    sceneUrl: './assets/php/g1_with_terrain.xml',
    includeRobotXml: false,
    initialJointPos: PHP_DEFAULT_JOINT_POS,
    policy: {
      modelPath: './policies/php/student.onnx',
      depthModelPath: null,
      controlDt: 0.02,
    },
  },
};

function getInitialModelId(): ModelId {
  const model = new URLSearchParams(window.location.search).get('model')?.toLowerCase();
  return model === 'php' ? 'php' : 'sonic';
}

async function setupVFS(mujoco: any, profile: ModelProfile) {
  mujoco.FS.mkdir('/working');

  const sceneText = await (await fetch(profile.sceneUrl)).text();
  mujoco.FS.writeFile(MUJOCO_SCENE_PATH, sceneText);

  const xmlSources = [sceneText];
  if (profile.includeRobotXml) {
    // The Sonic scene includes the robot XML in the same directory.
    const robotUrl = './assets/g1/g1_29dof_rev_1_0.xml';
    let robotText = await (await fetch(robotUrl)).text();
    // The original XML lives next to a sibling meshes/ directory. In the VFS
    // we place the XML and meshes both under /working/, so adjust meshdir.
    robotText = robotText.replace(/meshdir="\.\.\/meshes\/g1\/"/g, 'meshdir="meshes/g1/"');
    mujoco.FS.writeFile('/working/g1_29dof_rev_1_0.xml', robotText);
    xmlSources.push(robotText);
  }

  // Parse mesh file names from the robot XML.
  const parser = new DOMParser();
  const meshFiles = Array.from(new Set(xmlSources.flatMap((xml) => {
    const xmlDoc = parser.parseFromString(xml, 'text/xml');
    return Array.from(xmlDoc.querySelectorAll('mesh'))
      .map((el) => el.getAttribute('file'))
      .filter(Boolean) as string[];
  })));

  const meshDir = '/working/meshes/g1';
  mujoco.FS.mkdir('/working/meshes');
  mujoco.FS.mkdir(meshDir);

  await Promise.all(
    meshFiles.map(async (file) => {
      const url = `./assets/g1/meshes/g1/${file}`;
      const res = await fetch(url);
      if (!res.ok) {
        console.warn(`Failed to fetch mesh ${file}`);
        return;
      }
      const buffer = await res.arrayBuffer();
      mujoco.FS.writeFile(`${meshDir}/${file}`, new Uint8Array(buffer));
    }),
  );
}

async function init() {
  const app = document.getElementById('app')!;
  const activeModelId = getInitialModelId();
  const activeProfile = MODEL_PROFILES[activeModelId];

  // Canvas container
  const canvasContainer = document.createElement('div');
  canvasContainer.id = 'canvas-container';
  app.appendChild(canvasContainer);

  // Load MuJoCo WASM.
  const mujoco = await loadMujoco({ locateFile: (path: string) => (path === 'mujoco.wasm' ? wasmUrl : path) });
  await setupVFS(mujoco, activeProfile);

  const mjScene = await loadMuJoCoScene(mujoco, MUJOCO_SCENE_PATH);

  // Three.js renderer and scene.
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  canvasContainer.appendChild(renderer.domElement);

  mjScene.root.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      (obj as THREE.Mesh).frustumCulled = false;
    }
  });

  // Camera.
  const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 200);
  camera.position.set(2.5, 1.8, 2.5);

  // Controls.
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.8, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;

  // Lights.
  const ambient = new THREE.AmbientLight(0xffffff, 0.6);
  mjScene.root.add(ambient);

  const sun = new THREE.DirectionalLight(0xfff5e1, 2.5);
  sun.position.set(5, 8, 4);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.near = 0.5;
  sun.shadow.camera.far = 50;
  sun.shadow.camera.left = -10;
  sun.shadow.camera.right = 10;
  sun.shadow.camera.top = 10;
  sun.shadow.camera.bottom = -10;
  mjScene.root.add(sun);

  const fill = new THREE.HemisphereLight(0x87ceeb, 0x5d4c3a, 0.4);
  mjScene.root.add(fill);

  // Sky gradient.
  const skyCanvas = document.createElement('canvas');
  skyCanvas.width = 512;
  skyCanvas.height = 512;
  const ctx = skyCanvas.getContext('2d')!;
  const grad = ctx.createLinearGradient(0, skyCanvas.height, 0, 0);
  grad.addColorStop(0.0, '#e8dcc8');
  grad.addColorStop(0.35, '#a7c4d9');
  grad.addColorStop(1.0, '#6d9cc8');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, 512, 512);
  const skyTexture = new THREE.CanvasTexture(skyCanvas);
  skyTexture.colorSpace = THREE.SRGBColorSpace;

  // Add root to a separate scene so background applies only to main camera.
  const scene = new THREE.Scene();
  scene.background = skyTexture;
  scene.fog = new THREE.Fog(new THREE.Color(0xa7c4d9), 20, 90);
  scene.add(mjScene.root);

  // State.
  let motion: MotionClip | null = null;
  let isPlaying = activeModelId === 'sonic';
  let playMode: PlayMode = 'kinematic';
  let playSpeed = 1.0;
  let currentTime = 0;
  let simTime = 0;
  let lastFrameTime = performance.now();
  let controllerOptions: ControllerOptions = { ...DEFAULT_CONTROLLER_OPTIONS };
  let phpPolicy: any = null;
  let phpPolicyReady = false;
  let phpPolicyEnabled = activeModelId === 'php';
  let phpPolicyStepCounter = 0;
  let phpPolicyDecimation = 1;

  // Build UI.
  const ui = buildUI({
    activeModelId,
    initialPlaying: isPlaying,
    onPlayPause: () => {
      isPlaying = !isPlaying;
    },
    onModelChange: (modelId) => {
      const params = new URLSearchParams(window.location.search);
      params.set('model', modelId);
      window.location.search = params.toString();
    },
    onReset: () => {
      currentTime = 0;
      simTime = 0;
      phpPolicyStepCounter = 0;
      if (motion) {
        const ref = sampleMotion(motion, 0);
        setStateKinematic(mjScene.model, mjScene.data, ref.qpos);
        mujoco.mj_forward(mjScene.model, mjScene.data);
        updateSceneTransforms(mjScene.model, mjScene.data, mjScene.bodies);
      } else {
        resetModelState(mjScene, mujoco, activeProfile);
        phpPolicy?.reset?.();
      }
    },
    onModeChange: (mode) => {
      playMode = mode;
      if (motion) {
        const ref = sampleMotion(motion, currentTime);
        setStateKinematic(mjScene.model, mjScene.data, ref.qpos);
        mujoco.mj_forward(mjScene.model, mjScene.data);
        updateSceneTransforms(mjScene.model, mjScene.data, mjScene.bodies);
      }
    },
    onSpeedChange: (s) => (playSpeed = s),
    onFile: async (file) => {
      motion = await loadMotionFromFile(file);
      currentTime = 0;
      simTime = 0;
      updateMotionUI(motion);
    },
    onKpChange: (kp) => (controllerOptions.kp = kp),
    onKdChange: (kd) => (controllerOptions.kd = kd),
    onPhpPolicyChange: (enabled) => (phpPolicyEnabled = enabled),
    onPhpAutoForwardChange: (enabled) => {
      if (phpPolicy) phpPolicy.autoForward = enabled;
    },
    onPhpHighSpeedChange: (enabled) => {
      if (phpPolicy) phpPolicy.highSpeedMode = enabled;
    },
    onSeek: (t) => {
      currentTime = t;
      simTime = t;
      if (motion) {
        const ref = sampleMotion(motion, currentTime);
        setStateKinematic(mjScene.model, mjScene.data, ref.qpos);
        mujoco.mj_forward(mjScene.model, mjScene.data);
        updateSceneTransforms(mjScene.model, mjScene.data, mjScene.bodies);
      }
    },
  });
  app.appendChild(ui.root);

  // Camera windows.
  const rgbWindow = new CameraWindow(ui.rgbContainer, false);
  const depthWindow = new CameraWindow(ui.depthContainer, true);

  if (activeProfile.policy) {
    ui.policyStatusEl!.textContent = 'Loading policy...';
    phpPolicy = new PolicyController(mujoco, activeProfile.policy);
    phpPolicy.init(mjScene.model).then(() => {
      phpPolicyReady = true;
      phpPolicyDecimation = Math.max(1, Math.round(activeProfile.policy!.controlDt / mjScene.model.opt.timestep));
      ui.policyStatusEl!.textContent = 'Policy ready';
    }).catch((err: unknown) => {
      console.error('Failed to initialize PHP policy:', err);
      ui.policyStatusEl!.textContent = 'Policy failed';
    });
  }

  // Plots.
  const jointErrorPlot = ui.jointErrorCanvas ? new RealTimePlot(
    ui.jointErrorCanvas,
    'Joint Tracking Error (rad)',
    ['left_hip', 'right_hip', 'left_knee', 'right_knee'],
    ['#3870e8', '#00bcd4', '#f59e0b', '#ef4444'],
  ) : null;
  const rootHeightPlot = ui.rootHeightCanvas ? new RealTimePlot(
    ui.rootHeightCanvas,
    'Root Height (m)',
    ['ref', 'actual'],
    ['#00bcd4', '#3870e8'],
  ) : null;

  if (activeProfile.defaultMotionUrl) {
    loadMotionFromURL(activeProfile.defaultMotionUrl).then((m) => {
      motion = m;
      updateMotionUI(motion);
      const ref = sampleMotion(motion, 0);
      setStateKinematic(mjScene.model, mjScene.data, ref.qpos);
      mujoco.mj_forward(mjScene.model, mjScene.data);
      updateSceneTransforms(mjScene.model, mjScene.data, mjScene.bodies);
    });
  } else {
    resetModelState(mjScene, mujoco, activeProfile);
  }

  function updateMotionUI(m: MotionClip) {
    ui.durationEl.textContent = `${m.duration.toFixed(2)}s`;
    ui.timeSlider.max = String(m.duration);
    ui.timeSlider.step = String(m.duration / 1000);
  }

  function updateHeadCamera() {
    const headTransform = getBodyWorldTransform(mjScene.model, mjScene.data, mjScene.headBodyId);
    rgbWindow.updateCamera(headTransform.position, headTransform.quaternion);
    depthWindow.updateCamera(headTransform.position, headTransform.quaternion);
  }

  let plotAccumulator = 0;

  function animate() {
    requestAnimationFrame(animate);

    const now = performance.now();
    const dt = Math.min((now - lastFrameTime) / 1000, 0.1);
    lastFrameTime = now;

    if (motion && isPlaying) {
      if (playMode === 'kinematic') {
        currentTime += dt * playSpeed;
        if (currentTime > motion.duration) currentTime = 0;
        const ref = sampleMotion(motion, currentTime);
        setStateKinematic(mjScene.model, mjScene.data, ref.qpos);
        mujoco.mj_forward(mjScene.model, mjScene.data);
      } else {
        // Sonic: track the motion reference through the physics model.
        const step = mjScene.model.opt.timestep;
        const targetSimTime = simTime + dt * playSpeed;
        let steps = 0;
        while (simTime < targetSimTime && steps < 20) {
          const ref = sampleMotion(motion, simTime);
          applyPDControl(mujoco, mjScene.model, mjScene.data, ref.qpos, ref.qvel, controllerOptions);
          mujoco.mj_step(mjScene.model, mjScene.data);
          simTime += step;
          steps++;
        }
        currentTime = simTime;
        if (currentTime > motion.duration) {
          currentTime = 0;
          simTime = 0;
        }
      }

      ui.timeSlider.value = String(currentTime);
      ui.timeDisplay.textContent = `${currentTime.toFixed(2)}s`;
    } else if (!motion && isPlaying) {
      const step = mjScene.model.opt.timestep;
      const targetSimTime = simTime + dt * playSpeed;
      let steps = 0;
      while (simTime < targetSimTime && steps < 20) {
        if (activeProfile.policy) {
          if (!phpPolicyReady || !phpPolicyEnabled) break;
          if (phpPolicyStepCounter % phpPolicyDecimation === 0) {
            phpPolicy._updateCommandState?.();
            void phpPolicy.requestAction(mjScene.model, mjScene.data).catch((err: unknown) => {
              console.error('PHP policy inference error:', err);
            });
          }
          phpPolicy.applyControl(mjScene.model, mjScene.data);
          phpPolicyStepCounter++;
        }
        mujoco.mj_step(mjScene.model, mjScene.data);
        simTime += step;
        steps++;
      }
    }

    updateSceneTransforms(mjScene.model, mjScene.data, mjScene.bodies);
    updateHeadCamera();

    // Update plots every ~100ms.
    plotAccumulator += dt;
    if (plotAccumulator > 0.1 && motion) {
      plotAccumulator = 0;
      const ref = sampleMotion(motion, currentTime);
      const qpos = mjScene.data.qpos;
      const errors = [
        Math.abs(ref.qpos[7 + 0] - qpos[7 + 0]),
        Math.abs(ref.qpos[7 + 6] - qpos[7 + 6]),
        Math.abs(ref.qpos[7 + 3] - qpos[7 + 3]),
        Math.abs(ref.qpos[7 + 9] - qpos[7 + 9]),
      ];
      jointErrorPlot?.push(errors, currentTime.toFixed(1));
      rootHeightPlot?.push([ref.qpos[2], qpos[2]], currentTime.toFixed(1));
    }

    controls.update();

    renderer.render(scene, camera);
    rgbWindow.render(scene);
    depthWindow.render(scene);
  }

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  animate();
}

interface UIControls {
  activeModelId: ModelId;
  initialPlaying: boolean;
  onPlayPause: () => void;
  onReset: () => void;
  onModelChange: (modelId: ModelId) => void;
  onModeChange: (mode: PlayMode) => void;
  onSpeedChange: (speed: number) => void;
  onFile: (file: File) => void;
  onKpChange: (kp: number) => void;
  onKdChange: (kd: number) => void;
  onPhpPolicyChange: (enabled: boolean) => void;
  onPhpAutoForwardChange: (enabled: boolean) => void;
  onPhpHighSpeedChange: (enabled: boolean) => void;
  onSeek: (time: number) => void;
}

function buildUI(c: UIControls) {
  const isSonic = c.activeModelId === 'sonic';
  const root = document.createElement('div');
  root.className = 'pointer-events-none absolute inset-0 z-10 flex flex-col justify-between p-4';

  // Header.
  const header = document.createElement('div');
  header.className = 'pointer-events-auto glass-panel px-5 py-3 inline-flex items-center gap-3 self-start';
  header.innerHTML = `
    <div class="text-lg font-bold tracking-tight"><span class="text-cyan-400">${MODEL_PROFILES[c.activeModelId].label}</span> Visualizer</div>
    <div class="text-xs text-slate-400 hidden sm:block">Unitree G1 · MuJoCo WASM</div>
  `;
  root.appendChild(header);

  // Bottom bar.
  const bottom = document.createElement('div');
  bottom.className = 'pointer-events-auto flex flex-wrap items-end gap-4';

  // Playback panel.
  const playbackPanel = document.createElement('div');
  playbackPanel.className = 'glass-panel p-4 flex flex-col gap-3 min-w-[280px] max-w-[360px]';

  const row1 = document.createElement('div');
  row1.className = 'flex items-center gap-2';
  const playBtn = document.createElement('button');
  playBtn.className = c.initialPlaying ? 'glass-button active' : 'glass-button';
  playBtn.textContent = c.initialPlaying ? 'Pause' : 'Play';
  playBtn.onclick = () => {
    c.onPlayPause();
    playBtn.textContent = playBtn.textContent === 'Pause' ? 'Play' : 'Pause';
    playBtn.classList.toggle('active');
  };
  const resetBtn = document.createElement('button');
  resetBtn.className = 'glass-button';
  resetBtn.textContent = 'Reset';
  resetBtn.onclick = c.onReset;
  row1.append(playBtn, resetBtn);

  const modelRow = document.createElement('div');
  modelRow.className = 'flex items-center gap-2 text-sm';
  modelRow.innerHTML = '<span class="text-slate-300 w-14">Model</span>';
  const modelSelect = document.createElement('select');
  modelSelect.className = 'glass-input flex-1';
  modelSelect.innerHTML = `
    <option value="sonic">Sonic</option>
    <option value="php">PHP</option>
  `;
  modelSelect.value = c.activeModelId;
  modelSelect.onchange = () => c.onModelChange(modelSelect.value as ModelId);
  modelRow.append(modelSelect);

  const modeRow = document.createElement('div');
  modeRow.className = 'flex gap-2';
  const kinBtn = document.createElement('button');
  kinBtn.className = 'glass-button active flex-1';
  kinBtn.textContent = 'Kinematic';
  const simBtn = document.createElement('button');
  simBtn.className = 'glass-button flex-1';
  simBtn.textContent = 'Sonic';
  kinBtn.onclick = () => {
    c.onModeChange('kinematic');
    kinBtn.classList.add('active');
    simBtn.classList.remove('active');
  };
  simBtn.onclick = () => {
    c.onModeChange('sonic');
    simBtn.classList.add('active');
    kinBtn.classList.remove('active');
  };
  modeRow.append(kinBtn, simBtn);

  const speedRow = document.createElement('div');
  speedRow.className = 'flex items-center gap-2 text-sm';
  speedRow.innerHTML = '<span class="text-slate-300 w-14">Speed</span>';
  const speedInput = document.createElement('input');
  speedInput.type = 'range';
  speedInput.min = '0.1';
  speedInput.max = '2.0';
  speedInput.step = '0.1';
  speedInput.value = '1.0';
  speedInput.className = 'glass-range flex-1';
  const speedVal = document.createElement('span');
  speedVal.className = 'text-slate-300 w-8 text-right';
  speedVal.textContent = '1.0x';
  speedInput.oninput = () => {
    const v = parseFloat(speedInput.value);
    speedVal.textContent = v.toFixed(1) + 'x';
    c.onSpeedChange(v);
  };
  speedRow.append(speedInput, speedVal);

  const timeRow = document.createElement('div');
  timeRow.className = 'flex items-center gap-2 text-sm';
  const timeDisplay = document.createElement('span');
  timeDisplay.className = 'text-cyan-300 font-mono w-14';
  timeDisplay.textContent = '0.00s';
  const timeSlider = document.createElement('input');
  timeSlider.type = 'range';
  timeSlider.min = '0';
  timeSlider.max = '1';
  timeSlider.step = '0.01';
  timeSlider.value = '0';
  timeSlider.className = 'glass-range flex-1';
  timeSlider.oninput = () => {
    c.onSeek(parseFloat(timeSlider.value));
  };
  const durationEl = document.createElement('span');
  durationEl.className = 'text-slate-400 w-14 text-right';
  durationEl.textContent = '0.00s';
  timeRow.append(timeDisplay, timeSlider, durationEl);

  const fileRow = document.createElement('div');
  fileRow.className = 'flex items-center gap-2';
  const fileInput = document.createElement('input');
  fileInput.type = 'file';
  fileInput.accept = '.json';
  fileInput.className = 'hidden';
  fileInput.onchange = () => {
    if (fileInput.files?.length) c.onFile(fileInput.files[0]);
  };
  const uploadBtn = document.createElement('button');
  uploadBtn.className = 'glass-button w-full';
  uploadBtn.textContent = 'Upload motion JSON';
  uploadBtn.onclick = () => fileInput.click();
  fileRow.append(fileInput, uploadBtn);

  playbackPanel.append(row1, modelRow);
  if (isSonic) playbackPanel.append(modeRow);
  playbackPanel.append(speedRow);
  if (isSonic) playbackPanel.append(timeRow, fileRow);

  // Options panel.
  const optionsPanel = document.createElement('div');
  optionsPanel.className = 'glass-panel p-4 flex flex-col gap-3 min-w-[220px]';
  optionsPanel.innerHTML = `
    <div class="text-sm font-semibold text-slate-200">Sonic PD Gains</div>
  `;
  const kpRow = createNumberRow('Kp', DEFAULT_CONTROLLER_OPTIONS.kp, 0, 400, 10, c.onKpChange);
  const kdRow = createNumberRow('Kd', DEFAULT_CONTROLLER_OPTIONS.kd, 0, 40, 1, c.onKdChange);
  optionsPanel.append(kpRow, kdRow);

  const phpOptionsPanel = document.createElement('div');
  phpOptionsPanel.className = 'glass-panel p-4 flex flex-col gap-3 min-w-[220px]';
  phpOptionsPanel.innerHTML = `
    <div class="text-sm font-semibold text-slate-200">PHP Policy</div>
    <div class="text-xs text-slate-400" id="policy-status">Policy disabled</div>
  `;
  const policyRow = createCheckboxRow('Enabled', true, c.onPhpPolicyChange);
  const autoForwardRow = createCheckboxRow('Auto forward', false, c.onPhpAutoForwardChange);
  const highSpeedRow = createCheckboxRow('High speed', true, c.onPhpHighSpeedChange);
  phpOptionsPanel.append(policyRow, autoForwardRow, highSpeedRow);

  // Camera windows panel.
  const cameraPanel = document.createElement('div');
  cameraPanel.className = 'glass-panel p-3 flex flex-col gap-2';
  cameraPanel.innerHTML = `
    <div class="text-xs font-semibold text-slate-300">RGB Camera</div>
    <div class="camera-window" id="rgb-container"></div>
    <div class="text-xs font-semibold text-slate-300 mt-1">Depth Camera</div>
    <div class="camera-window" id="depth-container"></div>
  `;

  // Plots panel.
  const plotsPanel = document.createElement('div');
  plotsPanel.className = 'glass-panel p-3 flex flex-col gap-2 min-w-[280px]';
  plotsPanel.innerHTML = `
    <div class="plot-container"><canvas id="joint-error-canvas"></canvas></div>
    <div class="plot-container"><canvas id="root-height-canvas"></canvas></div>
  `;

  bottom.append(playbackPanel);
  if (isSonic) bottom.append(optionsPanel);
  if (!isSonic) bottom.append(phpOptionsPanel);
  bottom.append(cameraPanel);
  if (isSonic) bottom.append(plotsPanel);
  root.appendChild(bottom);

  return {
    root,
    rgbContainer: cameraPanel.querySelector('#rgb-container') as HTMLDivElement,
    depthContainer: cameraPanel.querySelector('#depth-container') as HTMLDivElement,
    jointErrorCanvas: plotsPanel.querySelector('#joint-error-canvas') as HTMLCanvasElement | null,
    rootHeightCanvas: plotsPanel.querySelector('#root-height-canvas') as HTMLCanvasElement | null,
    policyStatusEl: phpOptionsPanel.querySelector('#policy-status') as HTMLDivElement | null,
    timeDisplay,
    timeSlider,
    durationEl,
  };
}

function resetModelState(scene: MuJoCoScene, mujoco: any, profile: ModelProfile) {
  mujoco.mj_resetData(scene.model, scene.data);
  if (profile.initialJointPos) {
    for (let i = 0; i < profile.initialJointPos.length; i++) {
      scene.data.qpos[7 + i] = profile.initialJointPos[i];
      scene.data.qvel[6 + i] = 0;
    }
  }
  mujoco.mj_forward(scene.model, scene.data);
  updateSceneTransforms(scene.model, scene.data, scene.bodies);
}

function createNumberRow(
  label: string,
  initial: number,
  min: number,
  max: number,
  step: number,
  onChange: (v: number) => void,
) {
  const row = document.createElement('div');
  row.className = 'flex items-center gap-2 text-sm';
  row.innerHTML = `<span class="text-slate-300 w-10">${label}</span>
    <input type="number" class="glass-input w-16 text-right" min="${min}" max="${max}" step="${step}" value="${initial}">`;
  const input = row.querySelector('input')!;
  input.onchange = () => onChange(parseFloat(input.value));
  return row;
}

function createCheckboxRow(
  label: string,
  initial: boolean,
  onChange: (v: boolean) => void,
) {
  const row = document.createElement('label');
  row.className = 'flex items-center justify-between gap-3 text-sm text-slate-300';
  const span = document.createElement('span');
  span.textContent = label;
  const input = document.createElement('input');
  input.type = 'checkbox';
  input.checked = initial;
  input.className = 'h-4 w-4 accent-blue-500';
  input.onchange = () => onChange(input.checked);
  row.append(span, input);
  return row;
}

init().catch((err) => {
  console.error('Initialization failed:', err);
  document.body.innerHTML = `
    <div class="p-8 text-red-400">
      <h1 class="text-xl font-bold">Failed to load visualizer</h1>
      <pre class="mt-2 text-sm">${err?.message || err}</pre>
    </div>`;
});
