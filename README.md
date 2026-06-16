# Sonic / PHP WASM Visualizer

A browser-based visualizer for Sonic motion-reference tracking and the PHP G1 scene, built with the official DeepMind [`@mujoco/mujoco`](https://www.npmjs.com/package/@mujoco/mujoco) WASM bindings.

It lets you choose between **Sonic** and **PHP**. Sonic plays motion references and can track them through physics; PHP loads the parkour terrain scene and runs the PHP ONNX policy.

## Features

- **Official MuJoCo WASM** — single-threaded `@mujoco/mujoco` physics in the browser.
- **Model / policy selection** — choose Sonic or PHP from the control panel.
- **Unitree G1** — 29-DOF robot model with STL meshes.
- **Motion playback** — load built-in or user-uploaded motion JSONs.
- **Sonic playback modes**
  - *Kinematic*: directly set `qpos` from the reference.
  - *Sonic*: run MuJoCo physics with a PD controller tracking the reference.
- **PHP policy controls** — enable the PHP ONNX policy and toggle auto-forward / high-speed commands.
- **First-person cameras** — RGB + depth windows attached to the robot head.
- **Real-time plots** — joint tracking error and root height.
- **Fancy glass UI** — dark theme, options panel, playback speed, PD gains.

## Quick start

```bash
npm install --include=dev
npm run dev
```

Open `http://localhost:5173`.

## Build

```bash
npm run build
```

The static site is output to `dist/` and can be deployed to GitHub Pages.

## Convert a Sonic deploy reference to motion JSON

```bash
python3 scripts/convert_motion_csv.py \
  /path/to/gear_sonic_deploy/reference/example/squat_001__A359 \
  public/motions/my_motion.json \
  --fps 30
```

The input directory must contain `joint_pos.csv`, `joint_vel.csv`, `body_pos.csv`, `body_quat.csv`, `body_lin_vel.csv`, and `body_ang_vel.csv`.

## Project structure

```
public/assets/g1/        # Shared G1 MuJoCo XML + STL meshes
public/assets/php/       # PHP terrain scene
public/motions/          # Sample converted motions
public/policies/php/     # PHP ONNX policy assets
scripts/                 # Python conversion utilities
src/
  main.ts                # App entry, render loop, UI
  mujocoScene.ts         # Load MuJoCo model and build Three.js scene
  motion.ts              # Motion JSON loading and interpolation
  controller.ts          # PD tracking controller
  cameras.ts             # RGB + depth first-person camera windows
  plots.ts               # Chart.js real-time plots
```

## License

MIT-like where applicable. The G1 robot model and Sonic reference data retain their original licenses.
