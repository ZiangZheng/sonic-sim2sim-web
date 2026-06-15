import type loadMujoco from '@mujoco/mujoco';

type MujocoModule = Awaited<ReturnType<typeof loadMujoco>>;

export interface ControllerOptions {
  kp: number;
  kd: number;
}

export const DEFAULT_CONTROLLER_OPTIONS: ControllerOptions = {
  kp: 150,
  kd: 8,
};

export function applyPDControl(
  mujoco: MujocoModule,
  model: any,
  data: any,
  qposRef: Float32Array,
  qvelRef: Float32Array,
  options: ControllerOptions,
): void {
  const nq = model.nq;
  const nv = model.nv;
  const qpos = data.qpos;
  const qvel = data.qvel;
  const qfrc = data.qfrc_applied;

  // Clear applied forces before adding control torques.
  for (let i = 0; i < nv; i++) {
    qfrc[i] = 0.0;
  }

  // Root dofs have no actuation.
  for (let j = 0; j < model.njnt; j++) {
    const dofAdr = model.jnt_dofadr[j];
    const qposAdr = model.jnt_qposadr[j];
    const type = model.jnt_type[j];
    if (type === mujoco.mjtJoint.mjJNT_FREE.value) continue; // floating base

    const errPos = qposRef[qposAdr] - qpos[qposAdr];
    const errVel = qvelRef[dofAdr] - qvel[dofAdr];
    qfrc[dofAdr] = options.kp * errPos + options.kd * errVel;
  }
}

export function setStateKinematic(
  model: any,
  data: any,
  qposRef: Float32Array,
): void {
  for (let i = 0; i < model.nq; i++) {
    data.qpos[i] = qposRef[i];
  }
}
