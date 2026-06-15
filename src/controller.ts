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

// Cache the mapping from joint index to actuator ctrl index so we don't
// rebuild it every physics step.
let jointToActuatorCache: Map<number, number> | null = null;

function buildJointToActuatorMap(mujoco: MujocoModule, model: any): Map<number, number> {
  const map = new Map<number, number>();
  for (let i = 0; i < model.nu; i++) {
    if (model.actuator_trntype[i] === mujoco.mjtTrn.mjTRN_JOINT.value) {
      const jointId = model.actuator_trnid[i * 2];
      map.set(jointId, i);
    }
  }
  return map;
}

export function applyPDControl(
  mujoco: MujocoModule,
  model: any,
  data: any,
  qposRef: Float32Array,
  qvelRef: Float32Array,
  options: ControllerOptions,
): void {
  if (!jointToActuatorCache || jointToActuatorCache.size === 0) {
    jointToActuatorCache = buildJointToActuatorMap(mujoco, model);
  }

  // Zero out any previously applied controls / forces.
  for (let i = 0; i < model.nu; i++) {
    data.ctrl[i] = 0.0;
  }
  for (let i = 0; i < model.nv; i++) {
    data.qfrc_applied[i] = 0.0;
  }

  for (let j = 0; j < model.njnt; j++) {
    const dofAdr = model.jnt_dofadr[j];
    const qposAdr = model.jnt_qposadr[j];
    const type = model.jnt_type[j];
    if (type === mujoco.mjtJoint.mjJNT_FREE.value) continue; // floating base

    const errPos = qposRef[qposAdr] - data.qpos[qposAdr];
    const errVel = qvelRef[dofAdr] - data.qvel[dofAdr];
    const torque = options.kp * errPos + options.kd * errVel;

    const actIdx = jointToActuatorCache.get(j);
    if (actIdx !== undefined) {
      data.ctrl[actIdx] = torque;
    } else {
      // Fallback if no actuator is defined for this joint.
      data.qfrc_applied[dofAdr] = torque;
    }
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
