import type loadMujoco from '@mujoco/mujoco';

type MujocoModule = Awaited<ReturnType<typeof loadMujoco>>;

export interface ControllerOptions {
  kp: number;
  kd: number;
  maxTorque: number;
}

export const DEFAULT_CONTROLLER_OPTIONS: ControllerOptions = {
  kp: 120,
  kd: 12,
  maxTorque: 180,
};

const jointToActuatorCache = new WeakMap<object, Map<number, number>>();

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
  let jointToActuator = jointToActuatorCache.get(model);
  if (!jointToActuator) {
    jointToActuator = buildJointToActuatorMap(mujoco, model);
    jointToActuatorCache.set(model, jointToActuator);
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

    const actIdx = jointToActuator.get(j);
    if (actIdx !== undefined) {
      data.ctrl[actIdx] = clampActuatorControl(model, actIdx, torque, options.maxTorque);
    } else {
      // Fallback if no actuator is defined for this joint.
      data.qfrc_applied[dofAdr] = torque;
    }
  }
}

function clampActuatorControl(
  model: any,
  actuatorIndex: number,
  value: number,
  fallbackLimit: number,
): number {
  const limited = model.actuator_ctrllimited?.[actuatorIndex] ?? 0;
  const range = model.actuator_ctrlrange;
  if (!limited || !range || range.length < (actuatorIndex + 1) * 2) {
    if (Number.isFinite(fallbackLimit) && fallbackLimit > 0) {
      return Math.min(Math.max(value, -fallbackLimit), fallbackLimit);
    }
    return value;
  }

  const min = range[actuatorIndex * 2];
  const max = range[(actuatorIndex * 2) + 1];
  if (!Number.isFinite(min) || !Number.isFinite(max) || min >= max) {
    return value;
  }
  return Math.min(Math.max(value, min), max);
}

export function setStateKinematic(
  model: any,
  data: any,
  qposRef: Float32Array,
  qvelRef?: Float32Array,
): void {
  for (let i = 0; i < model.nq && i < qposRef.length; i++) {
    data.qpos[i] = qposRef[i];
  }
  if (qvelRef) {
    for (let i = 0; i < model.nv && i < qvelRef.length; i++) {
      data.qvel[i] = qvelRef[i];
    }
  }
}
