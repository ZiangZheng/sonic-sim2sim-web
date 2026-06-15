export interface MotionClip {
  fps: number;
  duration: number;
  jointNames: string[];
  times: Float32Array;
  qpos: Float32Array[];
  qvel: Float32Array[];
  rootPos: Float32Array[];
  rootQuat: Float32Array[];
}

export async function loadMotionFromURL(url: string): Promise<MotionClip> {
  const res = await fetch(url);
  const json = await res.json();
  return normalizeMotion(json);
}

export function loadMotionFromFile(file: File): Promise<MotionClip> {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const json = JSON.parse(reader.result as string);
        resolve(normalizeMotion(json));
      } catch (e) {
        reject(e);
      }
    };
    reader.onerror = reject;
    reader.readAsText(file);
  });
}

function normalizeMotion(raw: any): MotionClip {
  const fps = raw.fps || 30;
  const times = new Float32Array(raw.times);
  const qpos = (raw.qpos as number[][]).map((row) => new Float32Array(row));
  const qvel = (raw.qvel as number[][]).map((row) => new Float32Array(row));
  const rootPos = (raw.root_pos as number[][]).map((row) => new Float32Array(row));
  const rootQuat = (raw.root_quat as number[][]).map((row) => new Float32Array(row));
  const duration = raw.duration || times[times.length - 1];
  return {
    fps,
    duration,
    jointNames: raw.joint_names || [],
    times,
    qpos,
    qvel,
    rootPos,
    rootQuat,
  };
}

export function sampleMotion(
  motion: MotionClip,
  time: number,
): { qpos: Float32Array; qvel: Float32Array; idx: number; alpha: number } {
  const { times, qpos, qvel } = motion;
  const n = times.length;
  if (n === 0) return { qpos: new Float32Array(0), qvel: new Float32Array(0), idx: 0, alpha: 0 };

  let t = Math.max(0, Math.min(time, times[n - 1]));
  let idx = 0;
  for (let i = 0; i < n - 1; i++) {
    if (t >= times[i] && t < times[i + 1]) {
      idx = i;
      break;
    }
  }
  const dt = times[idx + 1] - times[idx];
  const alpha = dt > 0 ? (t - times[idx]) / dt : 0;

  const a = qpos[idx];
  const b = qpos[Math.min(idx + 1, n - 1)];
  const outQpos = new Float32Array(a.length);
  outQpos[0] = a[0] + alpha * (b[0] - a[0]);
  outQpos[1] = a[1] + alpha * (b[1] - a[1]);
  outQpos[2] = a[2] + alpha * (b[2] - a[2]);
  // Spherical linear interpolation for root quaternion.
  slerp(a, b, outQpos, 3, alpha);
  for (let i = 7; i < a.length; i++) {
    outQpos[i] = a[i] + alpha * (b[i] - a[i]);
  }

  const va = qvel[idx];
  const vb = qvel[Math.min(idx + 1, n - 1)];
  const outQvel = new Float32Array(va.length);
  for (let i = 0; i < va.length; i++) {
    outQvel[i] = va[i] + alpha * (vb[i] - va[i]);
  }

  return { qpos: outQpos, qvel: outQvel, idx, alpha };
}

function slerp(
  a: Float32Array,
  b: Float32Array,
  out: Float32Array,
  offset: number,
  t: number,
) {
  let dot =
    a[offset] * b[offset] +
    a[offset + 1] * b[offset + 1] +
    a[offset + 2] * b[offset + 2] +
    a[offset + 3] * b[offset + 3];
  let qb0 = b[offset];
  let qb1 = b[offset + 1];
  let qb2 = b[offset + 2];
  let qb3 = b[offset + 3];
  if (dot < 0) {
    dot = -dot;
    qb0 = -qb0;
    qb1 = -qb1;
    qb2 = -qb2;
    qb3 = -qb3;
  }
  let theta0, theta, s0, s1;
  if (dot > 0.9995) {
    out[offset] = a[offset] + t * (qb0 - a[offset]);
    out[offset + 1] = a[offset + 1] + t * (qb1 - a[offset + 1]);
    out[offset + 2] = a[offset + 2] + t * (qb2 - a[offset + 2]);
    out[offset + 3] = a[offset + 3] + t * (qb3 - a[offset + 3]);
    const invLen = 1 / Math.hypot(out[offset], out[offset + 1], out[offset + 2], out[offset + 3]);
    out[offset] *= invLen;
    out[offset + 1] *= invLen;
    out[offset + 2] *= invLen;
    out[offset + 3] *= invLen;
    return;
  }
  theta0 = Math.acos(dot);
  theta = theta0 * t;
  s0 = Math.cos(theta) - dot * Math.sin(theta) / Math.sin(theta0);
  s1 = Math.sin(theta) / Math.sin(theta0);
  out[offset] = a[offset] * s0 + qb0 * s1;
  out[offset + 1] = a[offset + 1] * s0 + qb1 * s1;
  out[offset + 2] = a[offset + 2] * s0 + qb2 * s1;
  out[offset + 3] = a[offset + 3] * s0 + qb3 * s1;
}
