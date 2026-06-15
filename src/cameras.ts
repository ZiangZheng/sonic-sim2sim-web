import * as THREE from 'three';

export class CameraWindow {
  canvas: HTMLCanvasElement;
  renderer: THREE.WebGLRenderer;
  camera: THREE.PerspectiveCamera;
  isDepth: boolean;
  depthMaterial: THREE.ShaderMaterial;
  private savedBackground: THREE.Color | null = null;

  constructor(container: HTMLElement, isDepth: boolean) {
    this.isDepth = isDepth;
    this.canvas = document.createElement('canvas');
    this.canvas.className = 'w-full h-full block';
    container.appendChild(this.canvas);

    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas,
      antialias: false,
      alpha: false,
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    this.renderer.setSize(container.clientWidth, container.clientHeight);

    this.camera = new THREE.PerspectiveCamera(
      58,
      container.clientWidth / container.clientHeight,
      0.05,
      20,
    );

    this.depthMaterial = new THREE.ShaderMaterial({
      uniforms: {
        near: { value: this.camera.near },
        far: { value: this.camera.far },
      },
      vertexShader: `
        varying float vDepth;
        void main() {
          vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
          vDepth = -mvPosition.z;
          gl_Position = projectionMatrix * mvPosition;
        }
      `,
      fragmentShader: `
        varying float vDepth;
        uniform float near;
        uniform float far;
        void main() {
          float gray = clamp((vDepth - near) / (far - near), 0.0, 1.0);
          gl_FragColor = vec4(vec3(gray), 1.0);
        }
      `,
      side: THREE.DoubleSide,
    });

    const resizeObserver = new ResizeObserver(() => {
      const w = container.clientWidth;
      const h = container.clientHeight;
      this.renderer.setSize(w, h);
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
    });
    resizeObserver.observe(container);
  }

  updateCamera(headPos: THREE.Vector3, headQuat: THREE.Quaternion, offset = new THREE.Vector3(0, 0, 0.08)) {
    const forward = offset.clone().applyQuaternion(headQuat);
    this.camera.position.copy(headPos).add(forward);
    this.camera.quaternion.copy(headQuat);
    this.camera.rotateX(-0.08); // slight downward tilt
  }

  render(scene: THREE.Scene, background?: THREE.Color | THREE.Texture) {
    if (this.isDepth) {
      this.savedBackground = scene.background as THREE.Color | null;
      scene.background = new THREE.Color(0xffffff);
      this.depthMaterial.uniforms.near.value = this.camera.near;
      this.depthMaterial.uniforms.far.value = this.camera.far;
      const originalOverride = scene.overrideMaterial;
      scene.overrideMaterial = this.depthMaterial;
      this.renderer.render(scene, this.camera);
      scene.overrideMaterial = originalOverride;
      scene.background = this.savedBackground;
    } else {
      this.renderer.render(scene, this.camera);
    }
  }
}
