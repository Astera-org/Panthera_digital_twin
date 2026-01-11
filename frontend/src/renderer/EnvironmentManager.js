/**
 * Environment Manager - Handles scene environment related functionality
 * Includes lighting, ground plane, coordinate system, etc.
 */
import * as THREE from 'three';

export class EnvironmentManager {
    constructor(scene) {
        this.scene = scene;
        this.lights = {};
        this.groundPlane = null;
        this.globalAxes = null;
        this.referenceGrid = null;
        this.envMap = null;
        this.pmremGenerator = null;
    }

    /**
     * Setup lighting system
     */
    setupLights() {
        // Hemisphere light
        this.lights.ambient = new THREE.HemisphereLight(0xffffff, 0x444444, 0.5);
        this.lights.ambient.position.set(0, 1, 0);
        this.scene.add(this.lights.ambient);

        // Main directional light
        this.lights.directional = new THREE.DirectionalLight(0xffffff, Math.PI);
        this.lights.directional.position.set(4, 10, 1);
        this.lights.directional.castShadow = true;

        // Optimize shadow quality
        this.lights.directional.shadow.mapSize.width = 2048;
        this.lights.directional.shadow.mapSize.height = 2048;
        this.lights.directional.shadow.camera.near = 0.1;
        this.lights.directional.shadow.camera.far = 50;
        this.lights.directional.shadow.normalBias = 0.001;

        // Set initial shadow camera range
        this.lights.directional.shadow.camera.left = -5;
        this.lights.directional.shadow.camera.right = 5;
        this.lights.directional.shadow.camera.top = 5;
        this.lights.directional.shadow.camera.bottom = -5;

        this.scene.add(this.lights.directional);
        this.scene.add(this.lights.directional.target);

        // Add fill light
        this.lights.fill = new THREE.DirectionalLight(0xffffff, Math.PI * 0.3);
        this.lights.fill.position.set(-2, 2, -2);
        this.lights.fill.castShadow = false;
        this.scene.add(this.lights.fill);

        // Setup environment map for reflections
        this.setupEnvironmentMap();
    }

    /**
     * Setup environment map for material reflections
     */
    setupEnvironmentMap(renderer = null) {
        // Create a simple environment map using PMREMGenerator
        // This will be used for material reflections
        if (renderer) {
            // Initialize PMREMGenerator if renderer is available
            if (!this.pmremGenerator) {
                this.pmremGenerator = new THREE.PMREMGenerator(renderer);
                this.pmremGenerator.compileEquirectangularShader();
            }

            // Create a simple scene for environment map generation
            const envScene = new THREE.Scene();

            // Add hemisphere light to environment scene
            const envLight = new THREE.HemisphereLight(0xffffff, 0x444444, 1.0);
            envScene.add(envLight);

            // Generate environment map
            this.envMap = this.pmremGenerator.fromScene(envScene, 0.04).texture;
            this.envMap.mapping = THREE.EquirectangularReflectionMapping;

            // Set environment map on scene
            this.scene.environment = this.envMap;
            this.scene.background = this.scene.background || new THREE.Color(0x505050);
        } else {
            // Fallback: create a simple cube texture environment map
            // This works without renderer but provides basic reflections
            const size = 256;
            const data = new Uint8Array(size * size * 4);
            const color = new THREE.Color(0xffffff);

            for (let i = 0; i < size * size; i++) {
                const stride = i * 4;
                data[stride] = Math.floor(color.r * 255);
                data[stride + 1] = Math.floor(color.g * 255);
                data[stride + 2] = Math.floor(color.b * 255);
                data[stride + 3] = 255;
            }

            const texture = new THREE.DataTexture(data, size, size);
            texture.needsUpdate = true;

            // Create cube texture from single color
            const cubeTexture = new THREE.CubeTexture([
                texture, texture, texture,
                texture, texture, texture
            ]);
            cubeTexture.needsUpdate = true;
            cubeTexture.mapping = THREE.CubeReflectionMapping;

            this.envMap = cubeTexture;
            this.scene.environment = this.envMap;
        }
    }

    /**
     * Initialize environment map with renderer (called after renderer is created)
     */
    initializeEnvironmentMap(renderer) {
        if (renderer && !this.envMap) {
            this.setupEnvironmentMap(renderer);
        }
    }

    /**
     * Get environment map
     */
    getEnvironmentMap() {
        return this.envMap;
    }

    /**
     * Setup ground plane
     */
    setupGroundPlane() {
        const planeGeometry = new THREE.PlaneGeometry(40, 40);
        const planeMaterial = new THREE.ShadowMaterial({
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.25
        });
        this.groundPlane = new THREE.Mesh(planeGeometry, planeMaterial);
        this.groundPlane.rotation.x = -Math.PI / 2;
        this.groundPlane.position.y = 0;  // Fixed at Y = 0
        this.groundPlane.receiveShadow = true;
        this.groundPlane.castShadow = false;
        this.groundPlane.visible = true;
        this.groundPlane.scale.set(10, 10, 10);
        this.groundPlane.name = 'groundPlane';

        this.scene.add(this.groundPlane);

        // Create reference grid
        this.createReferenceGrid();
    }

    /**
     * Create reference grid
     */
    createReferenceGrid() {
        const gridSize = 10;  // 10m x 10m
        const divisions = 20; // 20 divisions (0.5m per cell)

        // Use default white first (dark theme)
        this.referenceGrid = new THREE.GridHelper(gridSize, divisions, 0xffffff, 0xcccccc);
        this.referenceGrid.name = 'referenceGrid';
        this.referenceGrid.position.y = 0; // Fixed at Y = 0, aligned with ground

        // Increase opacity to make grid lines more visible (appear thicker)
        this.referenceGrid.material.opacity = 0.7;
        this.referenceGrid.material.transparent = true;

        // Disable depth write to avoid interference with ground
        this.referenceGrid.material.depthWrite = false;

        // Set higher render order to ensure grid lines display above ground
        this.referenceGrid.renderOrder = 1;

        this.scene.add(this.referenceGrid);
    }

    /**
     * Update grid color based on theme
     * @param {string} theme - 'light' or 'dark'
     */
    updateGridColorForTheme(theme) {
        if (!this.referenceGrid) return;

        if (theme === 'light') {
            // Light theme: black grid
            this.referenceGrid.material.color.setHex(0x000000); // Center line black
            // GridHelper uses two materials, need to update both
            const colors = this.referenceGrid.geometry.attributes.color;
            if (colors) {
                // Update all vertex colors to black and dark gray
                for (let i = 0; i < colors.count; i++) {
                    colors.setXYZ(i, 0, 0, 0); // Black
                }
                colors.needsUpdate = true;
            }
        } else {
            // Dark theme: white grid
            this.referenceGrid.material.color.setHex(0xffffff); // Center line white
            const colors = this.referenceGrid.geometry.attributes.color;
            if (colors) {
                // Update all vertex colors to white and light gray
                for (let i = 0; i < colors.count; i++) {
                    colors.setXYZ(i, 1, 1, 1); // White
                }
                colors.needsUpdate = true;
            }
        }
    }

    /**
     * Update directional light shadow camera range
     */
    updateShadowCamera(bbox) {
        const dirLight = this.lights.directional;
        if (!dirLight || !dirLight.castShadow) return;

        const sphere = bbox.getBoundingSphere(new THREE.Sphere());
        const minmax = sphere.radius;

        const cam = dirLight.shadow.camera;
        cam.left = cam.bottom = -minmax;
        cam.right = cam.top = minmax;

        // Make directional light follow model center
        const center = bbox.getCenter(new THREE.Vector3());
        const offset = dirLight.position.clone().sub(dirLight.target.position);
        dirLight.target.position.copy(center);
        dirLight.position.copy(center).add(offset);

        cam.updateProjectionMatrix();
    }

    /**
     * Update ground position (move to model lowest point)
     */
    updateGroundPosition(minY) {
        if (this.groundPlane) {
            this.groundPlane.position.y = minY;
        }
        // Grid should also follow ground movement
        if (this.referenceGrid) {
            this.referenceGrid.position.y = minY;
        }
    }

    /**
     * Set ground visibility
     */
    setGroundVisible(visible) {
        if (this.groundPlane) {
            this.groundPlane.visible = visible;
        }
    }

    /**
     * Enable/disable shadows
     */
    setShadowEnabled(enabled, renderer) {
        renderer.shadowMap.enabled = enabled;

        if (this.lights.directional) {
            this.lights.directional.castShadow = enabled;

            if (!enabled && this.lights.directional.shadow) {
                this.lights.directional.shadow.map?.dispose();
                this.lights.directional.shadow.map = null;
            }
        }
    }

    /**
     * Get directional light
     */
    getDirectionalLight() {
        return this.lights.directional;
    }
}

