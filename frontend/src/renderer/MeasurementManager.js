import * as THREE from 'three';

/**
 * MeasurementManager - Handles distance measurement visualization
 */
export class MeasurementManager {
    constructor(sceneManager) {
        this.sceneManager = sceneManager;
        this.measurementHelper = null;
    }

    /**
     * Show measurement between two objects
     * @param {THREE.Vector3} pos1 - World position of first object
     * @param {THREE.Vector3} pos2 - World position of second object
     * @param {Object} delta - Three-axis distances {x, y, z}
     * @param {number} totalDistance - Total distance
     * @param {string} name1 - First object name
     * @param {string} name2 - Second object name
     * @param {boolean} hasGround - Whether includes ground (if yes, only show vertical height)
     */
    showMeasurement(pos1, pos2, delta, totalDistance, name1, name2, hasGround = false) {
        // Clear previous measurement
        this.clearMeasurement();

        const measurementGroup = new THREE.Group();
        measurementGroup.name = 'measurementHelper';

        // Create materials
        const xAxisColor = 0xff0000; // Red X axis
        const yAxisColor = 0x00ff00; // Green Y axis
        const zAxisColor = 0x0000ff; // Blue Z axis
        const lineColor = 0xffff00;  // Yellow total distance line

        if (hasGround) {
            // If includes ground, only show vertical direction (Y axis) measurement
            const verticalHeight = Math.abs(delta.y);

            // Draw vertical line (green solid line)
            const verticalLine = new THREE.BufferGeometry().setFromPoints([pos1, pos2]);
            const verticalMaterial = new THREE.LineBasicMaterial({
                color: yAxisColor,
                linewidth: 3,
                depthTest: false
            });
            const line = new THREE.Line(verticalLine, verticalMaterial);
            line.renderOrder = 999;
            measurementGroup.add(line);

            // Vertical height label
            const midPoint = new THREE.Vector3().addVectors(pos1, pos2).multiplyScalar(0.5);
            const heightLabel = this.createLabel(
                `Δh ${(verticalHeight * 1000).toFixed(1)}mm`,
                midPoint,
                '#00ff00'
            );
            measurementGroup.add(heightLabel);

        } else {
            // Normal mode: show full three-axis measurement

            // 1. Draw total distance line (yellow dashed line) and label
            const lineGeometry = new THREE.BufferGeometry().setFromPoints([pos1, pos2]);
            const lineMaterial = new THREE.LineDashedMaterial({
                color: lineColor,
                dashSize: 0.01,
                gapSize: 0.005,
                linewidth: 2,
                depthTest: false
            });
            const line = new THREE.Line(lineGeometry, lineMaterial);
            line.computeLineDistances();
            line.renderOrder = 999;
            measurementGroup.add(line);

            // Total distance label (at line midpoint)
            const midTotal = new THREE.Vector3().addVectors(pos1, pos2).multiplyScalar(0.5);
            const totalLabel = this.createLabel(
                `${(totalDistance * 1000).toFixed(1)}mm`,
                midTotal,
                '#ffff00'
            );
            measurementGroup.add(totalLabel);

            // 2. Draw three-axis projection lines and labels
            // X axis projection (red)
            if (Math.abs(delta.x) > 0.001) {
                const xStart = new THREE.Vector3(pos1.x, pos1.y, pos1.z);
                const xEnd = new THREE.Vector3(pos2.x, pos1.y, pos1.z);
                const xPoints = [xStart, xEnd];
                const xGeometry = new THREE.BufferGeometry().setFromPoints(xPoints);
                const xMaterial = new THREE.LineBasicMaterial({
                    color: xAxisColor,
                    linewidth: 2,
                    depthTest: false
                });
                const xLine = new THREE.Line(xGeometry, xMaterial);
                xLine.renderOrder = 999;
                measurementGroup.add(xLine);

                // X axis label
                const xMid = new THREE.Vector3().addVectors(xStart, xEnd).multiplyScalar(0.5);
                const xLabel = this.createLabel(
                    `ΔX ${(delta.x * 1000).toFixed(1)}mm`,
                    xMid,
                    '#ff0000'
                );
                measurementGroup.add(xLabel);
            }

            // Y axis projection (green)
            if (Math.abs(delta.y) > 0.001) {
                const yStart = new THREE.Vector3(pos2.x, pos1.y, pos1.z);
                const yEnd = new THREE.Vector3(pos2.x, pos2.y, pos1.z);
                const yPoints = [yStart, yEnd];
                const yGeometry = new THREE.BufferGeometry().setFromPoints(yPoints);
                const yMaterial = new THREE.LineBasicMaterial({
                    color: yAxisColor,
                    linewidth: 2,
                    depthTest: false
                });
                const yLine = new THREE.Line(yGeometry, yMaterial);
                yLine.renderOrder = 999;
                measurementGroup.add(yLine);

                // Y axis label
                const yMid = new THREE.Vector3().addVectors(yStart, yEnd).multiplyScalar(0.5);
                const yLabel = this.createLabel(
                    `ΔY ${(delta.y * 1000).toFixed(1)}mm`,
                    yMid,
                    '#00ff00'
                );
                measurementGroup.add(yLabel);
            }

            // Z axis projection (blue)
            if (Math.abs(delta.z) > 0.001) {
                const zStart = new THREE.Vector3(pos2.x, pos2.y, pos1.z);
                const zEnd = new THREE.Vector3(pos2.x, pos2.y, pos2.z);
                const zPoints = [zStart, zEnd];
                const zGeometry = new THREE.BufferGeometry().setFromPoints(zPoints);
                const zMaterial = new THREE.LineBasicMaterial({
                    color: zAxisColor,
                    linewidth: 2,
                    depthTest: false
                });
                const zLine = new THREE.Line(zGeometry, zMaterial);
                zLine.renderOrder = 999;
                measurementGroup.add(zLine);

                // Z axis label
                const zMid = new THREE.Vector3().addVectors(zStart, zEnd).multiplyScalar(0.5);
                const zLabel = this.createLabel(
                    `ΔZ ${(delta.z * 1000).toFixed(1)}mm`,
                    zMid,
                    '#0000ff'
                );
                measurementGroup.add(zLabel);
            }
        }

        // 3. Add small sphere markers at two joint positions
        const sphereGeometry = new THREE.SphereGeometry(0.01, 16, 16);
        const sphereMaterial = new THREE.MeshBasicMaterial({
            color: 0xffff00,
            depthTest: false
        });

        const sphere1 = new THREE.Mesh(sphereGeometry, sphereMaterial);
        sphere1.position.copy(pos1);
        sphere1.renderOrder = 1000;
        measurementGroup.add(sphere1);

        const sphere2 = new THREE.Mesh(sphereGeometry, sphereMaterial);
        sphere2.position.copy(pos2);
        sphere2.renderOrder = 1000;
        measurementGroup.add(sphere2);

        this.sceneManager.scene.add(measurementGroup);
        this.measurementHelper = measurementGroup;
        this.sceneManager.redraw();
    }

    /**
     * Helper function: create text label (no background)
     */
    createLabel(text, position, color) {
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        canvas.width = 256;
        canvas.height = 64;

        // Fully transparent background
        context.clearRect(0, 0, canvas.width, canvas.height);

        // Add text stroke (black border) to make text clear on any background
        context.font = 'Bold 32px Arial';
        context.textAlign = 'center';
        context.textBaseline = 'middle';

        // Black stroke
        context.strokeStyle = 'rgba(0, 0, 0, 0.8)';
        context.lineWidth = 4;
        context.strokeText(text, 128, 32);

        // Colored text
        context.fillStyle = color;
        context.fillText(text, 128, 32);

        const texture = new THREE.CanvasTexture(canvas);
        const spriteMaterial = new THREE.SpriteMaterial({
            map: texture,
            depthTest: false,
            depthWrite: false,
            transparent: true
        });
        const sprite = new THREE.Sprite(spriteMaterial);
        sprite.position.copy(position);
        sprite.scale.set(0.18, 0.045, 1);
        sprite.renderOrder = 1000;
        return sprite;
    }

    /**
     * Clear measurement display
     */
    clearMeasurement() {
        if (this.measurementHelper) {
            this.sceneManager.scene.remove(this.measurementHelper);
            this.measurementHelper = null;
            this.sceneManager.redraw();
        }
    }

    /**
     * Clear all measurements
     */
    clear() {
        this.clearMeasurement();
    }
}

