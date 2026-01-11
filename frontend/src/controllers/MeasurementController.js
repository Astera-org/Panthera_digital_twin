/**
 * MeasurementController - Measurement feature controller
 * Responsible for distance measurement between objects
 */
import * as THREE from 'three';
import * as d3 from 'd3';

export class MeasurementController {
    constructor(sceneManager) {
        this.sceneManager = sceneManager;
        this.selectedObjects = [];
    }

    /**
     * Handle measurement object selection
     */
    handleSelection(object, element, type) {
        const index = this.selectedObjects.findIndex(obj => obj.name === object.name && obj.type === type);

        if (index >= 0) {
            // Deselect
            this.selectedObjects.splice(index, 1);
            d3.select(element).classed('measurement-selected', false);
        } else {
            // Add selection
            if (this.selectedObjects.length >= 2) {
                const firstObj = this.selectedObjects.shift();
                const svg = d3.select('#model-graph-svg');

                if (firstObj.type === 'joint') {
                    svg.selectAll('.graph-joint-group')
                        .filter(d => d.target?.data?.jointName === firstObj.name)
                        .classed('measurement-selected', false);
                } else if (firstObj.type === 'link') {
                    svg.selectAll('.graph-node')
                        .filter(d => d.data?.data?.name === firstObj.name)
                        .classed('measurement-selected', false);
                }
            }

            this.selectedObjects.push({ ...object, type: type });
            d3.select(element).classed('measurement-selected', true);
        }

        // If 2 objects selected, show measurement result
        if (this.selectedObjects.length === 2) {
            this.showMeasurement(this.selectedObjects[0], this.selectedObjects[1]);
        } else {
            if (this.sceneManager) {
                this.sceneManager.measurementManager.clearMeasurement();
            }
        }
    }

    /**
     * Show measurement between two objects
     */
    showMeasurement(obj1, obj2) {
        if (!this.sceneManager) return;

        const getPosition = (obj) => {
            const pos = new THREE.Vector3();

            if (obj.type === 'joint' && obj.threeObject) {
                obj.threeObject.getWorldPosition(pos);
            } else if (obj.type === 'link') {
                if (obj.name === 'ground') {
                    pos.set(0, this.sceneManager.groundPlane?.position.y || 0, 0);
                } else if (obj.threeObject) {
                    obj.threeObject.getWorldPosition(pos);
                }
            }

            return pos;
        };

        const pos1 = getPosition(obj1);
        const pos2 = getPosition(obj2);

        const hasGround = obj1.name === 'ground' || obj2.name === 'ground';

        const delta = {
            x: pos2.x - pos1.x,
            y: pos2.y - pos1.y,
            z: pos2.z - pos1.z
        };
        const totalDistance = pos1.distanceTo(pos2);

        const name1 = obj1.name === 'ground' ? 'Ground' : obj1.name;
        const name2 = obj2.name === 'ground' ? 'Ground' : obj2.name;

        this.sceneManager.measurementManager.showMeasurement(pos1, pos2, delta, totalDistance, name1, name2, hasGround);
    }

    /**
     * Update current measurement
     */
    updateMeasurement() {
        if (this.selectedObjects.length === 2) {
            this.showMeasurement(this.selectedObjects[0], this.selectedObjects[1]);
        }
    }

    /**
     * Clear measurement
     */
    clearMeasurement() {
        this.selectedObjects = [];

        const svg = d3.select('#model-graph-svg');
        svg.selectAll('.graph-node').classed('measurement-selected', false);
        svg.selectAll('.graph-joint-group').classed('measurement-selected', false);

        if (this.sceneManager) {
            this.sceneManager.measurementManager.clearMeasurement();
        }
    }

    /**
     * Get selected objects
     */
    getSelectedObjects() {
        return this.selectedObjects;
    }
}

