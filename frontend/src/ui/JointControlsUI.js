/**
 * JointControlsUI - Joint control UI module with robot connection support
 * Extended for Digital Twin to support real-time robot synchronization
 */
import { ModelLoaderFactory } from '../loaders/ModelLoaderFactory.js';
import { XMLUpdater } from '../utils/XMLUpdater.js';

export class JointControlsUI {
    constructor(sceneManager) {
        this.sceneManager = sceneManager;
        this.angleUnit = 'rad';
        this.initialJointValues = new Map();
        this.codeEditorManager = null;
        this.isUpdatingFromEditor = false;

        // Robot connection support
        this.robotConnection = null;
        this.isConnectedMode = false;
        this.isUpdatingFromRobot = false;  // Prevent feedback loops
        this.jointIndexMap = new Map();  // joint name -> index mapping

        // Control mode: 'position', 'gravity_comp', 'impedance'
        this.controlMode = 'position';
    }

    /**
     * Set robot connection for real-time synchronization
     */
    setRobotConnection(robotConnection) {
        this.robotConnection = robotConnection;
    }

    /**
     * Enable/disable connected mode
     * In connected mode, slider changes send commands to robot
     */
    setConnectedMode(enabled) {
        this.isConnectedMode = enabled;

        // Update UI to reflect mode
        const container = document.getElementById('joint-controls');
        if (container) {
            container.classList.toggle('connected-mode', enabled);
        }
    }

    /**
     * Set control mode and update UI accordingly
     * @param {string} mode - 'position', 'gravity_comp', or 'impedance'
     */
    setControlMode(mode) {
        this.controlMode = mode;

        const container = document.getElementById('joint-controls');
        if (!container) return;

        // Remove all mode classes
        container.classList.remove('gravity-mode', 'impedance-mode', 'position-mode');

        // Add current mode class
        if (mode === 'gravity_comp') {
            container.classList.add('gravity-mode');
        } else if (mode === 'impedance') {
            container.classList.add('impedance-mode');
        } else {
            container.classList.add('position-mode');
        }

        console.log(`[JointControlsUI] Control mode set to: ${mode}`);
    }

    /**
     * Get current control mode
     */
    getControlMode() {
        return this.controlMode;
    }

    /**
     * Update UI from robot state (called when receiving robot data)
     */
    updateFromRobotState(state) {
        if (!state || !state.positions) return;

        this.isUpdatingFromRobot = true;

        try {
            const positions = state.positions;

            // Update each slider
            document.querySelectorAll('.joint-slider').forEach((slider, index) => {
                if (index < positions.length) {
                    const position = positions[index];
                    const control = slider.closest('.joint-control');

                    // Check if value input is currently focused (user is typing)
                    const valueInput = control?.querySelector('.joint-value-input');
                    const isInputFocused = valueInput && document.activeElement === valueInput;

                    // Always update slider (doesn't interfere with typing)
                    slider.value = position;

                    // Only update value display if user is NOT typing
                    if (control && control._updateDisplay && !isInputFocused) {
                        control._updateDisplay();
                    }

                    // Update 3D model
                    const jointName = slider.getAttribute('data-joint');
                    if (jointName && this.sceneManager.currentModel) {
                        ModelLoaderFactory.setJointAngle(
                            this.sceneManager.currentModel,
                            jointName,
                            position
                        );
                    }
                }
            });

            // Render scene
            if (this.sceneManager) {
                this.sceneManager.redraw();
                this.sceneManager.render();
            }
        } finally {
            this.isUpdatingFromRobot = false;
        }
    }

    /**
     * Set code editor manager reference
     */
    setCodeEditorManager(codeEditorManager) {
        this.codeEditorManager = codeEditorManager;
    }

    /**
     * Update XML content in editor (URDF format only)
     */
    updateEditorXML(jointName, limits) {
        if (this.isUpdatingFromEditor) return;
        if (!this.codeEditorManager) return;

        const editor = this.codeEditorManager.getEditor();
        if (!editor) return;

        const currentContent = editor.getValue();
        if (!currentContent || !currentContent.includes('<robot')) return;

        this.isUpdatingFromEditor = true;

        try {
            const updatedXML = XMLUpdater.updateURDFJointLimits(currentContent, jointName, limits);
            if (updatedXML !== currentContent) {
                const cursorPos = editor.view.state.selection.main.head;
                editor.setValue(updatedXML);
                try {
                    const maxPos = editor.view.state.doc.length;
                    const newPos = Math.min(cursorPos, maxPos);
                    editor.view.dispatch({
                        selection: { anchor: newPos, head: newPos }
                    });
                } catch (e) {}
            }
        } catch (error) {
            console.error('Failed to update editor XML:', error);
        } finally {
            setTimeout(() => { this.isUpdatingFromEditor = false; }, 100);
        }
    }

    /**
     * Setup joint controls
     */
    setupJointControls(model, robotConfig = null) {
        const container = document.getElementById('joint-controls');
        if (!container) return;

        container.innerHTML = '';

        // Build joint index mapping if robot config provided
        this.jointIndexMap.clear();
        if (robotConfig && robotConfig.joints) {
            robotConfig.joints.forEach((jc, index) => {
                this.jointIndexMap.set(jc.name, index);
            });
        }

        if (!model || !model.joints || model.joints.size === 0) {
            const emptyState = document.createElement('div');
            emptyState.className = 'empty-state';
            emptyState.textContent = window.i18n ? window.i18n.t('noModel') : 'No model loaded';
            container.appendChild(emptyState);
            return;
        }

        let controllableJoints = 0;
        model.joints.forEach((joint) => {
            if (joint.type !== 'fixed') controllableJoints++;
        });

        if (controllableJoints === 0) {
            const emptyState = document.createElement('div');
            emptyState.className = 'empty-state';
            emptyState.textContent = window.i18n ? window.i18n.t('noControllableJoints') : 'No controllable joints';
            container.appendChild(emptyState);
            return;
        }

        // Save initial joint values
        this.initialJointValues.clear();
        let jointIndex = 0;
        model.joints.forEach((joint, name) => {
            if (joint.type !== 'fixed') {
                const limits = joint.limits || {};
                const lower = limits.lower !== undefined ? limits.lower : -Math.PI;
                const upper = limits.upper !== undefined ? limits.upper : Math.PI;
                const initialValue = joint.currentValue !== undefined ? joint.currentValue : (lower + upper) / 2;
                this.initialJointValues.set(name, initialValue);

                // Map joint name to index if not already mapped
                if (!this.jointIndexMap.has(name)) {
                    this.jointIndexMap.set(name, jointIndex);
                }
                jointIndex++;
            }
        });

        // Create controls for each joint
        jointIndex = 0;
        model.joints.forEach((joint, name) => {
            if (joint.type === 'fixed') return;
            const control = this.createJointControl(joint, model, jointIndex);
            container.appendChild(control);
            jointIndex++;
        });
    }

    /**
     * Create joint control element
     */
    createJointControl(joint, model, jointIndex) {
        const div = document.createElement('div');
        div.className = 'joint-control';
        div.setAttribute('data-joint-index', jointIndex);

        // Header row: name + value
        const header = document.createElement('div');
        header.className = 'joint-header';

        const name = document.createElement('div');
        name.className = 'joint-name';
        name.textContent = joint.name;
        name.title = joint.name;

        header.appendChild(name);

        // Slider row
        const sliderRow = document.createElement('div');
        sliderRow.className = 'joint-slider-row';

        const limits = joint.limits || {};
        let lower = limits.lower !== undefined ? limits.lower : -Math.PI;
        let upper = limits.upper !== undefined ? limits.upper : Math.PI;

        if (joint.type === 'continuous') {
            lower = -Math.PI;
            upper = Math.PI;
        }

        const slider = document.createElement('input');
        slider.type = 'range';
        slider.className = 'joint-slider';
        slider.setAttribute('data-joint', joint.name);
        slider.setAttribute('data-joint-index', jointIndex);
        slider.min = lower;
        slider.max = upper;

        let initialValue = joint.currentValue !== undefined ? joint.currentValue : (lower + upper) / 2;
        slider.value = initialValue;
        slider.step = (upper - lower) / 1000;

        // Min/max labels
        const minLabel = document.createElement('input');
        minLabel.type = 'number';
        minLabel.className = 'joint-limit-min editable-limit';
        minLabel.step = '0.01';
        minLabel.title = 'Click to edit min limit';

        const maxLabel = document.createElement('input');
        maxLabel.type = 'number';
        maxLabel.className = 'joint-limit-max editable-limit';
        maxLabel.step = '0.01';
        maxLabel.title = 'Click to edit max limit';

        // Value input
        const valueInput = document.createElement('input');
        valueInput.type = 'number';
        valueInput.className = 'joint-value-input';
        valueInput.setAttribute('data-joint-input', joint.name);
        valueInput.step = '0.01';

        const valueUnit = document.createElement('span');
        valueUnit.className = 'joint-value-unit';
        valueUnit.textContent = this.angleUnit === 'deg' ? '°' : 'rad';

        const updateLabels = () => {
            const currentMin = parseFloat(slider.min);
            const currentMax = parseFloat(slider.max);
            if (this.angleUnit === 'deg') {
                minLabel.value = (currentMin * 180 / Math.PI).toFixed(1);
                maxLabel.value = (currentMax * 180 / Math.PI).toFixed(1);
            } else {
                minLabel.value = currentMin.toFixed(2);
                maxLabel.value = currentMax.toFixed(2);
            }
        };

        const updateValueInput = () => {
            const value = parseFloat(slider.value);
            valueInput.value = this.angleUnit === 'deg' ?
                (value * 180 / Math.PI).toFixed(1) :
                value.toFixed(2);
        };

        updateLabels();
        updateValueInput();

        // Min limit change handler
        minLabel.addEventListener('change', () => {
            let inputValue = parseFloat(minLabel.value);
            if (isNaN(inputValue)) { updateLabels(); return; }

            let valueInRad = this.angleUnit === 'deg' ? inputValue * Math.PI / 180 : inputValue;
            const currentMax = parseFloat(slider.max);
            if (valueInRad >= currentMax) { updateLabels(); return; }

            slider.min = valueInRad;
            slider.step = (slider.max - slider.min) / 1000;

            if (joint.limits) joint.limits.lower = valueInRad;
            this.updateEditorXML(joint.name, { lower: valueInRad });

            const currentValue = parseFloat(slider.value);
            if (currentValue < valueInRad) {
                slider.value = valueInRad;
                this.handleJointChange(joint.name, valueInRad, jointIndex, model);
            }
            updateLabels();
        });

        // Max limit change handler
        maxLabel.addEventListener('change', () => {
            let inputValue = parseFloat(maxLabel.value);
            if (isNaN(inputValue)) { updateLabels(); return; }

            let valueInRad = this.angleUnit === 'deg' ? inputValue * Math.PI / 180 : inputValue;
            const currentMin = parseFloat(slider.min);
            if (valueInRad <= currentMin) { updateLabels(); return; }

            slider.max = valueInRad;
            slider.step = (slider.max - slider.min) / 1000;

            if (joint.limits) joint.limits.upper = valueInRad;
            this.updateEditorXML(joint.name, { upper: valueInRad });

            const currentValue = parseFloat(slider.value);
            if (currentValue > valueInRad) {
                slider.value = valueInRad;
                this.handleJointChange(joint.name, valueInRad, jointIndex, model);
            }
            updateLabels();
        });

        // Build slider container
        const sliderContainer = document.createElement('div');
        sliderContainer.className = 'joint-slider-container';
        sliderContainer.appendChild(slider);

        const valueInputContainer = document.createElement('div');
        valueInputContainer.className = 'joint-value-input-container';
        valueInputContainer.appendChild(valueInput);
        valueInputContainer.appendChild(valueUnit);

        sliderRow.appendChild(minLabel);
        sliderRow.appendChild(sliderContainer);
        sliderRow.appendChild(maxLabel);
        sliderRow.appendChild(valueInputContainer);

        // Slider events with robot integration
        slider.addEventListener('mousedown', () => {
            if (this.sceneManager.axesManager) {
                this.sceneManager.axesManager.showOnlyJointAxis(joint);
            }
        });

        slider.addEventListener('mouseup', () => {
            if (this.sceneManager.axesManager) {
                this.sceneManager.axesManager.restoreAllJointAxes();
            }
        });

        // Main slider input handler
        slider.addEventListener('input', () => {
            if (this.isUpdatingFromRobot) return;  // Don't send back to robot

            const value = parseFloat(slider.value);
            this.handleJointChange(joint.name, value, jointIndex, model);
            updateValueInput();
        });

        // Value input handler
        valueInput.addEventListener('change', () => {
            if (this.isUpdatingFromRobot) return;

            let inputValue = parseFloat(valueInput.value);
            if (isNaN(inputValue)) { updateValueInput(); return; }

            let valueInRad = this.angleUnit === 'deg' ? inputValue * Math.PI / 180 : inputValue;
            const currentMin = parseFloat(slider.min);
            const currentMax = parseFloat(slider.max);
            valueInRad = Math.max(currentMin, Math.min(currentMax, valueInRad));

            slider.value = valueInRad;
            this.handleJointChange(joint.name, valueInRad, jointIndex, model);
            updateValueInput();
        });

        // Save update function for external updates
        div._updateDisplay = () => {
            updateValueInput();
            updateLabels();
            valueUnit.textContent = this.angleUnit === 'deg' ? '°' : 'rad';
        };

        div.appendChild(header);
        div.appendChild(sliderRow);

        return div;
    }

    /**
     * Handle joint value change - sends to robot if connected, otherwise updates visualization
     * When connected: visualization is updated by robot state feedback (digital twin principle)
     * When offline: visualization is updated directly
     */
    handleJointChange(jointName, value, jointIndex, model) {
        // When connected to robot: ONLY send command, don't update visualization
        // Visualization will be updated by robot state feedback (updateFromRobotState)
        if (this.isConnectedMode && this.robotConnection && this.robotConnection.isConnected()) {
            const index = this.jointIndexMap.get(jointName);
            if (index !== undefined) {
                if (this.controlMode === 'position') {
                    // Position mode: direct joint control
                    this.robotConnection.moveJoint(index, value);
                } else if (this.controlMode === 'impedance') {
                    // Impedance mode: set target position
                    this.robotConnection.setImpedanceTarget(index, value);
                }
                // gravity_comp mode: sliders don't send commands (display only)
            }
            // Don't update visualization here - let robot state feedback do it
            return;
        }

        // Offline mode: update visualization directly
        ModelLoaderFactory.setJointAngle(model, jointName, value);

        const joint = model.joints.get(jointName);
        if (joint) joint.currentValue = value;

        // Apply constraints
        if (this.sceneManager.constraintManager) {
            this.sceneManager.constraintManager.applyConstraints(model, joint);
        }

        // Render
        requestAnimationFrame(() => {
            this.sceneManager.redraw();
            this.sceneManager.render();

            if (this.sceneManager.onMeasurementUpdate) {
                this.sceneManager.onMeasurementUpdate();
            }
        });
    }

    /**
     * Set angle unit
     */
    setAngleUnit(unit) {
        this.angleUnit = unit;
        const controls = document.querySelectorAll('.joint-control');
        controls.forEach(control => {
            if (control._updateDisplay) control._updateDisplay();
        });
    }

    /**
     * Reset all joints to initial positions
     * When connected: sends command to robot, visualization updated by robot state
     * When offline: updates visualization directly
     */
    resetAllJoints(model) {
        if (!model || !model.joints) return;

        const positions = [];

        // Collect initial positions
        model.joints.forEach((joint, name) => {
            if (joint.type !== 'fixed') {
                let initialValue = this.initialJointValues.get(name);

                if (initialValue === undefined) {
                    const limits = joint.limits || {};
                    const lower = limits.lower !== undefined ? limits.lower : -Math.PI;
                    const upper = limits.upper !== undefined ? limits.upper : Math.PI;
                    initialValue = joint.currentValue !== undefined ? joint.currentValue : (lower + upper) / 2;
                }

                positions.push(initialValue);
            }
        });

        // When connected: ONLY send command, don't update visualization
        if (this.isConnectedMode && this.robotConnection && this.robotConnection.isConnected()) {
            if (this.controlMode === 'position') {
                this.robotConnection.moveAll(positions);
            } else if (this.controlMode === 'impedance') {
                this.robotConnection.setImpedanceTargetAll(positions);
            }
            // gravity_comp mode: don't send commands
            // Visualization will be updated by robot state feedback
            return;
        }

        // Offline mode: update visualization directly
        let posIndex = 0;
        model.joints.forEach((joint, name) => {
            if (joint.type !== 'fixed') {
                const initialValue = positions[posIndex++];

                ModelLoaderFactory.setJointAngle(model, name, initialValue, true);
                joint.currentValue = initialValue;

                const slider = document.querySelector(`input[data-joint="${name}"]`);
                if (slider) {
                    slider.value = initialValue;
                    const control = slider.closest('.joint-control');
                    if (control && control._updateDisplay) control._updateDisplay();
                }
            }
        });

        this.sceneManager.render();

        if (this.sceneManager.onMeasurementUpdate) {
            this.sceneManager.onMeasurementUpdate();
        }
    }

    /**
     * Update limits for all sliders
     */
    updateAllSliderLimits(model, ignoreLimits) {
        if (!model) return;

        document.querySelectorAll('.joint-slider').forEach(slider => {
            const jointName = slider.getAttribute('data-joint');
            const joint = model.joints.get(jointName);

            if (joint && joint.type !== 'fixed') {
                if (ignoreLimits) {
                    slider.min = -Math.PI * 2;
                    slider.max = Math.PI * 2;
                    slider.step = 0.01;
                } else {
                    const limits = joint.limits || {};
                    const lower = limits.lower !== undefined ? limits.lower : -Math.PI;
                    const upper = limits.upper !== undefined ? limits.upper : Math.PI;

                    if (joint.type === 'continuous') {
                        slider.min = -Math.PI;
                        slider.max = Math.PI;
                    } else {
                        slider.min = lower;
                        slider.max = upper;
                    }
                    slider.step = (slider.max - slider.min) / 1000;
                }

                const control = slider.closest('.joint-control');
                if (control && control._updateDisplay) control._updateDisplay();
            }
        });
    }
}
