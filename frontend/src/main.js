/**
 * Digital Twin - Main Application Entry Point
 * Combines robot visualization with real-time robot connection
 */
import * as THREE from 'three';
import * as d3 from 'd3';
import { SceneManager } from './renderer/SceneManager.js';
import { UIController } from './ui/UIController.js';
import { FileHandler } from './controllers/FileHandler.js';
import { JointControlsUI } from './ui/JointControlsUI.js';
import { PanelManager } from './ui/PanelManager.js';
import { ModelGraphView } from './views/ModelGraphView.js';
import { FileTreeView } from './views/FileTreeView.js';
import { MeasurementController } from './controllers/MeasurementController.js';
import { i18n } from './utils/i18n.js';
import { RobotConnection, robotConnection } from './robot/RobotConnection.js';
import { ConnectionUI } from './ui/ConnectionUI.js';
import { CameraConnection } from './camera/CameraConnection.js';

// Expose d3 globally for PanelManager
window.d3 = d3;
window.i18n = i18n;

class DigitalTwinApp {
    constructor() {
        this.sceneManager = null;
        this.uiController = null;
        this.fileHandler = null;
        this.jointControlsUI = null;
        this.panelManager = null;
        this.modelGraphView = null;
        this.fileTreeView = null;
        this.measurementController = null;
        this.currentModel = null;
        this.angleUnit = 'rad';

        // Robot connection
        this.robotConnection = robotConnection;
        this.connectionUI = null;
        this.robotConfig = null;
        this.isConnectedMode = false;
        this.endEffectorOffset = 0.07;  // Default offset from Link_6 origin to actual tool tip (meters)
        this.endEffectorMarker = null;  // Red dot showing actual end effector position

        // Camera streaming
        this.cameraConnection = new CameraConnection();
        this.cameraStreaming = false;
    }

    async init() {
        try {
            // Initialize i18n
            i18n.init();

            // Initialize scene manager
            const canvas = document.getElementById('canvas');
            if (!canvas) {
                console.error('Canvas element not found');
                return;
            }

            this.sceneManager = new SceneManager(canvas);
            window.sceneManager = this.sceneManager;

            // Initialize file handler
            this.fileHandler = new FileHandler();
            this.fileHandler.setupFileDrop();

            this.fileHandler.onFilesLoaded = (files) => {
                if (this.fileTreeView) {
                    this.fileTreeView.updateFileTree(files, this.fileHandler.getFileMap());
                }
            };

            this.fileHandler.onModelLoaded = (model, file, isMesh = false) => {
                this.handleModelLoaded(model, file, isMesh);
            };

            // Initialize joint controls UI with robot connection support
            this.jointControlsUI = new JointControlsUI(this.sceneManager);
            this.jointControlsUI.setRobotConnection(this.robotConnection);

            // Initialize model graph view
            this.modelGraphView = new ModelGraphView(this.sceneManager);

            // Initialize file tree view
            this.fileTreeView = new FileTreeView();
            this.fileTreeView.onFileClick = (fileInfo) => {
                this.handleFileClick(fileInfo);
            };
            this.fileTreeView.updateFileTree([], new Map());

            // Initialize panel manager
            this.panelManager = new PanelManager();
            this.panelManager.initAllPanels();

            if (this.modelGraphView) {
                this.panelManager.setModelGraphView(this.modelGraphView);
            }

            // Initialize UI controller
            this.uiController = new UIController(this.sceneManager);
            this.uiController.setupAll({
                onThemeChanged: (theme) => this.handleThemeChanged(theme),
                onAngleUnitChanged: (unit) => this.handleAngleUnitChanged(unit),
                onIgnoreLimitsChanged: (ignore) => this.handleIgnoreLimitsChanged(ignore),
                onLanguageChanged: (lang) => this.handleLanguageChanged(lang),
                onResetJoints: () => this.handleResetJoints()
            });

            // Set measurement update callback
            this.sceneManager.onMeasurementUpdate = () => {
                if (this.measurementController) {
                    this.measurementController.updateMeasurement();
                }
            };

            // Set joint drag update callback (for robot connection)
            // Returns true if robot is connected (to prevent local visualization update)
            this.sceneManager.onJointDragUpdate = (joint, angle, model) => {
                if (this.isConnectedMode && this.robotConnection && this.robotConnection.isConnected()) {
                    // Get joint index from name
                    const jointIndex = this.jointControlsUI.jointIndexMap.get(joint.name);
                    if (jointIndex !== undefined) {
                        const controlMode = this.jointControlsUI.getControlMode();
                        if (controlMode === 'position') {
                            this.robotConnection.moveJoint(jointIndex, angle);
                        } else if (controlMode === 'impedance') {
                            this.robotConnection.setImpedanceTarget(jointIndex, angle);
                        }
                        // gravity_comp mode: don't send commands
                    }
                    // Return true to indicate robot is connected - don't update visualization locally
                    return true;
                }
                // Return false to let SceneManager update visualization locally
                return false;
            };

            // Setup canvas click handler
            this.setupCanvasClickHandler(canvas);

            // Initialize measurement controller
            this.measurementController = new MeasurementController(this.sceneManager);

            if (this.modelGraphView) {
                this.modelGraphView.setMeasurementController(this.measurementController);
            }

            // Initialize connection UI
            this.connectionUI = new ConnectionUI(this.robotConnection);
            this.connectionUI.createUI();

            // Setup robot connection callbacks
            this.setupRobotConnectionCallbacks();

            // Setup model tree panel
            this.setupModelTreePanel();

            // Setup FK panel toggle
            this.setupFKPanel();

            // Setup Waypoints panel
            this.setupWaypointsPanel();

            // Setup Camera panel
            this.setupCameraPanel();

            // Start render loop
            this.animate();

            console.log('[DigitalTwin] Application initialized');

            // Auto-load default URDF from arm_description folder
            this.loadDefaultURDF();

        } catch (error) {
            console.error('Initialization error:', error);
        }
    }

    /**
     * Load default URDF from arm_description folder
     */
    async loadDefaultURDF() {
        try {
            // Fetch file list from backend
            const response = await fetch('/api/arm_description_files');
            if (!response.ok) {
                console.log('[DigitalTwin] No default URDF available (backend not running or arm_description not found)');
                return;
            }

            const data = await response.json();
            if (!data.success || !data.files) {
                console.log('[DigitalTwin] No files found in arm_description');
                return;
            }

            console.log('[DigitalTwin] Loading default URDF from arm_description...');

            // Load files from server
            await this.fileHandler.loadFromServer(data.base_url, data.files);

        } catch (error) {
            // This is expected when running without backend
            console.log('[DigitalTwin] Default URDF not loaded:', error.message);
        }
    }

    /**
     * Setup robot connection callbacks
     */
    setupRobotConnectionCallbacks() {
        // On connection established
        this.connectionUI.onConnect = (config) => {
            console.log('[DigitalTwin] Robot connected:', config);
            this.robotConfig = config;
            this.isConnectedMode = true;
            this.endEffectorOffset = config.end_effector_offset || 0;

            // Enable connected mode in joint controls
            this.jointControlsUI.setConnectedMode(true);

            // Set initial control mode from config
            if (config.control_mode) {
                this.jointControlsUI.setControlMode(config.control_mode);
            }

            // If model loaded, sync joint configuration
            if (this.currentModel) {
                this.jointControlsUI.setupJointControls(this.currentModel, config);
            }

            // Update FK panel status
            this.updateFKStatus(true, config.demo_mode ? 'Demo mode' : 'Connected');
        };

        // On disconnection
        this.connectionUI.onDisconnect = () => {
            console.log('[DigitalTwin] Robot disconnected');
            this.robotConfig = null;
            this.isConnectedMode = false;

            // Disable connected mode
            this.jointControlsUI.setConnectedMode(false);

            // Update FK panel status
            this.updateFKStatus(false, 'Not connected');
        };

        // On robot state update (real-time position streaming)
        this.robotConnection.onStateUpdate = (state) => {
            if (this.isConnectedMode && this.currentModel) {
                // Update joint positions from robot
                this.jointControlsUI.updateFromRobotState(state);
            }

            // Update FK display (always, when connected)
            if (state.forward_kinematics) {
                this.updateFKDisplay(state.forward_kinematics);
                // Also update ROB position in calibration display
                this.updateCalibrationRobPosition(state.forward_kinematics);
            }
        };

        // On mode changed
        this.connectionUI.onModeChanged = (mode) => {
            console.log('[DigitalTwin] Control mode changed to:', mode);
            if (this.jointControlsUI) {
                this.jointControlsUI.setControlMode(mode);
            }
        };
    }

    /**
     * Handle model loaded
     */
    handleModelLoaded(model, file, isMesh = false) {
        // Clear old model
        if (this.currentModel) {
            this.sceneManager.removeModel(this.currentModel);
            this.currentModel = null;
        }

        this.currentModel = model;

        // Add to scene
        this.sceneManager.addModel(model);

        // Hide drop zone
        const dropZone = document.getElementById('drop-zone');
        if (dropZone) {
            dropZone.classList.remove('show');
            dropZone.classList.remove('drag-over');
        }

        if (!isMesh) {
            // Normal model
            this.sceneManager.setGroundVisible(true);

            // Setup joint controls with robot config if connected
            if (this.isConnectedMode && this.robotConfig) {
                this.jointControlsUI.setupJointControls(model, this.robotConfig);
            } else {
                this.jointControlsUI.setupJointControls(model);
            }

            // Create end effector marker (small red dot at tool tip)
            this.createEndEffectorMarker();

            // Draw model graph
            if (this.modelGraphView) {
                this.modelGraphView.drawModelGraph(model);
            }

            // Show panels
            const graphPanel = document.getElementById('model-graph-panel');
            if (graphPanel) graphPanel.style.display = 'block';

            const jointsPanel = document.getElementById('joints-panel');
            if (jointsPanel) jointsPanel.style.display = 'block';

            // Hide axes by default
            this.setAxesButtonState(false);
        } else {
            // Mesh file
            this.sceneManager.setGroundVisible(false);

            // Clear graph
            if (this.modelGraphView) {
                const svg = d3.select('#model-graph-svg');
                svg.selectAll('*:not(defs)').remove();
                const emptyState = document.getElementById('graph-empty-state');
                if (emptyState) emptyState.classList.remove('hidden');
            }

            const graphPanel = document.getElementById('model-graph-panel');
            if (graphPanel) graphPanel.style.display = 'none';

            // Clear joint controls
            const jointContainer = document.getElementById('joint-controls');
            if (jointContainer) {
                jointContainer.innerHTML = '';
                const emptyState = document.createElement('div');
                emptyState.className = 'empty-state';
                emptyState.textContent = window.i18n?.t('noModel') || 'No model loaded';
                jointContainer.appendChild(emptyState);
            }

            const jointsPanel = document.getElementById('joints-panel');
            if (jointsPanel) jointsPanel.style.display = 'none';

            // Show axes for mesh files
            this.setAxesButtonState(true);
        }

        // Update file tree
        if (this.fileTreeView && !this._isReloading) {
            this.fileTreeView.updateFileTree(
                this.fileHandler.getAvailableModels(),
                this.fileHandler.getFileMap(),
                true
            );
            this.fileTreeView.expandAndScrollToFile(file, this.fileHandler.getFileMap());
        }

        // Update model info
        this.updateModelInfo(model, file);

        // Render
        this.sceneManager.redraw();
        this.sceneManager.render();
    }

    /**
     * Setup canvas click handler
     */
    setupCanvasClickHandler(canvas) {
        let mouseDownPos = null;
        let mouseDownTime = 0;

        canvas.addEventListener('mousedown', (event) => {
            if (event.button === 0) {
                mouseDownPos = { x: event.clientX, y: event.clientY };
                mouseDownTime = Date.now();
            }
        }, true);

        canvas.addEventListener('mouseup', (event) => {
            if (event.button !== 0 || !this.sceneManager || !mouseDownPos) return;

            const dx = event.clientX - mouseDownPos.x;
            const dy = event.clientY - mouseDownPos.y;
            const distance = Math.sqrt(dx * dx + dy * dy);
            const duration = Date.now() - mouseDownTime;

            if (distance < 5 && duration < 300) {
                const raycaster = new THREE.Raycaster();
                const mouse = new THREE.Vector2();

                const rect = canvas.getBoundingClientRect();
                mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
                mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

                raycaster.setFromCamera(mouse, this.sceneManager.camera);
                const intersects = raycaster.intersectObjects(this.sceneManager.scene.children, true);

                const modelIntersects = intersects.filter(intersect => {
                    const obj = intersect.object;
                    let current = obj;
                    while (current) {
                        const name = current.name || '';
                        if (name.includes('jointAxis') || name.includes('helper') ||
                            name.includes('grid') || name.includes('Ground') ||
                            name === 'groundPlane') {
                            return false;
                        }
                        current = current.parent;
                    }
                    return obj.isMesh && obj.visible;
                });

                if (modelIntersects.length === 0) {
                    this.sceneManager.highlightManager.clearHighlight();

                    if (this.modelGraphView) {
                        const svg = d3.select('#model-graph-svg');
                        this.modelGraphView.clearAllSelections(svg);
                    }

                    if (this.measurementController) {
                        this.measurementController.clearMeasurement();
                    }
                }
            }

            mouseDownPos = null;
        }, true);
    }

    /**
     * Setup model tree panel
     */
    setupModelTreePanel() {
        const toggleBtn = document.getElementById('toggle-model-tree');
        const floatingPanel = document.getElementById('floating-model-tree');

        if (toggleBtn && floatingPanel) {
            // Structure panel hidden by default
            floatingPanel.style.display = 'none';
            toggleBtn.classList.remove('active');
        }

        if (floatingPanel) {
            floatingPanel.addEventListener('click', (event) => {
                const target = event.target;

                if (target === floatingPanel ||
                    target.classList?.contains('graph-controls-hint') ||
                    target.classList?.contains('empty-state') ||
                    target.id === 'floating-model-tree') {

                    if (this.modelGraphView) {
                        const svg = d3.select('#model-graph-svg');
                        this.modelGraphView.clearAllSelections(svg);
                    }

                    if (this.measurementController) {
                        this.measurementController.clearMeasurement();
                    }

                    if (this.sceneManager) {
                        this.sceneManager.highlightManager.clearHighlight();
                    }
                }
            });
        }
    }

    /**
     * Setup FK (End Effector) panel toggle
     */
    setupFKPanel() {
        const toggleBtn = document.getElementById('toggle-fk-panel');
        const fkPanel = document.getElementById('floating-fk-panel');

        if (toggleBtn && fkPanel) {
            toggleBtn.addEventListener('click', () => {
                const isVisible = fkPanel.style.display !== 'none';
                fkPanel.style.display = isVisible ? 'none' : 'block';
                toggleBtn.classList.toggle('active', !isVisible);
            });

            // Close button
            const closeBtn = fkPanel.querySelector('.panel-close-btn');
            if (closeBtn) {
                closeBtn.addEventListener('click', () => {
                    fkPanel.style.display = 'none';
                    toggleBtn.classList.remove('active');
                });
            }
        }
    }

    /**
     * Update FK display with new data
     */
    updateFKDisplay(fk) {
        if (!fk) return;

        // Update position
        if (fk.position) {
            const posX = document.getElementById('fk-pos-x');
            const posY = document.getElementById('fk-pos-y');
            const posZ = document.getElementById('fk-pos-z');

            if (posX) posX.textContent = fk.position[0].toFixed(4);
            if (posY) posY.textContent = fk.position[1].toFixed(4);
            if (posZ) posZ.textContent = fk.position[2].toFixed(4);
        }

        // Update euler angles (orientation)
        if (fk.euler) {
            const roll = document.getElementById('fk-roll');
            const pitch = document.getElementById('fk-pitch');
            const yaw = document.getElementById('fk-yaw');

            if (roll) roll.textContent = fk.euler[0].toFixed(2);
            if (pitch) pitch.textContent = fk.euler[1].toFixed(2);
            if (yaw) yaw.textContent = fk.euler[2].toFixed(2);
        }
    }

    /**
     * Update ROB position in calibration display
     */
    updateCalibrationRobPosition(fk) {
        if (!fk || !fk.position) return;

        const robPosX = document.getElementById('rob-pos-x');
        const robPosY = document.getElementById('rob-pos-y');
        const robPosZ = document.getElementById('rob-pos-z');

        if (robPosX) robPosX.textContent = `X: ${fk.position[0].toFixed(3)}`;
        if (robPosY) robPosY.textContent = `Y: ${fk.position[1].toFixed(3)}`;
        if (robPosZ) robPosZ.textContent = `Z: ${fk.position[2].toFixed(3)}`;
    }

    /**
     * Update FK panel connection status
     */
    updateFKStatus(connected, statusText) {
        const statusDot = document.querySelector('.fk-status-dot');
        const statusTextEl = document.querySelector('.fk-status-text');

        if (statusDot) {
            statusDot.classList.toggle('connected', connected);
            statusDot.classList.toggle('disconnected', !connected);
        }

        if (statusTextEl) {
            statusTextEl.textContent = statusText || (connected ? 'Connected' : 'Not connected');
        }
    }

    /**
     * Setup Waypoints panel toggle and functionality
     */
    setupWaypointsPanel() {
        const toggleBtn = document.getElementById('toggle-waypoints-panel');
        const waypointsPanel = document.getElementById('floating-waypoints-panel');

        // Waypoints state
        this.waypoints = [];
        this.trajectoryRunning = false;
        this.waypointMarkers = [];  // 3D markers for visualization

        if (toggleBtn && waypointsPanel) {
            toggleBtn.addEventListener('click', () => {
                const isVisible = waypointsPanel.style.display !== 'none';
                waypointsPanel.style.display = isVisible ? 'none' : 'block';
                toggleBtn.classList.toggle('active', !isVisible);
            });

            // Close button
            const closeBtn = waypointsPanel.querySelector('.panel-close-btn');
            if (closeBtn) {
                closeBtn.addEventListener('click', () => {
                    waypointsPanel.style.display = 'none';
                    toggleBtn.classList.remove('active');
                });
            }
        }

        // Add waypoint button
        const addBtn = document.getElementById('add-waypoint-btn');
        if (addBtn) {
            addBtn.addEventListener('click', () => this.addWaypoint());
        }

        // Clear waypoints button
        const clearBtn = document.getElementById('clear-waypoints-btn');
        if (clearBtn) {
            clearBtn.addEventListener('click', () => this.clearWaypoints());
        }

        // Run trajectory button
        const runBtn = document.getElementById('run-trajectory-btn');
        if (runBtn) {
            runBtn.addEventListener('click', () => this.runTrajectory());
        }

        // Stop trajectory button
        const stopBtn = document.getElementById('stop-trajectory-btn');
        if (stopBtn) {
            stopBtn.addEventListener('click', () => this.stopTrajectory());
        }

        // Setup WebSocket listeners for trajectory events
        this.setupTrajectoryWebSocketListeners();
    }

    /**
     * Setup WebSocket listeners for trajectory events
     */
    setupTrajectoryWebSocketListeners() {
        if (!this.robotConnection || !this.robotConnection.socket) {
            // Try again after connection
            setTimeout(() => this.setupTrajectoryWebSocketListeners(), 1000);
            return;
        }

        const socket = this.robotConnection.socket;

        // Waypoints updated
        socket.on('waypoints_updated', (data) => {
            this.waypoints = data.waypoints || [];
            this.renderWaypoints();
        });

        // Trajectory progress
        socket.on('trajectory_progress', (data) => {
            this.updateTrajectoryProgress(data.progress);
        });

        // Trajectory complete
        socket.on('trajectory_complete', (data) => {
            this.onTrajectoryComplete(data.success, data.error);
        });

        // Trajectory error
        socket.on('trajectory_error', (data) => {
            console.error('Trajectory error:', data.error);
            this.onTrajectoryComplete(false, data.error);
        });
    }

    /**
     * Add current position as waypoint
     */
    addWaypoint() {
        if (this.waypoints.length >= 6) {
            console.log('Maximum 6 waypoints allowed');
            return;
        }

        if (this.robotConnection && this.robotConnection.isConnected()) {
            // Send via WebSocket
            this.robotConnection.socket.emit('add_waypoint', {
                duration: 1.0
            });
        } else {
            // Local mode - use current joint values
            const positions = this.jointControlsUI ?
                this.jointControlsUI.getCurrentPositions() :
                [0, 0, 0, 0, 0, 0];

            this.waypoints.push({
                positions: positions,
                duration: 1.0,
                index: this.waypoints.length
            });
            this.renderWaypoints();
        }
    }

    /**
     * Delete a waypoint
     */
    deleteWaypoint(index) {
        if (this.robotConnection && this.robotConnection.isConnected()) {
            this.robotConnection.socket.emit('delete_waypoint', { index });
        } else {
            this.waypoints.splice(index, 1);
            // Update indices
            this.waypoints.forEach((wp, i) => wp.index = i);
            this.renderWaypoints();
        }
    }

    /**
     * Go to a specific waypoint
     */
    goToWaypoint(index) {
        if (this.robotConnection && this.robotConnection.isConnected()) {
            this.robotConnection.socket.emit('go_to_waypoint', { index });
        } else if (this.jointControlsUI && this.waypoints[index]) {
            // Local mode - update joint controls
            const positions = this.waypoints[index].positions;
            positions.forEach((pos, i) => {
                this.jointControlsUI.setJointValue(i, pos);
            });
        }
    }

    /**
     * Update waypoint duration
     */
    updateWaypointDuration(index, duration) {
        if (this.robotConnection && this.robotConnection.isConnected()) {
            this.robotConnection.socket.emit('update_waypoint_duration', { index, duration });
        } else {
            if (this.waypoints[index]) {
                this.waypoints[index].duration = duration;
            }
        }
    }

    /**
     * Clear all waypoints
     */
    clearWaypoints() {
        if (this.robotConnection && this.robotConnection.isConnected()) {
            this.robotConnection.socket.emit('clear_waypoints');
        } else {
            this.waypoints = [];
            this.renderWaypoints();
        }
    }

    /**
     * Run trajectory through all waypoints
     */
    runTrajectory() {
        if (this.waypoints.length < 2) {
            console.log('Need at least 2 waypoints to run trajectory');
            return;
        }

        this.trajectoryRunning = true;
        this.updateTrajectoryUI(true);

        if (this.robotConnection && this.robotConnection.isConnected()) {
            this.robotConnection.socket.emit('run_trajectory', {
                control_rate: 100
            });
        } else {
            // Local demo mode simulation
            this.simulateLocalTrajectory();
        }
    }

    /**
     * Stop running trajectory
     */
    stopTrajectory() {
        this.trajectoryRunning = false;

        if (this.robotConnection && this.robotConnection.isConnected()) {
            this.robotConnection.socket.emit('stop_trajectory');
        }

        this.updateTrajectoryUI(false);
    }

    /**
     * Simulate trajectory in local mode (no backend)
     */
    async simulateLocalTrajectory() {
        if (this.waypoints.length < 2) return;

        const totalDuration = this.waypoints.slice(0, -1).reduce((sum, wp) => sum + wp.duration, 0);
        let elapsed = 0;

        for (let seg = 0; seg < this.waypoints.length - 1; seg++) {
            if (!this.trajectoryRunning) break;

            const startPos = this.waypoints[seg].positions;
            const endPos = this.waypoints[seg + 1].positions;
            const duration = this.waypoints[seg].duration;
            const steps = Math.floor(duration * 30); // 30 fps

            for (let step = 0; step < steps; step++) {
                if (!this.trajectoryRunning) break;

                const t = step / steps;
                const s = 3 * t * t - 2 * t * t * t; // Smooth interpolation

                const currentPos = startPos.map((start, i) =>
                    start + s * (endPos[i] - start)
                );

                // Update joint controls
                if (this.jointControlsUI) {
                    currentPos.forEach((pos, i) => {
                        this.jointControlsUI.setJointValue(i, pos);
                    });
                }

                elapsed += duration / steps;
                this.updateTrajectoryProgress(elapsed / totalDuration);

                await new Promise(resolve => setTimeout(resolve, 1000 / 30));
            }
        }

        this.onTrajectoryComplete(true);
    }

    /**
     * Update trajectory progress display
     */
    updateTrajectoryProgress(progress) {
        const progressFill = document.getElementById('trajectory-progress-fill');
        const progressText = document.getElementById('trajectory-progress-text');

        if (progressFill) {
            progressFill.style.width = `${Math.min(100, progress * 100)}%`;
        }

        if (progressText) {
            progressText.textContent = `${Math.round(progress * 100)}%`;
        }
    }

    /**
     * Handle trajectory completion
     */
    onTrajectoryComplete(success, error) {
        this.trajectoryRunning = false;
        this.updateTrajectoryUI(false);

        if (success) {
            console.log('Trajectory completed successfully');
        } else {
            console.error('Trajectory failed:', error);
        }

        // Reset progress
        setTimeout(() => {
            this.updateTrajectoryProgress(0);
        }, 1000);
    }

    /**
     * Update trajectory UI state
     */
    updateTrajectoryUI(running) {
        const runBtn = document.getElementById('run-trajectory-btn');
        const stopBtn = document.getElementById('stop-trajectory-btn');
        const progressContainer = document.querySelector('.trajectory-progress-container');
        const addBtn = document.getElementById('add-waypoint-btn');
        const clearBtn = document.getElementById('clear-waypoints-btn');

        if (runBtn) {
            runBtn.style.display = running ? 'none' : 'block';
        }

        if (stopBtn) {
            stopBtn.style.display = running ? 'block' : 'none';
        }

        if (progressContainer) {
            progressContainer.style.display = running ? 'flex' : 'none';
        }

        // Disable add/clear during trajectory
        if (addBtn) addBtn.disabled = running;
        if (clearBtn) clearBtn.disabled = running;

        // Disable delete buttons during trajectory
        const deleteButtons = document.querySelectorAll('.waypoint-action-btn.delete');
        deleteButtons.forEach(btn => btn.disabled = running);
    }

    /**
     * Render waypoints list
     */
    renderWaypoints() {
        const listContainer = document.getElementById('waypoints-list');
        const countEl = document.getElementById('waypoints-count');
        const runBtn = document.getElementById('run-trajectory-btn');
        const addBtn = document.getElementById('add-waypoint-btn');

        if (!listContainer) return;

        // Update 3D markers in the scene
        this.updateWaypointMarkers();

        // Update count
        if (countEl) {
            countEl.textContent = `${this.waypoints.length}/6 waypoints`;
        }

        // Update run button state
        if (runBtn) {
            runBtn.disabled = this.waypoints.length < 2;
        }

        // Update add button state
        if (addBtn) {
            addBtn.disabled = this.waypoints.length >= 6;
        }

        // Clear container
        listContainer.innerHTML = '';

        if (this.waypoints.length === 0) {
            listContainer.innerHTML = '<div class="waypoints-empty-state">No waypoints added</div>';
            return;
        }

        // Render each waypoint
        this.waypoints.forEach((waypoint, index) => {
            const item = document.createElement('div');
            item.className = 'waypoint-item';
            item.innerHTML = `
                <div class="waypoint-index">${index + 1}</div>
                <div class="waypoint-info">
                    <div class="waypoint-positions">${this.formatPositions(waypoint.positions)}</div>
                    ${index < this.waypoints.length - 1 ? `
                    <div class="waypoint-duration">
                        <span class="waypoint-duration-label">Duration:</span>
                        <input type="number" class="waypoint-duration-input"
                            value="${waypoint.duration.toFixed(1)}"
                            min="0.1" max="10" step="0.1"
                            data-index="${index}">
                        <span class="waypoint-duration-unit">s</span>
                    </div>
                    ` : ''}
                </div>
                <div class="waypoint-actions">
                    <button class="waypoint-action-btn goto" title="Go to this position" data-index="${index}">▶</button>
                    <button class="waypoint-action-btn delete" title="Delete waypoint" data-index="${index}">✕</button>
                </div>
            `;

            // Duration input handler
            const durationInput = item.querySelector('.waypoint-duration-input');
            if (durationInput) {
                durationInput.addEventListener('change', (e) => {
                    const idx = parseInt(e.target.dataset.index);
                    const duration = parseFloat(e.target.value) || 1.0;
                    this.updateWaypointDuration(idx, Math.max(0.1, Math.min(10, duration)));
                });
            }

            // Go to button
            const gotoBtn = item.querySelector('.waypoint-action-btn.goto');
            if (gotoBtn) {
                gotoBtn.addEventListener('click', (e) => {
                    const idx = parseInt(e.target.dataset.index);
                    this.goToWaypoint(idx);
                });
            }

            // Delete button
            const deleteBtn = item.querySelector('.waypoint-action-btn.delete');
            if (deleteBtn) {
                deleteBtn.addEventListener('click', (e) => {
                    const idx = parseInt(e.target.dataset.index);
                    this.deleteWaypoint(idx);
                });
            }

            listContainer.appendChild(item);
        });
    }

    /**
     * Format positions for display
     */
    formatPositions(positions) {
        if (!positions || positions.length === 0) return '';
        return positions.map(p => p.toFixed(2)).join(', ');
    }

    /**
     * Update 3D waypoint markers in the scene
     */
    updateWaypointMarkers() {
        // Clear existing markers
        this.clearWaypointMarkers();

        if (!this.sceneManager || !this.currentModel || this.waypoints.length === 0) {
            return;
        }

        // Store current joint positions to restore later
        const savedPositions = this.saveCurrentJointPositions();

        // Create a marker for each waypoint
        this.waypoints.forEach((waypoint, index) => {
            const endEffectorPos = this.getEndEffectorPositionForWaypoint(waypoint.positions);
            if (endEffectorPos) {
                const marker = this.createWaypointMarker(index + 1, endEffectorPos);
                if (marker) {
                    this.waypointMarkers.push(marker);
                    // Add directly to scene (not world) - getWorldPosition returns absolute coordinates
                    // Adding to world would apply its rotation transform again
                    this.sceneManager.scene.add(marker);
                }
            }
        });

        // Restore original joint positions
        this.restoreJointPositions(savedPositions);
        this.sceneManager.redraw();
    }

    /**
     * Create a dragon ball style waypoint marker
     * @param {number} number - The waypoint number (1-6)
     * @param {THREE.Vector3} position - World position for the marker
     * @returns {THREE.Group} The marker group
     */
    createWaypointMarker(number, position) {
        const group = new THREE.Group();

        // Create a crystal-like orange sphere (dragon ball style)
        const sphereRadius = 0.03;  // 3cm radius
        const sphereGeometry = new THREE.SphereGeometry(sphereRadius, 64, 64);

        // Crystal-like material - transparent with high shininess
        const sphereMaterial = new THREE.MeshPhysicalMaterial({
            color: 0xffa500,  // Orange
            emissive: 0xff6600,  // Orange glow
            emissiveIntensity: 0.15,
            transparent: true,
            opacity: 0.65,
            metalness: 0.0,
            roughness: 0.05,
            clearcoat: 1.0,
            clearcoatRoughness: 0.05,
            ior: 1.5,  // Glass-like refraction
            thickness: 0.5,
            transmission: 0.3  // Some light passes through
        });

        const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
        group.add(sphere);

        // Add inner glowing core
        const coreGeometry = new THREE.SphereGeometry(sphereRadius * 0.5, 32, 32);
        const coreMaterial = new THREE.MeshBasicMaterial({
            color: 0xffdd00,
            transparent: true,
            opacity: 0.5
        });
        const core = new THREE.Mesh(coreGeometry, coreMaterial);
        group.add(core);

        // Create star sprite (larger)
        const numberSprite = this.createNumberSprite(number);
        if (numberSprite) {
            numberSprite.position.set(0, 0, 0);
            numberSprite.scale.set(0.07, 0.07, 1);  // Larger sprite for visibility
            group.add(numberSprite);
        }

        group.position.copy(position);
        group.userData.waypointIndex = number - 1;
        group.userData.isWaypointMarker = true;
        group.userData.sphereRadius = sphereRadius;
        group.userData.sphereMaterial = sphereMaterial;
        group.userData.coreMaterial = coreMaterial;
        group.userData.isGlowing = false;

        return group;
    }

    /**
     * Create a sprite with a number for the waypoint
     * @param {number} number - The number to display (1-6)
     * @returns {THREE.Sprite} The number sprite
     */
    createNumberSprite(number) {
        const canvas = document.createElement('canvas');
        canvas.width = 256;  // Higher resolution
        canvas.height = 256;
        const ctx = canvas.getContext('2d');

        // Clear with transparency
        ctx.clearRect(0, 0, 256, 256);

        // Draw the star pattern (dragon ball style)
        const stars = Math.min(number, 7);
        const centerX = 128;
        const centerY = 128;
        const starRadius = 22;  // Much bigger stars

        ctx.fillStyle = '#8B0000';  // Dark red stars
        ctx.shadowColor = '#FF0000';
        ctx.shadowBlur = 8;

        if (stars === 1) {
            // Single star in center
            this.drawStar(ctx, centerX, centerY, starRadius);
        } else if (stars === 2) {
            this.drawStar(ctx, 80, 128, starRadius);
            this.drawStar(ctx, 176, 128, starRadius);
        } else if (stars === 3) {
            this.drawStar(ctx, 128, 70, starRadius);
            this.drawStar(ctx, 70, 160, starRadius);
            this.drawStar(ctx, 186, 160, starRadius);
        } else if (stars === 4) {
            this.drawStar(ctx, 80, 80, starRadius);
            this.drawStar(ctx, 176, 80, starRadius);
            this.drawStar(ctx, 80, 176, starRadius);
            this.drawStar(ctx, 176, 176, starRadius);
        } else if (stars === 5) {
            // Pentagon arrangement
            for (let i = 0; i < 5; i++) {
                const angle = (i / 5) * Math.PI * 2 - Math.PI / 2;
                const r = 55;
                const x = centerX + Math.cos(angle) * r;
                const y = centerY + Math.sin(angle) * r;
                this.drawStar(ctx, x, y, starRadius * 0.85);
            }
        } else if (stars === 6) {
            // Hexagon arrangement
            for (let i = 0; i < 6; i++) {
                const angle = (i / 6) * Math.PI * 2 - Math.PI / 2;
                const r = 60;
                const x = centerX + Math.cos(angle) * r;
                const y = centerY + Math.sin(angle) * r;
                this.drawStar(ctx, x, y, starRadius * 0.8);
            }
        }

        // Create texture and sprite
        const texture = new THREE.CanvasTexture(canvas);
        const spriteMaterial = new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false
        });

        return new THREE.Sprite(spriteMaterial);
    }

    /**
     * Draw a 5-pointed star
     */
    drawStar(ctx, cx, cy, radius) {
        const spikes = 5;
        const outerRadius = radius;
        const innerRadius = radius * 0.4;

        ctx.beginPath();
        for (let i = 0; i < spikes * 2; i++) {
            const r = i % 2 === 0 ? outerRadius : innerRadius;
            const angle = (i * Math.PI) / spikes - Math.PI / 2;
            const x = cx + Math.cos(angle) * r;
            const y = cy + Math.sin(angle) * r;
            if (i === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        }
        ctx.closePath();
        ctx.fill();
    }

    /**
     * Get end effector world position for a given set of joint positions
     * @param {number[]} jointPositions - Array of joint positions
     * @returns {THREE.Vector3|null} World position of end effector, or null
     */
    getEndEffectorPositionForWaypoint(jointPositions) {
        if (!this.currentModel || !jointPositions) return null;

        // Find end effector link (Link_6)
        const endEffectorLink = this.currentModel.links?.get('Link_6');
        if (!endEffectorLink || !endEffectorLink.threeObject) {
            console.warn('End effector link (Link_6) not found');
            return null;
        }

        // Temporarily set joint positions
        const joints = Array.from(this.currentModel.joints?.values() || [])
            .filter(j => j.type === 'revolute' || j.type === 'continuous' || j.type === 'prismatic');

        joints.forEach((joint, index) => {
            if (index < jointPositions.length && joint.threeObject?.setJointValue) {
                joint.threeObject.setJointValue(jointPositions[index]);
            }
        });

        // Update the model matrices
        if (this.currentModel.threeObject) {
            this.currentModel.threeObject.updateMatrixWorld(true);
        }

        // Get world position of Link_6 origin
        const position = new THREE.Vector3();
        endEffectorLink.threeObject.getWorldPosition(position);

        // Apply end effector offset along Link_6's local Z-axis
        if (this.endEffectorOffset > 0) {
            // Get Link_6's world rotation matrix
            const worldQuaternion = new THREE.Quaternion();
            endEffectorLink.threeObject.getWorldQuaternion(worldQuaternion);

            // Create offset vector along local Z-axis and rotate to world coordinates
            const offsetVector = new THREE.Vector3(0, 0, this.endEffectorOffset);
            offsetVector.applyQuaternion(worldQuaternion);

            // Add offset to position
            position.add(offsetVector);
        }

        return position;
    }

    /**
     * Save current joint positions
     * @returns {number[]} Array of current joint positions
     */
    saveCurrentJointPositions() {
        if (!this.currentModel) return [];

        const joints = Array.from(this.currentModel.joints?.values() || [])
            .filter(j => j.type === 'revolute' || j.type === 'continuous' || j.type === 'prismatic');

        return joints.map(joint => joint.currentValue || 0);
    }

    /**
     * Restore joint positions
     * @param {number[]} positions - Array of joint positions to restore
     */
    restoreJointPositions(positions) {
        if (!this.currentModel || !positions) return;

        const joints = Array.from(this.currentModel.joints?.values() || [])
            .filter(j => j.type === 'revolute' || j.type === 'continuous' || j.type === 'prismatic');

        joints.forEach((joint, index) => {
            if (index < positions.length && joint.threeObject?.setJointValue) {
                joint.threeObject.setJointValue(positions[index]);
            }
        });

        // Update matrices
        if (this.currentModel.threeObject) {
            this.currentModel.threeObject.updateMatrixWorld(true);
        }
    }

    /**
     * Clear all waypoint markers from the scene
     */
    clearWaypointMarkers() {
        this.waypointMarkers.forEach(marker => {
            if (marker.parent) {
                marker.parent.remove(marker);
            }
            // Dispose geometries and materials
            marker.traverse(child => {
                if (child.geometry) child.geometry.dispose();
                if (child.material) {
                    if (child.material.map) child.material.map.dispose();
                    child.material.dispose();
                }
            });
        });
        this.waypointMarkers = [];

        if (this.sceneManager) {
            this.sceneManager.redraw();
        }
    }

    /**
     * Update model info display
     */
    updateModelInfo(model, file) {
        const statusInfo = document.getElementById('status-info');
        if (!statusInfo || !model) return;

        let info = `<strong>${file.name}</strong><br>`;

        const fileType = file.name.split('.').pop().toLowerCase();
        info += `Type: ${fileType.toUpperCase()}<br>`;

        if (model.links) {
            info += `Links: ${model.links.size}<br>`;
        }

        if (model.joints) {
            const controllableJoints = Array.from(model.joints.values()).filter(j => j.type !== 'fixed').length;
            info += `Joints: ${model.joints.size} (${controllableJoints} controllable)<br>`;
        }

        if (model.constraints && model.constraints.size > 0) {
            info += `<span style="color: #00aaff; font-weight: bold;">Constraints: ${model.constraints.size}</span><br>`;
        }

        if (model.rootLink) {
            info += `Root Link: ${model.rootLink}`;
        }

        // Add connection status
        if (this.isConnectedMode) {
            const mode = this.robotConfig?.demo_mode ? 'Demo' : 'Live';
            info += `<br><span style="color: #00ff88;">Connected (${mode})</span>`;
        }

        statusInfo.innerHTML = info;
        statusInfo.className = 'success';
    }

    /**
     * Handle file click
     */
    handleFileClick(fileInfo) {
        const ext = fileInfo.ext;
        const modelExts = ['urdf', 'xml', 'usd', 'usda', 'usdc', 'usdz'];
        const meshExts = ['dae', 'stl', 'obj', 'collada'];

        if (modelExts.includes(ext)) {
            this.fileHandler.loadFile(fileInfo.file);
        } else if (meshExts.includes(ext)) {
            this.fileHandler.loadMeshAsModel(fileInfo.file, fileInfo.name);
        }
    }

    /**
     * Handle theme change
     */
    handleThemeChanged(theme) {
        if (this.currentModel && this.modelGraphView) {
            this.modelGraphView.drawModelGraph(this.currentModel);
        }
    }

    /**
     * Handle angle unit change
     */
    handleAngleUnitChanged(unit) {
        this.angleUnit = unit;
        if (this.jointControlsUI) {
            this.jointControlsUI.setAngleUnit(unit);
        }
    }

    /**
     * Handle reset joints button
     */
    handleResetJoints() {
        if (this.currentModel && this.jointControlsUI) {
            this.jointControlsUI.resetAllJoints(this.currentModel);
        }
    }

    /**
     * Handle ignore limits toggle
     */
    handleIgnoreLimitsChanged(ignore) {
        if (this.jointControlsUI && this.currentModel) {
            this.jointControlsUI.updateAllSliderLimits(this.currentModel, ignore);
        }
    }

    /**
     * Handle language change
     */
    handleLanguageChanged(lang) {
        i18n.setLanguage(lang);

        if (this.currentModel && this.jointControlsUI) {
            if (this.isConnectedMode && this.robotConfig) {
                this.jointControlsUI.setupJointControls(this.currentModel, this.robotConfig);
            } else {
                this.jointControlsUI.setupJointControls(this.currentModel);
            }
        }

        if (this.currentModel && this.modelGraphView) {
            this.modelGraphView.drawModelGraph(this.currentModel);
        }

        if (this.fileTreeView && this.fileHandler) {
            this.fileTreeView.updateFileTree(
                this.fileHandler.getAvailableModels(),
                this.fileHandler.getFileMap(),
                true
            );
        }
    }

    /**
     * Set axes button state
     */
    setAxesButtonState(show) {
        const axesBtn = document.getElementById('toggle-axes-btn');
        if (!axesBtn) return;

        axesBtn.setAttribute('data-checked', show.toString());
        if (show) {
            axesBtn.classList.add('active');
            if (this.sceneManager) {
                this.sceneManager.axesManager.showAllAxes();
            }
        } else {
            axesBtn.classList.remove('active');
            if (this.sceneManager) {
                this.sceneManager.axesManager.hideAllAxes();
            }
        }
    }

    /**
     * Animation loop
     */
    animate() {
        requestAnimationFrame(() => this.animate());
        if (this.sceneManager) {
            this.sceneManager.update();
            this.sceneManager.render();

            // Update end effector marker position
            this.updateEndEffectorMarker();
        }
    }

    /**
     * Create the end effector marker (small red sphere)
     */
    createEndEffectorMarker() {
        // Remove existing marker
        if (this.endEffectorMarker) {
            if (this.endEffectorMarker.parent) {
                this.endEffectorMarker.parent.remove(this.endEffectorMarker);
            }
            if (this.endEffectorMarker.geometry) this.endEffectorMarker.geometry.dispose();
            if (this.endEffectorMarker.material) this.endEffectorMarker.material.dispose();
            this.endEffectorMarker = null;
        }

        // Create small red sphere - render on top to always be visible
        const geometry = new THREE.SphereGeometry(0.005, 16, 16);  // 5mm radius
        const material = new THREE.MeshBasicMaterial({
            color: 0xff0000,  // Solid red
            depthTest: false,  // Always render on top
            depthWrite: false
        });

        this.endEffectorMarker = new THREE.Mesh(geometry, material);
        this.endEffectorMarker.renderOrder = 999;  // Render last
        this.endEffectorMarker.userData.isEndEffectorMarker = true;

        // Add to scene
        this.sceneManager.scene.add(this.endEffectorMarker);
        console.log('[DigitalTwin] End effector marker created');
    }

    /**
     * Update end effector marker position
     */
    updateEndEffectorMarker() {
        if (!this.endEffectorMarker || !this.currentModel) return;

        // Find Link_6
        const endEffectorLink = this.currentModel.links?.get('Link_6');
        if (!endEffectorLink || !endEffectorLink.threeObject) {
            // Log only once per missing link
            if (!this._loggedMissingLink) {
                console.warn('[DigitalTwin] Link_6 not found for end effector marker');
                this._loggedMissingLink = true;
            }
            return;
        }

        // Get world position of Link_6
        const position = new THREE.Vector3();
        endEffectorLink.threeObject.getWorldPosition(position);

        // Apply offset along Link_6's local Z-axis
        if (this.endEffectorOffset > 0) {
            const worldQuaternion = new THREE.Quaternion();
            endEffectorLink.threeObject.getWorldQuaternion(worldQuaternion);

            const offsetVector = new THREE.Vector3(0, 0, this.endEffectorOffset);
            offsetVector.applyQuaternion(worldQuaternion);

            position.add(offsetVector);
        }

        // Update marker position
        this.endEffectorMarker.position.copy(position);

        // Update FK position display to match the red point exactly
        this.updateFKPositionFromMarker(position);

        // Update waypoint glow effects based on proximity
        this.updateWaypointGlowEffects(position);
    }

    /**
     * Update FK position display from the end effector marker position
     * This ensures the displayed position matches the red point exactly
     */
    updateFKPositionFromMarker(position) {
        const posX = document.getElementById('fk-pos-x');
        const posY = document.getElementById('fk-pos-y');
        const posZ = document.getElementById('fk-pos-z');

        if (posX) posX.textContent = position.x.toFixed(4);
        if (posY) posY.textContent = position.y.toFixed(4);
        if (posZ) posZ.textContent = position.z.toFixed(4);
    }

    /**
     * Setup Camera panel and image streaming
     */
    setupCameraPanel() {
        const toggleBtn = document.getElementById('toggle-camera-panel');
        const cameraPanel = document.getElementById('floating-camera-panel');
        const startBtn = document.getElementById('camera-start-btn');
        const stopBtn = document.getElementById('camera-stop-btn');
        const statusIndicator = document.getElementById('camera-status-indicator');
        const statusText = document.getElementById('camera-status-text');
        const statsContainer = document.getElementById('camera-stats');
        const fpsEl = document.getElementById('camera-fps');
        const latencyEl = document.getElementById('camera-latency');
        const cameraImage = document.getElementById('camera-image');
        const cameraPlaceholder = document.getElementById('camera-placeholder');
        const monoBtn = document.getElementById('camera-mode-mono');
        const depthBtn = document.getElementById('camera-mode-depth');
        const selectBtn = document.getElementById('camera-mode-select');
        const videoContainer = document.getElementById('camera-video-container');
        const selectionBox = document.getElementById('camera-selection-box');
        // Calibration display elements
        const calibrationDisplay = document.getElementById('camera-calibration-display');
        const camPosX = document.getElementById('cam-pos-x');
        const camPosY = document.getElementById('cam-pos-y');
        const camPosZ = document.getElementById('cam-pos-z');
        const robPosX = document.getElementById('rob-pos-x');
        const robPosY = document.getElementById('rob-pos-y');
        const robPosZ = document.getElementById('rob-pos-z');
        const calibrationRecordBtn = document.getElementById('calibration-record-btn');
        const calibrationSaveBtn = document.getElementById('calibration-save-btn');
        const calibrationClearBtn = document.getElementById('calibration-clear-btn');
        const calibrationPairsList = document.getElementById('calibration-pairs-list');

        // Selection mode state
        let selectMode = false;
        let isSelecting = false;
        let selectionStart = null;

        // Panel toggle functionality
        if (toggleBtn && cameraPanel) {
            toggleBtn.addEventListener('click', () => {
                const isVisible = cameraPanel.style.display !== 'none';
                cameraPanel.style.display = isVisible ? 'none' : 'block';
                toggleBtn.classList.toggle('active', !isVisible);
            });

            // Close button
            const closeBtn = cameraPanel.querySelector('.panel-close-btn');
            if (closeBtn) {
                closeBtn.addEventListener('click', () => {
                    cameraPanel.style.display = 'none';
                    toggleBtn.classList.remove('active');
                });
            }
        }

        // Camera frame callback - display image
        this.cameraConnection.onFrame = (data) => {
            // Update image display
            if (cameraImage && data.image) {
                cameraImage.src = `data:image/jpeg;base64,${data.image}`;
                cameraImage.style.display = 'block';
                if (cameraPlaceholder) cameraPlaceholder.style.display = 'none';
            }

            // Update stats display
            if (fpsEl) fpsEl.textContent = Math.round(data.fps);
            if (latencyEl) latencyEl.textContent = Math.round(data.latency_ms);
        };

        this.cameraConnection.onStatusChange = (streaming, config, message) => {
            this.cameraStreaming = streaming;

            // Update UI
            if (statusIndicator) {
                statusIndicator.classList.toggle('streaming', streaming);
            }
            if (statusText) {
                statusText.textContent = streaming ? 'Streaming' : (message || 'Not streaming');
            }
            if (statsContainer) {
                statsContainer.style.display = streaming ? 'flex' : 'none';
            }
            if (startBtn) {
                startBtn.style.display = streaming ? 'none' : 'block';
            }
            if (stopBtn) {
                stopBtn.style.display = streaming ? 'block' : 'none';
            }

            // Show/hide image vs placeholder
            if (!streaming) {
                if (cameraImage) cameraImage.style.display = 'none';
                if (cameraPlaceholder) cameraPlaceholder.style.display = 'flex';
            }

            console.log(`[DigitalTwin] Camera ${streaming ? 'started' : 'stopped'}: ${message}`);
        };

        // Start button click
        if (startBtn) {
            startBtn.addEventListener('click', () => {
                if (this.robotConnection && this.robotConnection.socket) {
                    this.cameraConnection.startCamera({
                        resolution: [640, 400],
                        fps: 60  // 60fps for smooth video
                    });
                } else {
                    console.warn('[DigitalTwin] Cannot start camera: not connected to backend');
                    if (statusText) statusText.textContent = 'Connect to backend first';
                }
            });
        }

        // Stop button click
        if (stopBtn) {
            stopBtn.addEventListener('click', () => {
                this.cameraConnection.stopCamera();
            });
        }

        // Mode button click handlers
        const updateModeButtons = (mode) => {
            if (monoBtn) monoBtn.classList.toggle('active', mode === 'mono');
            if (depthBtn) depthBtn.classList.toggle('active', mode === 'depth');
        };

        if (monoBtn) {
            monoBtn.addEventListener('click', () => {
                this.cameraConnection.setMode('mono');
                updateModeButtons('mono');
            });
        }

        if (depthBtn) {
            depthBtn.addEventListener('click', () => {
                this.cameraConnection.setMode('depth');
                updateModeButtons('depth');
            });
        }

        // Mode change callback (from other clients)
        this.cameraConnection.onModeChange = (mode) => {
            updateModeButtons(mode);
        };

        // Select button - toggle selection mode
        if (selectBtn) {
            selectBtn.addEventListener('click', () => {
                selectMode = !selectMode;
                selectBtn.classList.toggle('active', selectMode);
                if (videoContainer) {
                    videoContainer.classList.toggle('selecting', selectMode);
                }
                // Show/hide calibration display
                if (calibrationDisplay) {
                    calibrationDisplay.style.display = selectMode ? 'flex' : 'none';
                }
                // Clear circle when exiting select mode
                if (!selectMode && videoContainer && videoContainer._clearCircle) {
                    videoContainer._clearCircle();
                }
            });
        }

        // Calibration button handlers
        if (calibrationRecordBtn) {
            calibrationRecordBtn.addEventListener('click', () => {
                if (this.robotConnection && this.robotConnection.socket) {
                    console.log('[Calibration] Recording pair...');
                    this.robotConnection.socket.emit('record_calibration_pair');
                }
            });
        }

        if (calibrationSaveBtn) {
            calibrationSaveBtn.addEventListener('click', () => {
                if (this.robotConnection && this.robotConnection.socket) {
                    console.log('[Calibration] Saving to YAML...');
                    this.robotConnection.socket.emit('save_calibration_pairs');
                }
            });
        }

        if (calibrationClearBtn) {
            calibrationClearBtn.addEventListener('click', () => {
                if (this.robotConnection && this.robotConnection.socket) {
                    if (confirm('Clear all recorded calibration pairs?')) {
                        console.log('[Calibration] Clearing pairs...');
                        this.robotConnection.socket.emit('clear_calibration_pairs');
                    }
                }
            });
        }

        // Listen for calibration responses
        if (this.robotConnection && this.robotConnection.socket) {
            this.robotConnection.socket.on('calibration_response', (data) => {
                if (data.success) {
                    console.log('[Calibration]', data.message);
                    if (data.filepath) {
                        alert(`Saved to: ${data.filename}`);
                    }
                } else {
                    console.error('[Calibration] Error:', data.error);
                    alert(`Calibration error: ${data.error}`);
                }
            });

            this.robotConnection.socket.on('calibration_pairs_updated', (data) => {
                // Update pairs list
                if (calibrationPairsList && data.pairs) {
                    calibrationPairsList.innerHTML = data.pairs.map((pair, i) =>
                        `<div class="calibration-pair-item">#${i+1}: CAM(${pair.camera_frame.x.toFixed(3)}, ${pair.camera_frame.y.toFixed(3)}, ${pair.camera_frame.z.toFixed(3)}) → ROB(${pair.robot_frame.x.toFixed(3)}, ${pair.robot_frame.y.toFixed(3)}, ${pair.robot_frame.z.toFixed(3)})</div>`
                    ).join('');
                } else if (calibrationPairsList && data.total_pairs === 0) {
                    calibrationPairsList.innerHTML = '';
                }
            });
        }

        // Persistent circle state (in image coordinates)
        let circleCenter = null;  // {x, y} in image coords
        let circleRadius = 0;     // in image coords
        let dragMode = null;      // 'draw', 'move', 'resize', or null

        // Mouse event handlers for circle selection with reposition/resize support
        if (videoContainer) {
            const getImageCoordinates = (e) => {
                // Get the image element's bounding rect (or container if image not visible)
                const img = cameraImage && cameraImage.style.display !== 'none' ? cameraImage : videoContainer;
                const rect = img.getBoundingClientRect();
                const scaleX = 640 / rect.width;  // Image native width
                const scaleY = 400 / rect.height; // Image native height
                return {
                    x: Math.max(0, Math.min(640, (e.clientX - rect.left) * scaleX)),
                    y: Math.max(0, Math.min(400, (e.clientY - rect.top) * scaleY)),
                    displayX: e.clientX - rect.left,
                    displayY: e.clientY - rect.top,
                    scaleX: scaleX,
                    scaleY: scaleY,
                    rect: rect
                };
            };

            const updateCircleDisplay = (center, radius, coords) => {
                if (!selectionBox || !center) return;
                // Convert image coords to display coords
                const displayX = center.x / coords.scaleX;
                const displayY = center.y / coords.scaleY;
                const displayRadius = radius / coords.scaleX;  // Assume uniform scale
                const diameter = displayRadius * 2;

                selectionBox.style.left = displayX + 'px';
                selectionBox.style.top = displayY + 'px';
                selectionBox.style.width = diameter + 'px';
                selectionBox.style.height = diameter + 'px';
                selectionBox.style.display = 'block';
            };

            const queryDepthForCircle = () => {
                if (!circleCenter || circleRadius < 2) return;
                const box = [
                    Math.max(0, circleCenter.x - circleRadius),
                    Math.max(0, circleCenter.y - circleRadius),
                    Math.min(640, circleCenter.x + circleRadius),
                    Math.min(400, circleCenter.y + circleRadius)
                ];
                console.log('[DigitalTwin] Querying depth at circle center:',
                    [circleCenter.x, circleCenter.y], 'radius:', circleRadius, 'box:', box);
                this.cameraConnection.queryDepth(box);
            };

            const EDGE_THRESHOLD = 8;  // pixels - how close to edge to trigger resize

            // Update cursor based on hover position (called when not dragging)
            const updateCursor = (coords) => {
                if (!circleCenter || circleRadius <= 0) {
                    videoContainer.style.cursor = 'crosshair';
                    return;
                }

                const distToCenter = Math.sqrt(
                    Math.pow(coords.x - circleCenter.x, 2) +
                    Math.pow(coords.y - circleCenter.y, 2)
                );
                const distToEdge = Math.abs(distToCenter - circleRadius);
                const edgeThresholdImg = EDGE_THRESHOLD * coords.scaleX;

                if (distToEdge < edgeThresholdImg) {
                    videoContainer.style.cursor = 'nwse-resize';  // Resize cursor
                } else if (distToCenter < circleRadius) {
                    videoContainer.style.cursor = 'move';  // Move cursor
                } else {
                    videoContainer.style.cursor = 'crosshair';  // Draw new
                }
            };

            videoContainer.addEventListener('mousedown', (e) => {
                if (!selectMode) return;
                e.preventDefault();

                const coords = getImageCoordinates(e);
                isSelecting = true;

                // Check if clicking on existing circle
                if (circleCenter && circleRadius > 0) {
                    const distToCenter = Math.sqrt(
                        Math.pow(coords.x - circleCenter.x, 2) +
                        Math.pow(coords.y - circleCenter.y, 2)
                    );
                    const distToEdge = Math.abs(distToCenter - circleRadius);
                    const edgeThresholdImg = EDGE_THRESHOLD * coords.scaleX;

                    if (distToEdge < edgeThresholdImg) {
                        // Click near edge - resize mode
                        dragMode = 'resize';
                        selectionStart = coords;
                    } else if (distToCenter < circleRadius) {
                        // Click inside circle - move mode
                        dragMode = 'move';
                        selectionStart = coords;
                    } else {
                        // Click outside circle - draw new circle
                        dragMode = 'draw';
                        circleCenter = { x: coords.x, y: coords.y };
                        circleRadius = 0;
                        selectionStart = coords;
                        updateCircleDisplay(circleCenter, circleRadius, coords);
                    }
                } else {
                    // No existing circle - draw new one
                    dragMode = 'draw';
                    circleCenter = { x: coords.x, y: coords.y };
                    circleRadius = 0;
                    selectionStart = coords;
                    updateCircleDisplay(circleCenter, circleRadius, coords);
                }
            });

            videoContainer.addEventListener('mousemove', (e) => {
                if (!selectMode) return;

                const coords = getImageCoordinates(e);

                // If not dragging, just update cursor
                if (!isSelecting || !selectionStart || !dragMode) {
                    updateCursor(coords);
                    return;
                }

                if (dragMode === 'draw') {
                    // Drawing new circle - radius is distance from center
                    const dx = coords.x - circleCenter.x;
                    const dy = coords.y - circleCenter.y;
                    circleRadius = Math.sqrt(dx * dx + dy * dy);
                } else if (dragMode === 'move') {
                    // Moving circle - update center based on drag delta
                    const dx = coords.x - selectionStart.x;
                    const dy = coords.y - selectionStart.y;
                    circleCenter.x = Math.max(circleRadius, Math.min(640 - circleRadius, circleCenter.x + dx));
                    circleCenter.y = Math.max(circleRadius, Math.min(400 - circleRadius, circleCenter.y + dy));
                    selectionStart = coords;  // Update for next delta
                } else if (dragMode === 'resize') {
                    // Resizing - radius is distance from center to mouse
                    const dx = coords.x - circleCenter.x;
                    const dy = coords.y - circleCenter.y;
                    circleRadius = Math.max(2, Math.sqrt(dx * dx + dy * dy));  // Min radius of 2
                }

                updateCircleDisplay(circleCenter, circleRadius, coords);
            });

            videoContainer.addEventListener('mouseup', (e) => {
                if (!isSelecting) return;

                isSelecting = false;
                const coords = getImageCoordinates(e);

                // Finalize based on mode
                if (dragMode === 'draw') {
                    const dx = coords.x - circleCenter.x;
                    const dy = coords.y - circleCenter.y;
                    circleRadius = Math.sqrt(dx * dx + dy * dy);
                }

                // Query depth if circle is valid (very small threshold - 2 pixels)
                if (circleRadius >= 2) {
                    updateCircleDisplay(circleCenter, circleRadius, coords);
                    queryDepthForCircle();
                } else {
                    // Too small - clear circle
                    circleCenter = null;
                    circleRadius = 0;
                    if (selectionBox) selectionBox.style.display = 'none';
                }

                dragMode = null;
                selectionStart = null;
            });

            // Don't cancel on mouse leave - keep circle visible
            videoContainer.addEventListener('mouseleave', () => {
                if (isSelecting) {
                    isSelecting = false;
                    dragMode = null;
                    selectionStart = null;
                    // Keep circle displayed if it exists
                }
            });

            // Clear circle when exiting select mode
            const clearCircle = () => {
                circleCenter = null;
                circleRadius = 0;
                if (selectionBox) selectionBox.style.display = 'none';
            };

            // Store clearCircle for use in select button handler
            videoContainer._clearCircle = clearCircle;
        }

        // CAM2BOT display elements
        const cam2botRow = document.getElementById('cam2bot-row');
        const cam2botPosX = document.getElementById('cam2bot-pos-x');
        const cam2botPosY = document.getElementById('cam2bot-pos-y');
        const cam2botPosZ = document.getElementById('cam2bot-pos-z');

        // Depth response callback - update CAM and CAM2BOT position display
        this.cameraConnection.onDepthResponse = (data) => {
            console.log('[DigitalTwin] Depth response:', data);
            if (data.success && data.position) {
                const pos = data.position;
                if (camPosX) camPosX.textContent = `X: ${pos.x.toFixed(3)}`;
                if (camPosY) camPosY.textContent = `Y: ${pos.y.toFixed(3)}`;
                if (camPosZ) camPosZ.textContent = `Z: ${pos.z.toFixed(3)}`;

                // Update CAM2BOT position if available
                if (data.cam2bot_position) {
                    const c2b = data.cam2bot_position;
                    if (cam2botRow) cam2botRow.style.display = 'flex';
                    if (cam2botPosX) cam2botPosX.textContent = `X: ${c2b.x.toFixed(3)}`;
                    if (cam2botPosY) cam2botPosY.textContent = `Y: ${c2b.y.toFixed(3)}`;
                    if (cam2botPosZ) cam2botPosZ.textContent = `Z: ${c2b.z.toFixed(3)}`;
                } else {
                    // No cam2bot transform available - hide the row
                    if (cam2botRow) cam2botRow.style.display = 'none';
                }
            } else {
                if (camPosX) camPosX.textContent = 'X: err';
                if (camPosY) camPosY.textContent = 'Y: err';
                if (camPosZ) camPosZ.textContent = `Z: err`;
                if (cam2botRow) cam2botRow.style.display = 'none';
                console.warn('[DigitalTwin] Depth query failed:', data.error);
            }
        };

        // Attach camera connection to socket when robot connects
        const originalOnConnect = this.connectionUI?.onConnect;
        if (this.connectionUI) {
            const self = this;
            this.connectionUI.onConnect = (config) => {
                // Call original handler
                if (originalOnConnect) {
                    originalOnConnect.call(this.connectionUI, config);
                }
                // Attach camera to socket
                if (self.robotConnection && self.robotConnection.socket) {
                    self.cameraConnection.attachSocket(self.robotConnection.socket);
                }
            };
        }

        // Detach camera when disconnected
        const originalOnDisconnect = this.connectionUI?.onDisconnect;
        if (this.connectionUI) {
            const self = this;
            this.connectionUI.onDisconnect = () => {
                if (originalOnDisconnect) {
                    originalOnDisconnect.call(this.connectionUI);
                }
                self.cameraConnection.detachSocket();
                self.cameraStreaming = false;
                // Update UI to stopped state
                if (statusIndicator) statusIndicator.classList.remove('streaming');
                if (statusText) statusText.textContent = 'Not streaming';
                if (statsContainer) statsContainer.style.display = 'none';
                if (startBtn) startBtn.style.display = 'block';
                if (stopBtn) stopBtn.style.display = 'none';
                if (cameraImage) cameraImage.style.display = 'none';
                if (cameraPlaceholder) cameraPlaceholder.style.display = 'flex';
            };
        }

        console.log('[DigitalTwin] Camera panel initialized');
    }

    /**
     * Update waypoint marker glow effects based on end effector proximity
     * @param {THREE.Vector3} endEffectorPos - Current end effector position
     */
    updateWaypointGlowEffects(endEffectorPos) {
        if (!this.waypointMarkers || this.waypointMarkers.length === 0) return;

        this.waypointMarkers.forEach(marker => {
            if (!marker.userData.isWaypointMarker) return;

            const sphereRadius = marker.userData.sphereRadius || 0.03;
            const sphereMaterial = marker.userData.sphereMaterial;
            const coreMaterial = marker.userData.coreMaterial;

            if (!sphereMaterial || !coreMaterial) return;

            // Calculate distance between end effector and waypoint marker
            const distance = endEffectorPos.distanceTo(marker.position);

            // Check if end effector is inside the dragon ball radius
            const isInside = distance < sphereRadius;

            if (isInside && !marker.userData.isGlowing) {
                // Start glowing - intense effect
                marker.userData.isGlowing = true;

                // Sphere: bright golden glow
                sphereMaterial.emissive.setHex(0xffff00);  // Bright yellow emissive
                sphereMaterial.emissiveIntensity = 2.0;  // Very intense glow
                sphereMaterial.color.setHex(0xffff88);  // Brighter color
                sphereMaterial.opacity = 0.95;
                sphereMaterial.transmission = 0.1;  // Less transmission when glowing

                // Core: extremely bright white
                coreMaterial.color.setHex(0xffffff);
                coreMaterial.opacity = 1.0;

                // Scale up the marker for extra visual impact
                marker.scale.setScalar(1.3);

            } else if (!isInside && marker.userData.isGlowing) {
                // Stop glowing - restore original values
                marker.userData.isGlowing = false;

                // Restore sphere
                sphereMaterial.emissive.setHex(0xff6600);  // Original orange emissive
                sphereMaterial.emissiveIntensity = 0.15;
                sphereMaterial.color.setHex(0xffa500);  // Original orange
                sphereMaterial.opacity = 0.65;
                sphereMaterial.transmission = 0.3;

                // Restore core
                coreMaterial.color.setHex(0xffdd00);
                coreMaterial.opacity = 0.5;

                // Restore scale
                marker.scale.setScalar(1.0);
            }
        });
    }
}

// Create and start application
const app = new DigitalTwinApp();
app.init();

// Expose to global (for debugging)
window.app = app;
