/**
 * ConnectionUI - Robot connection management UI
 * Handles connection panel, status display, and control buttons
 */

export class ConnectionUI {
    constructor(robotConnection) {
        this.robotConnection = robotConnection;
        this.onConnect = null;      // (config) => void
        this.onDisconnect = null;   // () => void
        this.onModeChanged = null;  // (mode) => void

        this.panel = null;
        this.statusIndicator = null;
        this.connectBtn = null;
        this.serverInput = null;
        this.modeButtons = {};      // mode -> button element

        // Default server URL
        this.serverUrl = 'http://localhost:5000';

        // Current control mode
        this.currentMode = 'position';
    }

    /**
     * Create and inject connection UI elements
     */
    createUI() {
        // Create connection panel
        this.createConnectionPanel();

        // Create top bar connection indicator
        this.createTopBarIndicator();

        // Setup robot connection callbacks
        this.setupCallbacks();
    }

    /**
     * Create floating connection panel
     */
    createConnectionPanel() {
        // Check if panel container exists
        let panelContainer = document.getElementById('floating-connection-panel');
        if (panelContainer) {
            // Panel exists in HTML, just setup
            this.panel = panelContainer;
            this.setupPanelElements();
            return;
        }

        // Create panel dynamically
        this.panel = document.createElement('div');
        this.panel.id = 'floating-connection-panel';
        this.panel.className = 'floating-panel';
        this.panel.innerHTML = `
            <div class="floating-panel-header">
                <span>Robot Connection</span>
                <button class="panel-close-btn" title="Close">×</button>
            </div>
            <div class="floating-panel-content">
                <div class="connection-status-section">
                    <div class="connection-status">
                        <span class="status-dot disconnected"></span>
                        <span class="status-text">Disconnected</span>
                    </div>
                    <div class="connection-mode"></div>
                </div>

                <div class="connection-form">
                    <label for="server-url">Server URL</label>
                    <input type="text" id="server-url" value="http://localhost:5000" placeholder="http://localhost:5000">
                </div>

                <div class="connection-buttons">
                    <button id="connect-btn" class="control-button primary">Connect</button>
                    <button id="disconnect-btn" class="control-button" disabled>Disconnect</button>
                </div>

                <div class="robot-info" style="display: none;">
                    <div class="info-row">
                        <span class="info-label">Robot:</span>
                        <span class="info-value" id="robot-name">-</span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">Joints:</span>
                        <span class="info-value" id="joint-count">-</span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">Mode:</span>
                        <span class="info-value" id="connection-mode">-</span>
                    </div>
                </div>

                <div class="robot-controls" style="display: none;">
                    <div class="control-mode-section">
                        <label class="section-label">Control Mode</label>
                        <div class="mode-buttons">
                            <button id="mode-position" class="mode-button active" data-mode="position">
                                <span class="mode-name">Position</span>
                            </button>
                            <button id="mode-gravity" class="mode-button" data-mode="gravity_comp">
                                <span class="mode-name">Gravity</span>
                            </button>
                            <button id="mode-impedance" class="mode-button" data-mode="impedance">
                                <span class="mode-name">Impedance</span>
                            </button>
                        </div>
                        <div class="mode-description" id="mode-description">
                            Direct joint position control via sliders
                        </div>
                    </div>

                    <div class="control-grid">
                        <button id="home-btn" class="control-button">Home</button>
                        <button id="stop-btn" class="control-button danger">Stop</button>
                        <button id="set-zero-btn" class="control-button warning" title="Set current position as zero reference">Set Zero</button>
                    </div>

                    <div class="velocity-control">
                        <label for="velocity-slider">Velocity: <span id="velocity-value">0.5</span> rad/s</label>
                        <input type="range" id="velocity-slider" min="0.1" max="2.0" step="0.1" value="0.5">
                    </div>
                </div>
            </div>
        `;

        document.body.appendChild(this.panel);
        this.setupPanelElements();
    }

    /**
     * Setup panel element references and events
     */
    setupPanelElements() {
        this.serverInput = this.panel.querySelector('#server-url');
        this.connectBtn = this.panel.querySelector('#connect-btn');
        const disconnectBtn = this.panel.querySelector('#disconnect-btn');
        const homeBtn = this.panel.querySelector('#home-btn');
        const stopBtn = this.panel.querySelector('#stop-btn');
        const setZeroBtn = this.panel.querySelector('#set-zero-btn');
        const velocitySlider = this.panel.querySelector('#velocity-slider');
        const closeBtn = this.panel.querySelector('.panel-close-btn');

        // Connect button
        if (this.connectBtn) {
            this.connectBtn.addEventListener('click', () => this.handleConnect());
        }

        // Disconnect button
        if (disconnectBtn) {
            disconnectBtn.addEventListener('click', () => this.handleDisconnect());
        }

        // Home button
        if (homeBtn) {
            homeBtn.addEventListener('click', () => {
                if (this.robotConnection.isConnected()) {
                    this.robotConnection.home();
                }
            });
        }

        // Stop button
        if (stopBtn) {
            stopBtn.addEventListener('click', () => {
                if (this.robotConnection.isConnected()) {
                    this.robotConnection.stop();
                }
            });
        }

        // Set Zero button
        if (setZeroBtn) {
            setZeroBtn.addEventListener('click', () => {
                if (this.robotConnection.isConnected()) {
                    // Show confirmation dialog
                    if (confirm('Set current position as zero reference?\n\nThis will reset all encoder positions to 0.')) {
                        this.robotConnection.setZero();
                    }
                }
            });
        }

        // Velocity slider
        if (velocitySlider) {
            velocitySlider.addEventListener('input', (e) => {
                const value = parseFloat(e.target.value);
                const valueDisplay = this.panel.querySelector('#velocity-value');
                if (valueDisplay) valueDisplay.textContent = value.toFixed(1);

                // Send velocity update via REST
                if (this.robotConnection.isConnected()) {
                    fetch(`${this.serverUrl}/api/set_velocity`, {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({ velocity: value })
                    }).catch(console.error);
                }
            });
        }

        // Close button
        if (closeBtn) {
            closeBtn.addEventListener('click', () => this.hidePanel());
        }

        // Mode buttons
        this.setupModeButtons();
    }

    /**
     * Setup mode button event handlers
     */
    setupModeButtons() {
        const modeDescriptions = {
            'position': 'Direct joint position control via sliders',
            'gravity_comp': 'Robot floats freely - only gravity compensation',
            'impedance': 'Compliant control - robot moves to target with spring-like behavior'
        };

        const modeButtons = this.panel?.querySelectorAll('.mode-button');
        if (!modeButtons) return;

        modeButtons.forEach(btn => {
            const mode = btn.getAttribute('data-mode');
            this.modeButtons[mode] = btn;

            btn.addEventListener('click', () => {
                if (!this.robotConnection.isConnected()) return;

                // Update UI
                this.setActiveMode(mode);

                // Send to robot
                this.robotConnection.setMode(mode);

                // Callback
                if (this.onModeChanged) this.onModeChanged(mode);
            });
        });
    }

    /**
     * Set active mode in UI
     */
    setActiveMode(mode) {
        this.currentMode = mode;

        const modeDescriptions = {
            'position': 'Direct joint position control via sliders',
            'gravity_comp': 'Robot floats freely - only gravity compensation',
            'impedance': 'Compliant control - robot moves to target with spring-like behavior'
        };

        // Update button states
        Object.entries(this.modeButtons).forEach(([m, btn]) => {
            if (m === mode) {
                btn.classList.add('active');
            } else {
                btn.classList.remove('active');
            }
        });

        // Update description
        const descEl = this.panel?.querySelector('#mode-description');
        if (descEl) {
            descEl.textContent = modeDescriptions[mode] || '';
        }
    }

    /**
     * Create top bar connection indicator
     */
    createTopBarIndicator() {
        const topBar = document.getElementById('top-control-bar');
        if (!topBar) return;

        // Check if indicator already exists
        if (document.getElementById('connection-indicator')) return;

        // Create divider
        const divider = document.createElement('div');
        divider.className = 'control-bar-divider';

        // Create indicator button
        this.statusIndicator = document.createElement('button');
        this.statusIndicator.id = 'connection-indicator';
        this.statusIndicator.className = 'tool-button';
        this.statusIndicator.innerHTML = `
            <span class="status-dot-small disconnected"></span>
            <span class="connection-label">Offline</span>
        `;
        this.statusIndicator.title = 'Robot Connection';
        this.statusIndicator.addEventListener('click', () => this.togglePanel());

        // Insert at the beginning of top bar
        topBar.insertBefore(this.statusIndicator, topBar.firstChild);
        topBar.insertBefore(divider, this.statusIndicator.nextSibling);
    }

    /**
     * Setup robot connection callbacks
     */
    setupCallbacks() {
        this.robotConnection.onConnected = () => {
            this.updateConnectionStatus(true);
        };

        this.robotConnection.onDisconnected = () => {
            this.updateConnectionStatus(false);
            if (this.onDisconnect) this.onDisconnect();
        };

        this.robotConnection.onConfigReceived = (config) => {
            this.updateRobotInfo(config);
            // Set initial mode from config
            if (config.control_mode) {
                this.setActiveMode(config.control_mode);
            }
            if (this.onConnect) this.onConnect(config);
        };

        this.robotConnection.onError = (error) => {
            this.showError(error.message || 'Connection failed');
        };

        this.robotConnection.onModeChanged = (mode) => {
            this.setActiveMode(mode);
            if (this.onModeChanged) this.onModeChanged(mode);
        };
    }

    /**
     * Handle connect button click
     */
    async handleConnect() {
        const url = this.serverInput ? this.serverInput.value.trim() : this.serverUrl;
        if (!url) {
            this.showError('Please enter server URL');
            return;
        }

        this.serverUrl = url;
        this.setConnecting(true);

        try {
            await this.robotConnection.connect(url);
        } catch (error) {
            this.showError(error.message || 'Failed to connect');
            this.setConnecting(false);
        }
    }

    /**
     * Handle disconnect button click
     */
    handleDisconnect() {
        this.robotConnection.disconnect();
        this.updateConnectionStatus(false);
    }

    /**
     * Update connection status display
     */
    updateConnectionStatus(connected) {
        // Update panel status
        const statusDot = this.panel?.querySelector('.status-dot');
        const statusText = this.panel?.querySelector('.status-text');
        const connectBtn = this.panel?.querySelector('#connect-btn');
        const disconnectBtn = this.panel?.querySelector('#disconnect-btn');
        const robotInfo = this.panel?.querySelector('.robot-info');
        const robotControls = this.panel?.querySelector('.robot-controls');

        if (statusDot) {
            statusDot.className = `status-dot ${connected ? 'connected' : 'disconnected'}`;
        }
        if (statusText) {
            statusText.textContent = connected ? 'Connected' : 'Disconnected';
        }
        if (connectBtn) {
            connectBtn.disabled = connected;
            connectBtn.textContent = 'Connect';
        }
        if (disconnectBtn) {
            disconnectBtn.disabled = !connected;
        }
        if (robotInfo) {
            robotInfo.style.display = connected ? 'block' : 'none';
        }
        if (robotControls) {
            robotControls.style.display = connected ? 'block' : 'none';
        }

        // Update top bar indicator
        if (this.statusIndicator) {
            const dot = this.statusIndicator.querySelector('.status-dot-small');
            const label = this.statusIndicator.querySelector('.connection-label');

            if (dot) {
                dot.className = `status-dot-small ${connected ? 'connected' : 'disconnected'}`;
            }
            if (label) {
                label.textContent = connected ? 'Connected' : 'Offline';
            }

            this.statusIndicator.classList.toggle('active', connected);
        }
    }

    /**
     * Update robot info display
     */
    updateRobotInfo(config) {
        if (!config) return;

        const robotName = this.panel?.querySelector('#robot-name');
        const jointCount = this.panel?.querySelector('#joint-count');
        const connectionMode = this.panel?.querySelector('#connection-mode');

        if (robotName) {
            robotName.textContent = config.robot_name || 'Unknown';
        }
        if (jointCount) {
            jointCount.textContent = config.joints?.length || '-';
        }
        if (connectionMode) {
            connectionMode.textContent = config.demo_mode ? 'Demo Mode' : 'Live Robot';
            connectionMode.className = `info-value ${config.demo_mode ? 'demo' : 'live'}`;
        }
    }

    /**
     * Show connecting state
     */
    setConnecting(connecting) {
        if (this.connectBtn) {
            this.connectBtn.disabled = connecting;
            this.connectBtn.textContent = connecting ? 'Connecting...' : 'Connect';
        }
    }

    /**
     * Show error message
     */
    showError(message) {
        console.error('[ConnectionUI]', message);

        // Show toast notification
        const toast = document.createElement('div');
        toast.className = 'error-toast';
        toast.textContent = message;
        document.body.appendChild(toast);

        setTimeout(() => {
            toast.classList.add('fade-out');
            setTimeout(() => toast.remove(), 300);
        }, 3000);
    }

    /**
     * Toggle panel visibility
     */
    togglePanel() {
        if (this.panel) {
            this.panel.classList.toggle('hidden');
        }
    }

    /**
     * Show panel
     */
    showPanel() {
        if (this.panel) {
            this.panel.classList.remove('hidden');
        }
    }

    /**
     * Hide panel
     */
    hidePanel() {
        if (this.panel) {
            this.panel.classList.add('hidden');
        }
    }

    /**
     * Check if connected
     */
    isConnected() {
        return this.robotConnection.isConnected();
    }
}
