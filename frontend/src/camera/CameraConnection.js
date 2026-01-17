/**
 * CameraConnection - Handles camera WebSocket events
 *
 * Uses the existing socket connection from RobotConnection to handle:
 * - Camera start/stop commands
 * - Image streaming (mono or depth)
 * - Camera status and mode updates
 * - Depth query for 3D position measurement
 */

export class CameraConnection {
    constructor() {
        this.socket = null
        this.streaming = false
        this.config = null
        this.mode = 'mono'  // 'mono' or 'depth'

        // Callbacks
        this.onFrame = null             // (data) => void - { image, width, height, mode, fps, latency_ms }
        this.onStatusChange = null      // (streaming, config) => void
        this.onModeChange = null        // (mode) => void
        this.onDepthResponse = null     // (data) => void - { success, position: {x,y,z}, depth_range, box }
        this.onError = null             // (error) => void

        // Stats
        this.lastFrameTime = 0
        this.frameCount = 0
        this.actualFps = 0
    }

    /**
     * Attach to an existing socket connection
     * @param {Socket} socket - Socket.IO socket from RobotConnection
     */
    attachSocket(socket) {
        if (!socket) {
            console.error('[CameraConnection] No socket provided')
            return
        }

        this.socket = socket
        this._setupEventHandlers()
        console.log('[CameraConnection] Attached to socket')
    }

    /**
     * Detach from socket (cleanup)
     */
    detachSocket() {
        if (this.socket) {
            this.socket.off('camera_frame')
            this.socket.off('camera_status')
            this.socket.off('camera_started')
            this.socket.off('camera_stopped')
            this.socket.off('camera_config_response')
            this.socket.off('camera_mode_response')
            this.socket.off('camera_mode_changed')
            this.socket.off('camera_depth_response')
        }
        this.socket = null
        this.streaming = false
    }

    _setupEventHandlers() {
        // Camera frame received (base64 JPEG image)
        this.socket.on('camera_frame', (data) => {
            this._updateStats()

            if (this.onFrame) {
                this.onFrame({
                    image: data.image,
                    width: data.width,
                    height: data.height,
                    mode: data.mode,
                    fps: data.fps,
                    latency_ms: data.latency_ms,
                    frame_count: data.frame_count,
                    clientFps: this.actualFps
                })
            }
        })

        // Camera status update
        this.socket.on('camera_status', (data) => {
            console.log('[CameraConnection] Status:', data)
            this.streaming = data.streaming
            if (data.config) {
                this.config = data.config
                if (data.config.mode) {
                    this.mode = data.config.mode
                }
            }
            if (this.onStatusChange) {
                this.onStatusChange(this.streaming, this.config, data.message || data.error)
            }
        })

        // Camera started event (broadcast to all clients)
        this.socket.on('camera_started', (config) => {
            console.log('[CameraConnection] Camera started:', config)
            this.streaming = true
            this.config = config
            if (config.mode) {
                this.mode = config.mode
            }
            if (this.onStatusChange) {
                this.onStatusChange(true, config, 'Camera started')
            }
        })

        // Camera stopped event (broadcast to all clients)
        this.socket.on('camera_stopped', () => {
            console.log('[CameraConnection] Camera stopped')
            this.streaming = false
            if (this.onStatusChange) {
                this.onStatusChange(false, this.config, 'Camera stopped')
            }
        })

        // Camera config response
        this.socket.on('camera_config_response', (config) => {
            console.log('[CameraConnection] Config:', config)
            this.config = config
            if (config.mode) {
                this.mode = config.mode
            }
        })

        // Camera mode change response
        this.socket.on('camera_mode_response', (data) => {
            console.log('[CameraConnection] Mode response:', data)
            if (data.success && data.mode) {
                this.mode = data.mode
            }
        })

        // Camera mode changed broadcast
        this.socket.on('camera_mode_changed', (data) => {
            console.log('[CameraConnection] Mode changed:', data.mode)
            this.mode = data.mode
            if (this.onModeChange) {
                this.onModeChange(data.mode)
            }
        })

        // Depth query response
        this.socket.on('camera_depth_response', (data) => {
            console.log('[CameraConnection] Depth response:', data)
            if (this.onDepthResponse) {
                this.onDepthResponse(data)
            }
        })
    }

    _updateStats() {
        const now = performance.now()
        if (this.lastFrameTime > 0) {
            const dt = (now - this.lastFrameTime) / 1000
            if (dt > 0) {
                this.actualFps = 0.1 * this.actualFps + 0.9 * (1 / dt)
            }
        }
        this.lastFrameTime = now
        this.frameCount++
    }

    /**
     * Start camera streaming
     * @param {Object} options - { resolution: [w, h], fps: number, downsample: number }
     */
    startCamera(options = {}) {
        if (!this.socket) {
            console.error('[CameraConnection] Not connected')
            return
        }

        console.log('[CameraConnection] Starting camera with options:', options)
        this.socket.emit('camera_start', options)
    }

    /**
     * Stop camera streaming
     */
    stopCamera() {
        if (!this.socket) {
            console.error('[CameraConnection] Not connected')
            return
        }

        console.log('[CameraConnection] Stopping camera')
        this.socket.emit('camera_stop')
    }

    /**
     * Set camera mode (mono or depth)
     * @param {string} mode - 'mono' or 'depth'
     */
    setMode(mode) {
        if (!this.socket) {
            console.error('[CameraConnection] Not connected')
            return
        }

        console.log('[CameraConnection] Setting mode:', mode)
        this.socket.emit('camera_mode', { mode })
    }

    /**
     * Query depth at a selected region
     * @param {Array} box - [x1, y1, x2, y2] in image coordinates
     */
    queryDepth(box) {
        if (!this.socket) {
            console.error('[CameraConnection] Not connected')
            return
        }

        console.log('[CameraConnection] Querying depth at box:', box)
        this.socket.emit('camera_depth_query', { box })
    }

    /**
     * Request camera configuration
     */
    requestConfig() {
        if (!this.socket) return
        this.socket.emit('camera_config')
    }

    /**
     * Check if camera is currently streaming
     */
    isStreaming() {
        return this.streaming
    }

    /**
     * Get current mode
     */
    getMode() {
        return this.mode
    }

    /**
     * Get current config
     */
    getConfig() {
        return this.config
    }
}
