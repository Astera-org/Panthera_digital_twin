/**
 * URDF Adapter
 * Converts urdf-loaders result to unified model
 */
import * as THREE from 'three';
import { UnifiedRobotModel, Link, Joint, JointLimits, VisualGeometry, CollisionGeometry, InertialProperties } from '../models/UnifiedRobotModel.js';

export class URDFAdapter {
    /**
     * Convert urdf-loaders robot object to unified model
     * @param {THREE.Group} robot - Robot object returned by urdf-loaders
     * @param {string} urdfXML - Original URDF XML content (optional, for extracting inertial data)
     * @returns {UnifiedRobotModel}
     */
    static convert(robot, urdfXML = null) {
        const model = new UnifiedRobotModel();
        model.name = robot.name || 'robot';
        model.threeObject = robot;

        // Mark model type as URDF
        if (!robot.userData) robot.userData = {};
        robot.userData.type = 'urdf';

        if (!robot.links || !robot.joints) {
            console.warn('URDF model missing links or joints information');
            return model;
        }

        // If XML provided, parse inertial data
        let inertialData = {};
        if (urdfXML) {
            inertialData = this.parseInertialFromXML(urdfXML);
        }

        // Convert links
        Object.values(robot.links).forEach(urdfLink => {
            const link = this.convertLink(urdfLink);

            // If urdf-loader didn't parse inertial, get from XML
            if (!link.inertial && inertialData[urdfLink.name]) {
                link.inertial = inertialData[urdfLink.name];
            }

            model.addLink(link);
        });

        // Convert joints
        Object.values(robot.joints).forEach(urdfJoint => {
            const joint = this.convertJoint(urdfJoint);
            model.addJoint(joint);
        });

        // If XML provided, supplement effort and velocity from XML (urdf-loaders may not have parsed)
        if (urdfXML) {
            this.supplementJointLimitsFromXML(model, urdfXML);
        }

        // Find root link (link that is not a child of any joint)
        const allChildren = new Set(
            Array.from(model.joints.values()).map(j => j.child).filter(c => c)
        );
        const rootLinks = Array.from(model.links.keys()).filter(
            name => !allChildren.has(name)
        );
        if (rootLinks.length > 0) {
            model.rootLink = rootLinks[0];
        }

        // Enhance materials for better lighting (MuJoCo style)
        this.enhanceMaterials(robot);

        return model;
    }

    /**
     * Enhance materials for better lighting (MuJoCo style)
     * Applies consistent shininess and specular properties to all materials
     * Directly modifies materials in place to ensure changes persist
     */
    static enhanceMaterials(robotObject) {
        robotObject.traverse((child) => {
            if (child.isMesh && child.material) {
                // Handle material arrays
                if (Array.isArray(child.material)) {
                    child.material = child.material.map(mat => {
                        return this.enhanceSingleMaterial(mat);
                    });
                } else {
                    child.material = this.enhanceSingleMaterial(child.material);
                }
            }
        });
    }

    /**
     * Enhance a single material with better lighting properties
     * Returns enhanced material (may be cloned or modified in place)
     * Saves original properties for lighting toggle
     */
    static enhanceSingleMaterial(material) {
        if (material.isMeshPhongMaterial || material.isMeshStandardMaterial) {
            // Save original properties if not already saved (for lighting toggle)
            if (material.userData.originalShininess === undefined) {
                material.userData.originalShininess = material.shininess !== undefined ? material.shininess : 30;
                // IMPORTANT: Save original color to preserve URDF material colors
                if (!material.userData.originalColor && material.color) {
                    material.userData.originalColor = material.color.clone();
                }
                // Save original specular - if material had no specular, save null
                if (!material.specular) {
                    material.userData.originalSpecular = null;
                } else if (material.specular.isColor) {
                    const spec = material.specular;
                    if (spec.r < 0.1 && spec.g < 0.1 && spec.b < 0.1) {
                        material.userData.originalSpecular = null; // Likely default
                    } else {
                        material.userData.originalSpecular = spec.clone();
                    }
                } else if (typeof material.specular === 'number') {
                    if (material.specular === 0x111111 || material.specular < 0x111111) {
                        material.userData.originalSpecular = null;
                    } else {
                        material.userData.originalSpecular = new THREE.Color(material.specular);
                    }
                } else {
                    material.userData.originalSpecular = null;
                }
            }

            // IMPORTANT: Preserve original color from URDF material definitions
            // Do not modify material.color - urdf-loaders already sets the correct color

            // Apply enhanced lighting (default enabled)
            // Increase shininess for better highlights
            if (material.shininess === undefined || material.shininess < 50) {
                material.shininess = 50;
            }

            // Enhance specular reflection - ensure it's a Color object with proper values
            if (!material.specular) {
                material.specular = new THREE.Color(0.3, 0.3, 0.3);
            } else if (material.specular.isColor) {
                // If it's already a Color object, update values
                if (material.specular.r < 0.2 || material.specular.g < 0.2 || material.specular.b < 0.2) {
                    material.specular.setRGB(0.3, 0.3, 0.3);
                }
            } else if (typeof material.specular === 'number') {
                // Convert number to Color object
                if (material.specular < 0x333333) {
                    material.specular = new THREE.Color(0.3, 0.3, 0.3);
                } else {
                    // Convert hex to Color
                    material.specular = new THREE.Color(material.specular);
                    if (material.specular.r < 0.2) {
                        material.specular.setRGB(0.3, 0.3, 0.3);
                    }
                }
            }

            // Mark material as needing update
            material.needsUpdate = true;
            return material;
        } else if (material.type === 'MeshBasicMaterial') {
            // Convert MeshBasicMaterial to MeshPhongMaterial
            const oldMaterial = material;
            const envMap = typeof window !== 'undefined' && window.app?.sceneManager?.environmentManager?.getEnvironmentMap();
            const newMaterial = new THREE.MeshPhongMaterial({
                color: oldMaterial.color,
                map: oldMaterial.map,
                transparent: oldMaterial.transparent,
                opacity: oldMaterial.opacity,
                side: oldMaterial.side,
                shininess: 50,
                specular: new THREE.Color(0.3, 0.3, 0.3),
                envMap: envMap || null,
                reflectivity: envMap ? 0.3 : 0
            });
            // Save original properties for lighting toggle
            newMaterial.userData.originalShininess = 30;
            newMaterial.userData.originalSpecular = null; // MeshBasicMaterial had no specular
            return newMaterial;
        }

        return material;
    }

    static convertLink(urdfLink) {
        const link = new Link(urdfLink.name);
        link.threeObject = urdfLink;

        // Convert inertial properties
        if (urdfLink.inertial) {
            link.inertial = this.convertInertial(urdfLink.inertial);
        }

        // Note: urdf-loaders has already converted visual and collision to Three.js objects
        // We mainly extract metadata, actual meshes are in threeObject

        return link;
    }

    static convertJoint(urdfJoint) {
        // URDF joint object has jointType property (not type)
        const jointType = urdfJoint.jointType || urdfJoint.type || 'fixed';
        const joint = new Joint(urdfJoint.name, jointType);

        // Extract parent name (urdf-loader returns Three.js object reference)
        joint.parent = urdfJoint.parent?.name || null;

        // child may not be in urdfJoint.child, but in Three.js children array
        // In Three.js scene graph, Joint is parent node, Child Link is Joint's child node
        if (urdfJoint.child && urdfJoint.child.name) {
            joint.child = urdfJoint.child.name;
        } else if (urdfJoint.children && urdfJoint.children.length > 0) {
            // Find Link object from children array
            const childLink = urdfJoint.children.find(child =>
                child.isURDFLink || child.type === 'URDFLink'
            );
            if (childLink) {
                joint.child = childLink.name;
            }
        }

        joint.threeObject = urdfJoint;

        // Convert origin
        if (urdfJoint.origin) {
            joint.origin = {
                xyz: urdfJoint.origin.xyz || [0, 0, 0],
                rpy: urdfJoint.origin.rpy || [0, 0, 0]
            };
        }

        // Convert axis
        if (urdfJoint.axis) {
            joint.axis = { xyz: urdfJoint.axis.xyz || [0, 0, 1] };
        }

        // Convert limits
        if (urdfJoint.limit) {
            joint.limits = this.convertLimits(urdfJoint.limit);
        }

        // Get current value
        if (urdfJoint.angle !== undefined) {
            joint.currentValue = urdfJoint.angle;
        } else if (urdfJoint.jointValue !== undefined) {
            joint.currentValue = urdfJoint.jointValue;
        }

        return joint;
    }

    static convertLimits(limit) {
        const limits = new JointLimits();
        if (limit.lower !== undefined) limits.lower = limit.lower;
        if (limit.upper !== undefined) limits.upper = limit.upper;
        if (limit.effort !== undefined) limits.effort = limit.effort;
        if (limit.velocity !== undefined) limits.velocity = limit.velocity;
        return limits;
    }

    /**
     * Supplement joint effort and velocity information from URDF XML
     * (because urdf-loaders may not have parsed these attributes)
     */
    static supplementJointLimitsFromXML(model, urdfXML) {
        try {
            const parser = new DOMParser();
            const doc = parser.parseFromString(urdfXML, 'text/xml');

            const jointElements = doc.querySelectorAll('joint');
            jointElements.forEach(jointEl => {
                const jointName = jointEl.getAttribute('name');
                const joint = model.joints.get(jointName);

                if (joint) {
                    const limitEl = jointEl.querySelector('limit');
                    if (limitEl) {
                        if (!joint.limits) {
                            joint.limits = new JointLimits();
                        }

                        // Read effort and velocity
                        const effort = limitEl.getAttribute('effort');
                        const velocity = limitEl.getAttribute('velocity');

                        if (effort !== null) {
                            joint.limits.effort = parseFloat(effort);
                        }
                        if (velocity !== null) {
                            joint.limits.velocity = parseFloat(velocity);
                        }

                        // Also ensure lower and upper are correctly set
                        const lower = limitEl.getAttribute('lower');
                        const upper = limitEl.getAttribute('upper');
                        if (lower !== null) {
                            joint.limits.lower = parseFloat(lower);
                        }
                        if (upper !== null) {
                            joint.limits.upper = parseFloat(upper);
                        }
                    }
                }
            });
        } catch (error) {
            console.error('Failed to supplement joint limit information from XML:', error);
        }
    }

    static convertInertial(inertial) {
        const props = new InertialProperties();
        if (inertial.mass !== undefined) props.mass = inertial.mass;
        if (inertial.origin) {
            props.origin = {
                xyz: inertial.origin.xyz || [0, 0, 0],
                rpy: inertial.origin.rpy || [0, 0, 0]
            };
        }
        if (inertial.ixx !== undefined) props.ixx = inertial.ixx;
        if (inertial.iyy !== undefined) props.iyy = inertial.iyy;
        if (inertial.izz !== undefined) props.izz = inertial.izz;
        if (inertial.ixy !== undefined) props.ixy = inertial.ixy;
        if (inertial.ixz !== undefined) props.ixz = inertial.ixz;
        if (inertial.iyz !== undefined) props.iyz = inertial.iyz;
        return props;
    }

    /**
     * Parse inertial data from URDF XML
     * @param {string} xmlContent - URDF XML content
     * @returns {Object} Mapping from link names to inertial data
     */
    static parseInertialFromXML(xmlContent) {
        const inertialData = {};

        try {
            const parser = new DOMParser();
            const xmlDoc = parser.parseFromString(xmlContent, 'text/xml');

            // Get all link elements
            const links = xmlDoc.getElementsByTagName('link');

            for (let i = 0; i < links.length; i++) {
                const linkElement = links[i];
                const linkName = linkElement.getAttribute('name');

                // Find inertial element
                const inertialElement = linkElement.getElementsByTagName('inertial')[0];
                if (!inertialElement) continue;

                const inertial = new InertialProperties();

                // Parse mass
                const massElement = inertialElement.getElementsByTagName('mass')[0];
                if (massElement) {
                    inertial.mass = parseFloat(massElement.getAttribute('value'));
                }

                // Parse origin
                const originElement = inertialElement.getElementsByTagName('origin')[0];
                if (originElement) {
                    const xyz = originElement.getAttribute('xyz');
                    const rpy = originElement.getAttribute('rpy');
                    inertial.origin = {
                        xyz: xyz ? xyz.split(' ').map(parseFloat) : [0, 0, 0],
                        rpy: rpy ? rpy.split(' ').map(parseFloat) : [0, 0, 0]
                    };
                } else {
                    inertial.origin = { xyz: [0, 0, 0], rpy: [0, 0, 0] };
                }

                // Parse inertia
                const inertiaElement = inertialElement.getElementsByTagName('inertia')[0];
                if (inertiaElement) {
                    inertial.ixx = parseFloat(inertiaElement.getAttribute('ixx')) || 0;
                    inertial.iyy = parseFloat(inertiaElement.getAttribute('iyy')) || 0;
                    inertial.izz = parseFloat(inertiaElement.getAttribute('izz')) || 0;
                    inertial.ixy = parseFloat(inertiaElement.getAttribute('ixy')) || 0;
                    inertial.ixz = parseFloat(inertiaElement.getAttribute('ixz')) || 0;
                    inertial.iyz = parseFloat(inertiaElement.getAttribute('iyz')) || 0;
                }

                inertialData[linkName] = inertial;
            }
        } catch (error) {
            console.error('Failed to parse URDF XML inertial data:', error);
        }

        return inertialData;
    }

    /**
     * Set joint angle (using urdf-loaders' setJointValue method)
     * Reference URDFClasses.js setJointValue implementation
     */
    static setJointAngle(joint, angle, ignoreLimits = false) {
        joint.currentValue = angle;

        // URDF format: use urdf-loader's setJointValue method
        if (joint.threeObject) {
            // If ignoring limits, temporarily modify URDF joint object's limit values
            let originalLimits = null;
            if (ignoreLimits && joint.threeObject.limit) {
                originalLimits = {
                    lower: joint.threeObject.limit.lower,
                    upper: joint.threeObject.limit.upper
                };
                joint.threeObject.limit.lower = -Math.PI * 2;
                joint.threeObject.limit.upper = Math.PI * 2;
            }

            // Prefer setJointValue method (urdf-loader's standard method)
            if (typeof joint.threeObject.setJointValue === 'function') {
                joint.threeObject.setJointValue(angle);

                // Restore original limits
                if (originalLimits && joint.threeObject.limit) {
                    joint.threeObject.limit.lower = originalLimits.lower;
                    joint.threeObject.limit.upper = originalLimits.upper;
                }
                return;
            } else if (typeof joint.threeObject.setAngle === 'function') {
                joint.threeObject.setAngle(angle);

                // Restore original limits
                if (originalLimits && joint.threeObject.limit) {
                    joint.threeObject.limit.lower = originalLimits.lower;
                    joint.threeObject.limit.upper = originalLimits.upper;
                }
                return;
            }

            // Restore original limits (if none of the above methods executed)
            if (originalLimits && joint.threeObject.limit) {
                joint.threeObject.limit.lower = originalLimits.lower;
                joint.threeObject.limit.upper = originalLimits.upper;
            }

            // If none available, manually set rotation
            // Note: For URDF, we usually shouldn't reach here, as urdf-loader should provide setJointValue
            console.warn(`Joint ${joint.name} has no setJointValue or setAngle method, attempting manual setup`);
            if (joint.type === 'revolute' || joint.type === 'continuous') {
                const axis = joint.axis ? new THREE.Vector3(...joint.axis.xyz).normalize() : new THREE.Vector3(0, 0, 1);
                // Note: joint.threeObject may not be a Three.js object, cannot directly set rotation
                if (joint.threeObject.rotation) {
                    joint.threeObject.rotation.setFromAxisAngle(axis, angle);
                }
            } else if (joint.type === 'prismatic') {
                const axis = joint.axis ? new THREE.Vector3(...joint.axis.xyz).normalize() : new THREE.Vector3(1, 0, 0);
                if (joint.threeObject.position) {
                    joint.threeObject.position.copy(axis.multiplyScalar(angle));
                }
            }
        } else {
            console.warn(`Joint ${joint.name} has no threeObject`);
        }
    }
}

