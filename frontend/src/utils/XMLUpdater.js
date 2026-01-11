/**
 * XMLUpdater - XML update utility
 * Used for updating URDF/MJCF XML content
 */

export class XMLUpdater {
    /**
     * Update URDF joint limit attributes
     * @param {string} xmlContent - Original XML content
     * @param {string} jointName - Joint name
     * @param {Object} limits - New limit values { lower, upper, effort, velocity }
     * @returns {string} Updated XML content
     */
    static updateURDFJointLimits(xmlContent, jointName, limits) {
        try {
            // Use regex to find joint definition
            const jointRegex = new RegExp(
                `<joint[^>]*name="${jointName.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}"[^>]*>([\\s\\S]*?)</joint>`,
                'g'
            );

            const match = jointRegex.exec(xmlContent);
            if (!match) {
                console.warn(`Joint not found: ${jointName}`);
                return xmlContent;
            }

            const jointContent = match[0];
            let updatedJointContent = jointContent;

            // Find limit tag
            const limitRegex = /<limit([^>]*)>/;
            const limitMatch = limitRegex.exec(jointContent);

            if (limitMatch) {
                // Limit tag exists, update attributes
                let limitTag = limitMatch[0];
                const limitAttrs = limitMatch[1];

                // Update each attribute
                if (limits.lower !== undefined && limits.lower !== null) {
                    if (limitAttrs.includes('lower=')) {
                        limitTag = limitTag.replace(/lower="[^"]*"/, `lower="${limits.lower}"`);
                    } else {
                        limitTag = limitTag.replace(/>$/, ` lower="${limits.lower}">`);
                    }
                }

                if (limits.upper !== undefined && limits.upper !== null) {
                    if (limitAttrs.includes('upper=')) {
                        limitTag = limitTag.replace(/upper="[^"]*"/, `upper="${limits.upper}"`);
                    } else {
                        limitTag = limitTag.replace(/>$/, ` upper="${limits.upper}">`);
                    }
                }

                if (limits.effort !== undefined && limits.effort !== null) {
                    if (limitAttrs.includes('effort=')) {
                        limitTag = limitTag.replace(/effort="[^"]*"/, `effort="${limits.effort}"`);
                    } else {
                        limitTag = limitTag.replace(/>$/, ` effort="${limits.effort}">`);
                    }
                }

                if (limits.velocity !== undefined && limits.velocity !== null) {
                    if (limitAttrs.includes('velocity=')) {
                        limitTag = limitTag.replace(/velocity="[^"]*"/, `velocity="${limits.velocity}"`);
                    } else {
                        limitTag = limitTag.replace(/>$/, ` velocity="${limits.velocity}">`);
                    }
                }

                updatedJointContent = jointContent.replace(limitRegex, limitTag);
            } else {
                // No limit tag, create one
                const attrs = [];
                if (limits.lower !== undefined && limits.lower !== null) attrs.push(`lower="${limits.lower}"`);
                if (limits.upper !== undefined && limits.upper !== null) attrs.push(`upper="${limits.upper}"`);
                if (limits.effort !== undefined && limits.effort !== null) attrs.push(`effort="${limits.effort}"`);
                if (limits.velocity !== undefined && limits.velocity !== null) attrs.push(`velocity="${limits.velocity}"`);

                if (attrs.length > 0) {
                    const limitTag = `    <limit ${attrs.join(' ')}/>`;
                    // Insert before </joint>
                    updatedJointContent = jointContent.replace('</joint>', `${limitTag}\n  </joint>`);
                }
            }

            // Replace joint content in original XML
            return xmlContent.replace(jointContent, updatedJointContent);

        } catch (error) {
            console.error('Failed to update URDF joint limits:', error);
            return xmlContent;
        }
    }

    /**
     * Batch update multiple joint limits
     * @param {string} xmlContent - Original XML content
     * @param {Map} jointsLimits - Map<jointName, limits>
     * @returns {string} Updated XML content
     */
    static updateMultipleJointLimits(xmlContent, jointsLimits) {
        let updatedXML = xmlContent;

        for (let [jointName, limits] of jointsLimits.entries()) {
            updatedXML = this.updateURDFJointLimits(updatedXML, jointName, limits);
        }

        return updatedXML;
    }
}

