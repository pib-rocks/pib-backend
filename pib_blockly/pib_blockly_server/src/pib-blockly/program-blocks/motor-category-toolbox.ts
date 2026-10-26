/** Blockly toolbox block types for the Motoric skills category (colour 355). */
export const MOTOR_CATEGORY_TOOLBOX_BLOCK_TYPES: readonly string[] = [
    "move_motor",
    "move_motors",
    "move_to_pose",
    "save_current_pose",
    "save_detection_as_pose",
    "set_hand_position_xyz",
    "get_all_poses",
    "get_pose_joints",
    "has_pose",
    "pose_count",
    "play_pose_sequence",
    "motor_current",
    "get_hand_x",
    "get_hand_y",
    "get_hand_z",
];

export function motorCategoryToolboxXml(): string {
    return MOTOR_CATEGORY_TOOLBOX_BLOCK_TYPES
        .map((type) => `<block type="${type}"></block>`)
        .join("\n");
}
