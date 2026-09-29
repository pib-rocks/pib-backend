import * as Blockly from "blockly";

export const imu_blocks = Blockly.common.createBlockDefinitionsFromJsonArray([
    {
        type: "imu_get_acceleration",
        message0: "IMU acceleration %1",
        args0: [
            {
                type: "field_dropdown",
                name: "AXIS",
                options: [
                    ["X (m/s²)", "x"],
                    ["Y (m/s²)", "y"],
                    ["Z (m/s²)", "z"],
                    ["Total (m/s²)", "total"],
                ],
            },
        ],
        output: "Number",
        colour: 200,
        tooltip:
            "Returns linear acceleration in m/s² for the selected axis, or the total magnitude. Returns 0 if no IMU data is available.",
        helpUrl: "",
    },
    {
        type: "imu_get_angular_velocity",
        message0: "IMU angular velocity %1",
        args0: [
            {
                type: "field_dropdown",
                name: "AXIS",
                options: [
                    ["X (rad/s)", "x"],
                    ["Y (rad/s)", "y"],
                    ["Z (rad/s)", "z"],
                ],
            },
        ],
        output: "Number",
        colour: 200,
        tooltip:
            "Returns angular velocity in rad/s for the selected axis. Returns 0 if no IMU data is available.",
        helpUrl: "",
    },
    {
        type: "imu_is_data_available",
        message0: "IMU data available",
        output: "Boolean",
        colour: 200,
        tooltip:
            "Returns true if the IMU has published at least one sample that can be read.",
        helpUrl: "",
    },
    {
        type: "imu_get_data_age",
        message0: "IMU data age (s)",
        output: "Number",
        colour: 200,
        tooltip:
            "Returns the age of the latest IMU sample in seconds. Returns 0 if no IMU data is available.",
        helpUrl: "",
    },
]);
