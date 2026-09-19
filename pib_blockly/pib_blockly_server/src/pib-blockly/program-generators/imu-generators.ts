import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {
    IMPORT_MATH,
    IMPORT_OS,
    IMPORT_PIB_SDK_IMU,
} from "./util/definitions";
import {
    GET_IMU_ACCELERATION_FUNCTION,
    GET_IMU_ANGULAR_VELOCITY_FUNCTION,
    GET_IMU_DATA_AGE_FUNCTION,
    GET_IMU_IS_DATA_AVAILABLE_FUNCTION,
} from "./util/function-declarations";

const VALID_ACCELERATION_AXES = new Set(["x", "y", "z", "total"]);
const VALID_ANGULAR_VELOCITY_AXES = new Set(["x", "y", "z"]);

function addImuDefinitions(
    generator: typeof pythonGenerator,
    includeMath = false,
) {
    Object.assign(generator.definitions_, {
        IMPORT_OS,
        IMPORT_PIB_SDK_IMU,
        ...(includeMath ? {IMPORT_MATH} : {}),
    });
}

function axisField(block: Block, validAxes: Set<string>): string {
    const axis = String(block.getFieldValue("AXIS") || "x");
    if (!validAxes.has(axis)) {
        throw new Error(`'${axis}' is not a valid value for 'AXIS'.`);
    }
    return axis;
}

export function imu_get_acceleration(
    block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    const axis = axisField(block, VALID_ACCELERATION_AXES);
    addImuDefinitions(generator, true);

    const functionName = generator.provideFunction_(
        "get_imu_acceleration",
        GET_IMU_ACCELERATION_FUNCTION(generator),
    );

    return [`${functionName}(${JSON.stringify(axis)})`, Order.FUNCTION_CALL];
}

export function imu_get_angular_velocity(
    block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    const axis = axisField(block, VALID_ANGULAR_VELOCITY_AXES);
    addImuDefinitions(generator);

    const functionName = generator.provideFunction_(
        "get_imu_angular_velocity",
        GET_IMU_ANGULAR_VELOCITY_FUNCTION(generator),
    );

    return [`${functionName}(${JSON.stringify(axis)})`, Order.FUNCTION_CALL];
}

export function imu_is_data_available(
    _block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    addImuDefinitions(generator);

    const functionName = generator.provideFunction_(
        "get_imu_is_data_available",
        GET_IMU_IS_DATA_AVAILABLE_FUNCTION(generator),
    );

    return [`${functionName}()`, Order.FUNCTION_CALL];
}

export function imu_get_data_age(
    _block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    addImuDefinitions(generator);

    const functionName = generator.provideFunction_(
        "get_imu_data_age",
        GET_IMU_DATA_AGE_FUNCTION(generator),
    );

    return [`${functionName}()`, Order.FUNCTION_CALL];
}
