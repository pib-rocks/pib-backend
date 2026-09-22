import * as Blockly from "blockly";

const DETECTION_MODEL_ID_INPUT = {
    type: "input_value",
    name: "MODEL_ID",
    check: "String",
};

type ModelOption = [string, string];

const FALLBACK_MODEL_OPTIONS: ModelOption[] = [
    ["hand_tracking", "hand_tracking"],
];
const MODEL_REFRESH_INTERVAL_MS = 5_000;
let cachedModelOptions = FALLBACK_MODEL_OPTIONS;
let requestInFlight = false;
let lastRequestAt = 0;

export function rosbridgeUrl(
    location?: Pick<Location, "protocol" | "hostname">,
): string | null {
    if (typeof window === "undefined" || typeof window.WebSocket === "undefined") {
        return null;
    }

    const configured = (
        window as typeof window & {PIB_ROSBRIDGE_URL?: string}
    ).PIB_ROSBRIDGE_URL;
    if (configured) {
        return configured;
    }

    const currentLocation = location ?? window.location;
    const protocol = currentLocation.protocol === "https:" ? "wss:" : "ws:";
    return `${protocol}//${currentLocation.hostname}:9090`;
}

export function refreshModelOptions(): void {
    const url = rosbridgeUrl();
    const now = Date.now();
    if (
        !url ||
        requestInFlight ||
        now - lastRequestAt < MODEL_REFRESH_INTERVAL_MS
    ) {
        return;
    }

    requestInFlight = true;
    lastRequestAt = now;
    const requestId = `blockly-list-models-${now}`;
    const socket = new window.WebSocket(url);
    const timeout = window.setTimeout(() => socket.close(), 5_000);

    const finish = () => {
        window.clearTimeout(timeout);
        requestInFlight = false;
    };

    socket.onopen = () => {
        socket.send(
            JSON.stringify({
                op: "call_service",
                id: requestId,
                service: "/list_models",
                type: "datatypes/srv/ListModels",
                args: {},
            }),
        );
    };
    socket.onmessage = (event) => {
        let response;
        try {
            response = JSON.parse(String(event.data));
        } catch {
            socket.close();
            return;
        }
        if (response.id !== requestId) {
            return;
        }
        try {
            const models = Array.isArray(response.values?.models)
                ? response.values.models
                : [];
            const options = models
                .map((model: {model_id?: unknown}) => String(model.model_id || ""))
                .filter((modelId: string) => modelId.length > 0)
                .sort()
                .map((modelId: string): ModelOption => [modelId, modelId]);
            if (options.length > 0) {
                cachedModelOptions = options;
            }
        } finally {
            socket.close();
        }
    };
    socket.onerror = () => socket.close();
    socket.onclose = finish;
}

export function getModelDropdownOptions(): ModelOption[] {
    refreshModelOptions();
    return cachedModelOptions;
}

class ModelFieldDropdown extends Blockly.FieldDropdown {
    constructor() {
        super(getModelDropdownOptions);
    }

    override doClassValidation_(newValue: any) {
        // The compiler runs without a browser or ROS connection. Accept the
        // model ID saved by the editor even when it is not in the fallback menu.
        return newValue;
    }
}

const lifecycleBlocks = {
    start_model: {
        init(this: Blockly.Block) {
            refreshModelOptions();
            this.appendDummyInput()
                .appendField("start model")
                .appendField(new ModelFieldDropdown(), "MODEL_ID");
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour(200);
            this.setTooltip(
                "Starts the selected vision model with its registry defaults.",
            );
            this.setHelpUrl("");
        },
    },
    stop_model: {
        init(this: Blockly.Block) {
            refreshModelOptions();
            this.appendDummyInput()
                .appendField("stop model")
                .appendField(new ModelFieldDropdown(), "MODEL_ID");
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour(200);
            this.setTooltip("Stops the selected vision model.");
            this.setHelpUrl("");
        },
    },
};

const detectionBlocks = Blockly.common.createBlockDefinitionsFromJsonArray([
    {
        type: "get_detection_field",
        message0: "detection from model %1 item %2 field %3 name %4",
        args0: [
            DETECTION_MODEL_ID_INPUT,
            {
                type: "input_value",
                name: "INDEX",
                check: "Number",
                extensions: "number_validation",
            },
            {
                type: "field_dropdown",
                name: "FIELD",
                options: [
                    ["label", "label"],
                    ["score", "score"],
                    ["x min", "x_min"],
                    ["y min", "y_min"],
                    ["x max", "x_max"],
                    ["y max", "y_max"],
                    ["keypoint x", "keypoint_x"],
                    ["keypoint y", "keypoint_y"],
                    ["keypoint z", "keypoint_z"],
                    ["scalar value", "scalar_values"],
                ],
            },
            {
                type: "input_value",
                name: "NAME",
                check: "String",
            },
        ],
        output: null,
        colour: 200,
        tooltip:
            "Reads a field from the latest detection. Name selects a named keypoint or scalar; it is ignored for label, score, and bounding-box fields.",
        helpUrl: "",
    },
]);

export const modelBlocks = {
    ...lifecycleBlocks,
    ...detectionBlocks,
};
