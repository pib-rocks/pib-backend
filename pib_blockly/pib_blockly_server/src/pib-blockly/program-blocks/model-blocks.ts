import * as Blockly from "blockly";

type ModelOption = [string, string];

const FALLBACK_MODEL_OPTIONS: ModelOption[] = [
    ["hand_tracking", "hand_tracking"],
];
export const STOP_ALL_MODELS_VALUE = "__all__";
const STOP_ALL_MODELS_OPTION: ModelOption = ["All", STOP_ALL_MODELS_VALUE];
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

export function getStopModelDropdownOptions(): ModelOption[] {
    return [STOP_ALL_MODELS_OPTION, ...getModelDropdownOptions()];
}

class ModelFieldDropdown extends Blockly.FieldDropdown {
    constructor(menuGenerator: () => ModelOption[] = getModelDropdownOptions) {
        super(menuGenerator);
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
                .appendField(
                    new ModelFieldDropdown(getStopModelDropdownOptions),
                    "MODEL_ID",
                );
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour(200);
            this.setTooltip(
                "Stops the selected vision model, or all active models of this program when All is chosen. Models started elsewhere are left running.",
            );
            this.setHelpUrl("");
        },
    },
};

const detectionBlocks = Blockly.common.createBlockDefinitionsFromJsonArray([
    {
        type: "get_face_detections",
        message0: "latest face detections",
        output: "Array",
        colour: 200,
        tooltip:
            "Returns the latest face detections as a list. Each face is [label, score, x_min, y_min, x_max, y_max] (indices 0 through 5).",
        helpUrl: "",
    },
    {
        type: "get_object_detections",
        message0: "latest object detections",
        output: "Array",
        colour: 200,
        tooltip:
            "Returns the latest object detections as a list. Each object is [label, score, x_min, y_min, x_max, y_max] (indices 0 through 5).",
        helpUrl: "",
    },
    {
        type: "get_qr_detections",
        message0: "latest QR detections",
        output: "Array",
        colour: 200,
        tooltip:
            "Returns the latest QR detections as a list. Each code is [label, score, x_min, y_min, x_max, y_max] (indices 0 through 5).",
        helpUrl: "",
    },
    {
        type: "get_emotion_detections",
        message0: "latest emotion detections",
        output: "Array",
        colour: 200,
        tooltip:
            "Returns the latest emotion detections as a list. Each face is [label, score, x_min, y_min, x_max, y_max] (indices 0 through 5).",
        helpUrl: "",
    },
    {
        type: "get_head_pose_detections",
        message0: "latest head pose detections",
        output: "Array",
        colour: 200,
        tooltip:
            "Returns the latest head pose detections as a list. Each face is [label, score, x_min, y_min, x_max, y_max, yaw_deg, pitch_deg, roll_deg] (indices 0 through 8).",
        helpUrl: "",
    },
]);

export const modelBlocks = {
    ...lifecycleBlocks,
    ...detectionBlocks,
};
