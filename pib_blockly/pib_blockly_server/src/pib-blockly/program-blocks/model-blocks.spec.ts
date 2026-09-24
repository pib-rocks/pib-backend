import * as Blockly from "blockly";
import {
    getModelDropdownOptions,
    modelBlocks,
    refreshModelOptions,
    rosbridgeUrl,
    STOP_ALL_MODELS_VALUE,
} from "./model-blocks";

interface RosbridgeRequest {
    op: string;
    id: string;
    service: string;
    type: string;
    args: object;
}

class FakeWebSocket {
    static instances: FakeWebSocket[] = [];

    readonly sent: RosbridgeRequest[] = [];
    closed = false;
    onopen: (() => void) | null = null;
    onmessage: ((event: {data: string}) => void) | null = null;
    onerror: (() => void) | null = null;
    onclose: (() => void) | null = null;

    constructor(readonly url: string) {
        FakeWebSocket.instances.push(this);
    }

    send(data: string): void {
        this.sent.push(JSON.parse(data) as RosbridgeRequest);
    }

    open(): void {
        this.onopen?.();
    }

    receive(message: object): void {
        this.onmessage?.({data: JSON.stringify(message)});
    }

    receiveRaw(data: string): void {
        this.onmessage?.({data});
    }

    fail(): void {
        this.onerror?.();
    }

    close(): void {
        if (this.closed) {
            return;
        }
        this.closed = true;
        this.onclose?.();
    }
}

type WindowWithRosbridgeUrl = typeof window & {
    PIB_ROSBRIDGE_URL?: string;
};

describe("model blocks", () => {
    let originalWebSocket: typeof WebSocket;
    let testTime = 0;
    const originalBlockDefinitions = new Map<
        string,
        (typeof Blockly.Blocks)[string] | undefined
    >();

    beforeAll(() => {
        for (const type of Object.keys(modelBlocks)) {
            originalBlockDefinitions.set(type, Blockly.Blocks[type]);
        }
        Object.assign(Blockly.Blocks, modelBlocks);
    });

    beforeEach(() => {
        originalWebSocket = window.WebSocket;
        FakeWebSocket.instances = [];
        window.WebSocket = FakeWebSocket as unknown as typeof WebSocket;
        delete (window as WindowWithRosbridgeUrl).PIB_ROSBRIDGE_URL;
        jasmine.clock().install();
        testTime = Math.max(testTime + 10_000, Date.now() + 10_000);
        jasmine.clock().mockDate(new Date(testTime));
    });

    afterEach(() => {
        for (const socket of FakeWebSocket.instances) {
            socket.close();
        }
        jasmine.clock().uninstall();
        window.WebSocket = originalWebSocket;
        delete (window as WindowWithRosbridgeUrl).PIB_ROSBRIDGE_URL;
    });

    afterAll(() => {
        for (const type of Object.keys(modelBlocks)) {
            const originalDefinition = originalBlockDefinitions.get(type);
            if (originalDefinition) {
                Blockly.Blocks[type] = originalDefinition;
            } else {
                delete Blockly.Blocks[type];
            }
        }
    });

    it("builds rosbridge URLs for HTTP and HTTPS and honors configuration", () => {
        expect(rosbridgeUrl({protocol: "http:", hostname: "robot.local"})).toBe(
            "ws://robot.local:9090",
        );
        expect(
            rosbridgeUrl({protocol: "https:", hostname: "robot.local"}),
        ).toBe("wss://robot.local:9090");

        (window as WindowWithRosbridgeUrl).PIB_ROSBRIDGE_URL =
            "wss://configured.example/ros";
        expect(
            rosbridgeUrl({protocol: "http:", hostname: "ignored.local"}),
        ).toBe("wss://configured.example/ros");
    });

    it("returns null and does not refresh without WebSocket support", () => {
        window.WebSocket = undefined as unknown as typeof WebSocket;

        expect(rosbridgeUrl()).toBeNull();
        refreshModelOptions();
        expect(FakeWebSocket.instances).toEqual([]);
    });

    it("keeps lifecycle model IDs as dropdowns", () => {
        const workspace = new Blockly.Workspace();
        const start = workspace.newBlock("start_model");
        const stop = workspace.newBlock("stop_model");

        expect(start.getField("MODEL_ID")).toEqual(
            jasmine.any(Blockly.FieldDropdown),
        );
        expect(start.getInput("MODEL_ID")).toBeNull();
        expect(stop.getField("MODEL_ID")).toEqual(
            jasmine.any(Blockly.FieldDropdown),
        );
        expect(stop.getInput("MODEL_ID")).toBeNull();
        expect(
            (start.getField("MODEL_ID") as Blockly.FieldDropdown).getOptions(
                false,
            ),
        ).toEqual([["hand_tracking", "hand_tracking"]]);
        expect(
            (stop.getField("MODEL_ID") as Blockly.FieldDropdown).getOptions(
                false,
            ),
        ).toEqual([
            ["All", STOP_ALL_MODELS_VALUE],
            ["hand_tracking", "hand_tracking"],
        ]);
        start.setFieldValue("saved_custom_model", "MODEL_ID");
        expect(start.getFieldValue("MODEL_ID")).toBe("saved_custom_model");

        const socket = FakeWebSocket.instances[0];
        socket.open();
        expect(socket.sent.length).toBe(1);
        expect(socket.sent[0]).toEqual({
            op: "call_service",
            id: `blockly-list-models-${testTime}`,
            service: "/list_models",
            type: "datatypes/srv/ListModels",
            args: {},
        });

        socket.receive({id: "another-request", values: {models: []}});
        expect(socket.closed).toBeFalse();
        socket.receive({
            id: socket.sent[0].id,
            values: {
                models: [
                    {model_id: "pose"},
                    {model_id: ""},
                    {model_id: "face"},
                ],
            },
        });

        expect(socket.closed).toBeTrue();
        expect(getModelDropdownOptions()).toEqual([
            ["face", "face"],
            ["pose", "pose"],
        ]);
        workspace.dispose();
    });

    it("deduplicates in-flight requests and caches refreshes for five seconds", () => {
        refreshModelOptions();
        refreshModelOptions();
        expect(FakeWebSocket.instances.length).toBe(1);

        FakeWebSocket.instances[0].close();
        refreshModelOptions();
        expect(FakeWebSocket.instances.length).toBe(1);

        jasmine.clock().tick(5_000);
        refreshModelOptions();
        expect(FakeWebSocket.instances.length).toBe(2);
    });

    it("closes sockets on malformed responses, errors, and timeout", () => {
        refreshModelOptions();
        const malformed = FakeWebSocket.instances[0];
        malformed.receiveRaw("not json");
        expect(malformed.closed).toBeTrue();

        jasmine.clock().tick(5_000);
        refreshModelOptions();
        const failed = FakeWebSocket.instances[1];
        failed.fail();
        expect(failed.closed).toBeTrue();

        jasmine.clock().tick(5_000);
        refreshModelOptions();
        const timedOut = FakeWebSocket.instances[2];
        jasmine.clock().tick(5_000);
        expect(timedOut.closed).toBeTrue();
    });
});
