import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {get_detection_field, start_model, stop_model} from "./model-generators";

type MockGenerator = typeof pythonGenerator & {
    definitions_: Record<string, string>;
};

function createGenerator(values: Record<string, string> = {}): MockGenerator {
    const generator = Object.create(pythonGenerator) as MockGenerator;
    generator.definitions_ = {};
    generator.provideFunction_ = (name: string, code: string | string[]) => {
        const declaration = Array.isArray(code) ? code.join("\n") : code;
        generator.definitions_[`function_${name}`] = declaration.replace(
            generator.FUNCTION_NAME_PLACEHOLDER_,
            name,
        );
        return name;
    };
    generator.valueToCode = (_block: Block, name: string, order: number) => {
        expect(order).toBe(Order.NONE);
        return values[name] ?? "";
    };
    return generator;
}

function blockWithFields(fields: Record<string, string>): Block {
    return {
        getFieldValue: (name: string) => fields[name] ?? null,
    } as unknown as Block;
}

function definitions(generator: MockGenerator): string {
    return Object.values(generator.definitions_).join("\n");
}

describe("model generators", () => {
    it("generates SDK start code with the selected dropdown model", () => {
        const generator = createGenerator();

        expect(
            start_model(
                blockWithFields({MODEL_ID: "hand_tracking"}),
                generator,
            ),
        ).toBe("start_model_with_sdk('hand_tracking')\n");

        const declarations = definitions(generator);
        expect(declarations).toContain("import os");
        expect(declarations).toContain("from pib_sdk import Models");
        expect(declarations).toContain(
            "def start_model_with_sdk(model_id) -> None:",
        );
        expect(declarations).toContain(
            'rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")',
        );
        expect(declarations).toContain(
            "with Models(host=rosbridge_host, port=9090) as models:",
        );
        expect(declarations).toContain("models.start_model(str(model_id))");
    });

    it("generates SDK stop code and falls back to hand_tracking", () => {
        const generator = createGenerator();

        expect(stop_model(blockWithFields({MODEL_ID: ""}), generator)).toBe(
            "stop_model_with_sdk('hand_tracking')\n",
        );

        const declarations = definitions(generator);
        expect(declarations).toContain("import os");
        expect(declarations).toContain("from pib_sdk import Models");
        expect(declarations).toContain(
            "def stop_model_with_sdk(model_id) -> None:",
        );
        expect(declarations).toContain("models.stop_model(str(model_id))");
    });

    it("generates detection access from its value sockets", () => {
        const generator = createGenerator({
            MODEL_ID: "'face'",
            INDEX: "2",
            NAME: "'nose'",
        });

        const [code, order] = get_detection_field(
            blockWithFields({FIELD: "keypoint_z"}),
            generator,
        );

        expect(code).toBe(
            "get_detection_field('face', 2, 'keypoint_z', 'nose')",
        );
        expect(order).toBe(Order.FUNCTION_CALL);
        const declarations = definitions(generator);
        expect(declarations).toContain(
            "from datatypes.msg import DetectionArray",
        );
        expect(declarations).toContain('f"/detections/{model_id}"');
        expect(declarations).toContain(
            'if field in ("keypoint_x", "keypoint_y", "keypoint_z"):',
        );
    });

    it("uses detection defaults when sockets and field are empty", () => {
        const generator = createGenerator();

        const [code] = get_detection_field(blockWithFields({}), generator);

        expect(code).toBe('get_detection_field("hand_tracking", 0, \'\', "")');
    });
});
