import {Block} from "blockly/core/block";
import {
    closeCerebraFullscreenGenerator,
    openCerebraFullscreenGenerator,
    setFaceExpressionGenerator,
    showFaceTextGenerator,
} from "./display-generators";

function blockWithFields(fields: Record<string, string>): Block {
    return {
        getFieldValue: (name: string) => fields[name] ?? null,
    } as unknown as Block;
}

describe("display generators", () => {
    it("generates open_cerebra_fullscreen with SIGTERM cleanup handler", () => {
        const code = openCerebraFullscreenGenerator(
            blockWithFields({URL: "http://localhost"}),
        );
        expect(code).toContain(
            '_pib_publish_string(_pib_display_web_pub, "http://localhost", "display web page")',
        );
        expect(code).toContain(
            "signal.signal(signal.SIGTERM, _pib_on_terminate)",
        );
    });

    it("falls back to http://localhost when URL is blank", () => {
        const code = openCerebraFullscreenGenerator(
            blockWithFields({URL: " "}),
        );
        expect(code).toContain(
            '_pib_publish_string(_pib_display_web_pub, "http://localhost", "display web page")',
        );
    });

    it("generates close_cerebra_fullscreen", () => {
        const code = closeCerebraFullscreenGenerator(blockWithFields({}));
        expect(code).toContain(
            '_pib_publish_string(_pib_display_web_hide_pub, "hide", "display web hide")',
        );
    });

    it("generates set_face_expression and show_face_text", () => {
        const exprCode = setFaceExpressionGenerator(
            blockWithFields({EXPRESSION: "happy"}),
        );
        expect(exprCode).toContain(
            '_pib_publish_string(_pib_expression_pub, "happy", "expression")',
        );

        const textCode = showFaceTextGenerator(
            blockWithFields({TEXT: "Hello pib"}),
        );
        expect(textCode).toContain(
            '_pib_publish_string(_pib_display_text_pub, "Hello pib"[:40], "text")',
        );
    });
});
