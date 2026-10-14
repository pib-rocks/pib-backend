import { Block } from "blockly/core/block";
import { pythonGenerator } from "blockly/python";
import { playAudioFromSpeech } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/play-audio-from-speech-block";
import { playAudioFromSpeechGenerator } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/play-audio-from-speech-generator";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
  provideFunction_: (name: string, code: string) => string;
  valueToCode: (block: unknown, name: string, order: number) => string;
};

function createMockGenerator(textValue: string = '"Hallo Welt"'): MockGenerator {
  const definitions: Record<string, string> = {};
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = definitions;
  generator.provideFunction_ = (name: string, code: string) => {
    definitions[`FN_${name}`] = code;
    return name;
  };
  generator.valueToCode = () => textValue;
  return generator;
}

describe("playAudioFromSpeechGenerator", () => {
  it("generates speech function with voice, language and text inputs", () => {
    const block = {
      getFieldValue: (field: string) => {
        if (field === "VOICENAME") return "'F1'";
        if (field === "LANGUAGE") return "'de'";
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const generator = createMockGenerator('"Hallo PIB"');
    const code = playAudioFromSpeechGenerator(block, generator);

    expect(code).toBe('play_audio_from_speech("Hallo PIB", \'F1\', \'de\')\n');
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("PlayAudioFromSpeech");
    expect(defs).toContain("local Supertonic TTS");
  });

  it("supports male voice M3 and language auto in generated definitions", () => {
    const block = {
      getFieldValue: (field: string) => {
        if (field === "VOICENAME") return "'M3'";
        if (field === "LANGUAGE") return "'auto'";
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const generator = createMockGenerator('"Guten Tag"');
    const code = playAudioFromSpeechGenerator(block, generator);

    expect(code).toBe('play_audio_from_speech("Guten Tag", \'M3\', \'auto\')\n');
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("request.gender = voice");
    expect(defs).toContain("request.language = language");
  });

  it("passes a connected variable through the text input", () => {
    const block = {
      getFieldValue: (field: string) => {
        if (field === "VOICENAME") return "'F1'";
        if (field === "LANGUAGE") return "'de'";
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const generator = createMockGenerator("spoken_text");
    const code = playAudioFromSpeechGenerator(block, generator);

    expect(code).toBe("play_audio_from_speech(spoken_text, 'F1', 'de')\n");
  });
});

describe("play_audio_from_speech block definition", () => {
  it("puts the TEXT_INPUT value socket last so it hangs off the right side", () => {
    const json = definitionJson(
      playAudioFromSpeech["play_audio_from_speech"],
    ) as {
      message0: string;
      args0: Array<{ type: string; name: string; check?: string }>;
    };

    const last = json.args0[json.args0.length - 1];
    expect(last).toEqual({
      type: "input_value",
      name: "TEXT_INPUT",
      check: "String",
    });
    expect(json.message0.endsWith(`%${json.args0.length}`)).toBe(true);
    expect(json.args0.map((arg) => arg.name)).toEqual([
      "LANGUAGE",
      "VOICENAME",
      "TEXT_INPUT",
    ]);
  });
});

// createBlockDefinitionsFromJsonArray wraps each JSON definition in an init()
// that calls this.jsonInit(json). Capture the json without a workspace.
function definitionJson(definition: { init: () => void }): unknown {
  let captured: unknown = null;
  definition.init.call({
    jsonInit(json: unknown) {
      captured = json;
    },
  });
  return captured;
}
