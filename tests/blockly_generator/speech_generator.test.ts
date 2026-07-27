import { Block } from "blockly/core/block";
import { pythonGenerator } from "blockly/python";
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
});
