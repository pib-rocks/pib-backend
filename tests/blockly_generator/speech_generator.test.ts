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
  it("generates speech function with voice and text inputs", () => {
    const block = {
      getFieldValue: (field: string) => {
        if (field === "VOICENAME") return "'Hannah'";
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const generator = createMockGenerator('"Hallo PIB"');
    const code = playAudioFromSpeechGenerator(block, generator);

    expect(code).toBe('play_audio_from_speech("Hallo PIB", \'Hannah\')\n');
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("PlayAudioFromSpeech");
    expect(defs).toContain("supertonic_female_de");
    expect(defs).toContain("local Supertonic TTS");
  });

  it("supports Supertonic voice parameters in generated definitions", () => {
    const block = {
      getFieldValue: (field: string) => {
        if (field === "VOICENAME") return "'supertonic_male_de'";
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const generator = createMockGenerator('"Guten Tag"');
    const code = playAudioFromSpeechGenerator(block, generator);

    expect(code).toBe('play_audio_from_speech("Guten Tag", \'supertonic_male_de\')\n');
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("supertonic_male_de");
    expect(defs).toContain("request.gender = \"Male\"");
  });
});
