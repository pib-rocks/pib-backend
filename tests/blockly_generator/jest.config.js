/** @type {import('jest').Config} */
module.exports = {
  preset: "ts-jest",
  testEnvironment: "node",
  roots: ["<rootDir>"],
  testMatch: ["**/*.test.ts"],
  moduleFileExtensions: ["ts", "js"],
  clearMocks: true,
  // The blocks/generators under test live outside <rootDir> (pib_blockly/...), so Node's
  // upward node_modules lookup from those files never reaches this package. Resolve every
  // dependency against <rootDir>/node_modules so a single Blockly copy is used regardless
  // of cwd or of stray node_modules trees elsewhere in the repo.
  moduleDirectories: ["node_modules", "<rootDir>/node_modules"],
  moduleNameMapper: {
    "^blockly$": "<rootDir>/node_modules/blockly",
    "^blockly/(.*)$": "<rootDir>/node_modules/blockly/$1",
  },
  transform: {
    "^.+\\.tsx?$": [
      "ts-jest",
      {
        tsconfig: "<rootDir>/tsconfig.json",
        isolatedModules: true,
        diagnostics: false,
      },
    ],
  },
};
