/** @type {import('jest').Config} */
module.exports = {
  preset: "ts-jest",
  testEnvironment: "node",
  roots: ["<rootDir>"],
  testMatch: ["**/*.test.ts"],
  moduleFileExtensions: ["ts", "js"],
  clearMocks: true,
  moduleNameMapper: {
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
