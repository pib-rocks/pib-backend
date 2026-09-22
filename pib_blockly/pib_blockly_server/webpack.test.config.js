const path = require("path");

module.exports = {
  target: "node",
  mode: "development",
  entry: "./test/model-blocks.test.ts",
  output: {
    filename: "model-blocks.test.js",
    path: path.resolve(__dirname, "dist-test"),
    clean: true,
  },
  module: {
    rules: [
      {
        test: /\.tsx?$/,
        use: "ts-loader",
        exclude: /node_modules/,
      },
    ],
  },
  resolve: {
    extensions: [".tsx", ".ts", ".js"],
  },
};
