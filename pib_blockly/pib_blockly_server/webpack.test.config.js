const path = require("path");

const nodeExternals = require("webpack-node-externals");

module.exports = {
  target: "node",
  mode: "development",
  // Same externals as the production build: jsdom (used by blockly's headless mode)
  // resolves its own asset files through `__dirname`, which a bundled copy breaks
  // ("ENOENT ... default-stylesheet.css").
  externals: [nodeExternals()],
  entry: {
    "model-blocks.test": "./test/model-blocks.test.ts",
    "smart-home-blocks.test": "./test/smart-home-blocks.test.ts",
  },
  output: {
    filename: "[name].js",
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
