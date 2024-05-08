const path = require('path');

const nodeExternals = require('webpack-node-externals');

module.exports = {
  target: 'node',
  externals: [nodeExternals()],
  entry: './src/app.ts',
  output: {
    filename: 'pib_blockly_server_bundle.js',
    path: path.resolve(__dirname, 'dist'),
    clean: true,
  },
  module: {
    rules: [
      {
        test: /\.tsx?$/,
        use: 'ts-loader',
        exclude: /node_modules/,
      },
      {
        test: /(blockly\/.*\.js)$/,
        use: [require.resolve('source-map-loader')],
        enforce: 'pre',
      }
    ],
  },
  resolve: {
    extensions: ['.tsx', '.ts', '.js'],
  },
  ignoreWarnings: [/Failed to parse source map/]
}