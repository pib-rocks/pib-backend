FROM node:22.23.3

WORKDIR /app

COPY . .

RUN npm install
RUN npm run build

CMD ["node", "/app/dist/pib_blockly_server_bundle.js"]
