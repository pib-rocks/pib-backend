FROM node:18.20.2

WORKDIR /app

COPY . .

RUN npm install
RUN npm run build

CMD ["node", "/app/dist/pib_blockly_server_bundle.js"]
