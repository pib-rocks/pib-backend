import * as Blockly from 'blockly';
import * as http from 'http';
import { pythonGenerator } from './pib-blockly/program-generators/custom-generators';
import { customBlockDefinition } from './pib-blockly/program-blocks/custom-blocks';

function codeVisualToPython(codeVisual: string): string {
    let workspaceContent;;
    try {
        workspaceContent = JSON.parse(codeVisual);
    } catch (error) {
        throw new Error("input is not a well-formed json");
    }
    Blockly.serialization.workspaces.load(workspaceContent, workspace);
    return pythonGenerator.workspaceToCode(workspace);
}

function respond(res: http.ServerResponse<http.IncomingMessage>, code: number, message: string) {
    res.statusCode = code
    res.setHeader('Content-Type', 'text/plain');
    res.end(message);
}

customBlockDefinition();

const workspace = new Blockly.Workspace();
const hostname = '127.0.0.1';
const port = 2442;

const server = http.createServer((req, res) => {
    const buffer: Uint8Array[] = [];
    req.on('data', chunk => buffer.push(chunk));
    req.on('end', () => {
        const codeVisual = Buffer.concat(buffer).toString();
        let codePython: string = "";
        try {
            codePython = codeVisualToPython(codeVisual);
        } catch (error) {
            console.error(`following error occured while compiling input: ${error}.`);
            respond(res, 400, "failed to compile visual-code.")
            return;
        } 
        respond(res, 200, codePython);
    });
});

server.listen(port, hostname, () => {
  console.info(`pibly server is now listening on http://${hostname}:${port}/ ...`);
}); 